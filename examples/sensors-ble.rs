#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use adafruit_clue_embassy;
use adafruit_clue_embassy::{
    lcd_backlight, nrf_default_config, output_pin, red_led, sensors_i2c_scl, sensors_i2c_sda,
    white_led,
};
use core::cell::RefCell;
use core::mem;

use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_nrf as _;
use embassy_nrf::peripherals::TWISPI1;
use embassy_nrf::twim::Twim;
use embassy_nrf::{bind_interrupts, twim};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer};

use futures::future::{select, Either};

use futures::pin_mut;
use nrf_softdevice::ble::{gatt_server, peripheral, Connection};
use nrf_softdevice::{raw, Softdevice};

// accelerometer/Gyro
use lsm6ds33::Lsm6ds33;
use lsm6ds33::{AccelerometerBandwidth, AccelerometerOutput, AccelerometerScale};
use lsm6ds33::{GyroscopeFullScale, GyroscopeOutput};

// proximity/gesture/color
use apds9960::Apds9960;

// pressure/temperature
use bmp280_rs;

// magnetometer
use lis3mdl;

// humidity
use sht3x;

bind_interrupts!(struct Irqs {
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<TWISPI1>;
});

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[derive(Default)]
struct SensorValues {
    humid: i16,
    temp: i16,
    pressure: i16,
}

static SENSOR_VALUES: Signal<ThreadModeRawMutex, SensorValues> = Signal::new();

#[embassy_executor::task]
async fn read_sensors(sensors_twim: Twim<'static, TWISPI1>) {
    // Make a shared TWIM bus for the sensors.
    let sensors_twim_bus = NoopMutex::new(RefCell::new(sensors_twim));

    // Start up the prox/rgb/gesture device - just using the RGB right now.
    let prox_rgb_gesture_twim = I2cDevice::new(&sensors_twim_bus);
    let mut prox_rgb_gesture = Apds9960::new(prox_rgb_gesture_twim);
    prox_rgb_gesture.enable().unwrap();
    prox_rgb_gesture.enable_light().unwrap();

    // Start up the gyro/accelerometer sensor.
    let gyro_accel_twim = I2cDevice::new(&sensors_twim_bus);
    let mut gyro_accel =
        Lsm6ds33::new(gyro_accel_twim, adafruit_clue_embassy::I2C_GYROACCEL).unwrap();
    gyro_accel
        .set_accelerometer_scale(AccelerometerScale::G02)
        .unwrap();
    gyro_accel
        .set_accelerometer_bandwidth(AccelerometerBandwidth::Freq100)
        .unwrap();
    gyro_accel
        .set_accelerometer_output(AccelerometerOutput::Rate104)
        .unwrap();
    gyro_accel
        .set_gyroscope_scale(GyroscopeFullScale::Dps245)
        .unwrap();
    gyro_accel
        .set_gyroscope_output(GyroscopeOutput::Rate104)
        .unwrap();
    gyro_accel.set_low_power_mode(false).unwrap();

    // Start up the temperature/pressure sensor.
    let temp_pressure_config = bmp280_rs::Config {
        measurement_standby_time_millis: Some(
            bmp280_rs::MeasurementStandbyTimeMillis::ZeroPointFive,
        ),
        pressure_oversampling: bmp280_rs::PressureOversampling::Four,
        temperature_oversampling: bmp280_rs::TemperatureOversampling::Four,
        iir_filter: bmp280_rs::IIRFilterCoefficient::Four,
    };
    let mut temp_pressure_twim = I2cDevice::new(&sensors_twim_bus);
    let temp_pressure_sleep = bmp280_rs::BMP280::new(
        &mut temp_pressure_twim,
        bmp280_rs::I2CAddress::SdoPulledUp,
        temp_pressure_config,
    )
    .unwrap();
    let mut temp_pressure = temp_pressure_sleep
        .into_normal_mode(&mut temp_pressure_twim)
        .unwrap();

    let humidity_twim = I2cDevice::new(&sensors_twim_bus);
    let mut humidity = sht3x::SHT3x::new(humidity_twim, sht3x::Address::Low);

    let magnet_twim = I2cDevice::new(&sensors_twim_bus);
    let mut magnet = lis3mdl::Lis3mdl::new(magnet_twim, lis3mdl::Address::Addr1C).unwrap();

    let mut humidity_delay = Delay {};
    loop {
        let mut sensor_values = SensorValues::default();
        // prox/rgb/gesture
        let _rgb = prox_rgb_gesture.read_light().unwrap();

        // gyro/accel
        let (_x, _y, _z) = gyro_accel.read_gyro().unwrap();

        // temp/pressure
        let temp = temp_pressure
            .read_temperature(&mut temp_pressure_twim)
            .unwrap();
        sensor_values.temp = temp as i16;
        let pressure = temp_pressure
            .read_pressure(&mut temp_pressure_twim)
            .unwrap();
        sensor_values.pressure = pressure as i16;

        // humidity
        let h = humidity
            .measure(sht3x::Repeatability::High, &mut humidity_delay)
            .unwrap();
        sensor_values.humid = h.humidity as i16;

        // magnetometer
        let _xyz = magnet.get_mag_axes_mgauss().unwrap();

        SENSOR_VALUES.signal(sensor_values);
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[nrf_softdevice::gatt_service(uuid = "181A")]
struct EnvironmentalSensingService {
    #[characteristic(uuid = "2A6D", read, notify)]
    pressure: i16,
    #[characteristic(uuid = "2A6E", read, notify)]
    temperature: i16,
    #[characteristic(uuid = "2A6F", read, notify)]
    humidity: i16,
}

#[nrf_softdevice::gatt_server]
struct Server {
    env: EnvironmentalSensingService,
}

async fn notify_sensor_readings<'a>(server: &'a Server, connection: &'a Connection) {
    loop {
        // Try and notify the connected client of the current humidity value.
        let sensor_values = SENSOR_VALUES.wait().await;
        match server.env.humidity_notify(connection, &sensor_values.humid) {
            Ok(_) => (),
            Err(_) => server.env.humidity_set(&sensor_values.humid).unwrap(),
        };
        match server
            .env
            .temperature_notify(connection, &sensor_values.temp)
        {
            Ok(_) => (),
            Err(_) => server.env.temperature_set(&sensor_values.temp).unwrap(),
        };
        match server
            .env
            .pressure_notify(connection, &sensor_values.pressure)
        {
            Ok(_) => (),
            Err(_) => server.env.pressure_set(&sensor_values.pressure).unwrap(),
        };
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let nrf_config = nrf_default_config(true);
    let nrf_periph = embassy_nrf::init(nrf_config);
    let mut white = output_pin(white_led!(nrf_periph), false);
    let mut red = output_pin(red_led!(nrf_periph), false);
    // Turn off the LCD backlight since we aren't using it.
    output_pin(lcd_backlight!(nrf_periph), false);

    let mut sd_config = adafruit_clue_embassy::nrf_softdevice_default_config();
    let sd = Softdevice::enable(&sd_config);

    sd_config.gap_device_name =
        // XXX maybe make a macro for this?
        Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"AdafruitClue" as *const u8 as _,
            current_len: 12,
            max_len: 12,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        });

    let server = Server::new(sd).unwrap();
    spawner.spawn(softdevice_task(sd)).unwrap();

    let sensors_twim_config = adafruit_clue_embassy::sensors_twim_config();
    let sensors_twim = twim::Twim::new(
        nrf_periph.TWISPI1,
        Irqs,
        sensors_i2c_sda!(nrf_periph),
        sensors_i2c_scl!(nrf_periph),
        sensors_twim_config,
    );
    spawner.spawn(read_sensors(sensors_twim)).unwrap();

    //raw::BLE_GAP_AD_TYPE
    #[rustfmt::skip]
    let adv_data = &[
        0x02, raw::BLE_GAP_AD_TYPE_FLAGS as u8, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        0x03, raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8, 0x09, 0x18,
        0x0d, raw::BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME as u8, b'A', b'd', b'a', b'f', b'r', b'u', b'i', b't', b'C', b'l', b'u', b'e',
    ];
    #[rustfmt::skip]
    let scan_data = &[
        0x03, raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8, 0x09, 0x18,
    ];

    loop {
        red.set_low();
        white.set_high();

        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data,
            scan_data,
        };
        let conn = peripheral::advertise_connectable(sd, adv, &config)
            .await
            .unwrap();
        red.set_high();
        white.set_low();

        let gatt_fut = gatt_server::run(&conn, &server, |_| {});
        let notify_fut = notify_sensor_readings(&server, &conn);
        pin_mut!(gatt_fut);
        pin_mut!(notify_fut);
        let _ = match select(gatt_fut, notify_fut).await {
            Either::Left((_, _)) => {
                //info!("Connection dropped!");
            }
            Either::Right((_, _)) => {
                //info!("Sensor notification failed!");
            }
        };
    }
}

#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
