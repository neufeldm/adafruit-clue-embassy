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
use portable_atomic::AtomicU32;

use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_nrf as _;
use embassy_nrf::peripherals::TWISPI1;
use embassy_nrf::{bind_interrupts, twim};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Delay, Duration, Timer, with_timeout};

use nrf_softdevice::ble::peripheral;
use nrf_softdevice::ble::advertisement_builder::{AdvertisementDataType, Flag, LegacyAdvertisementBuilder};
use nrf_softdevice::{raw, Softdevice};

// pressure/temperature
use bmp280_rs;

// humidity
use sht3x;

bind_interrupts!(struct Irqs {
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<TWISPI1>;
});

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

static READ_INTERVAL_MS: AtomicU32 = AtomicU32::new(500);

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

    spawner.spawn(softdevice_task(sd)).unwrap();

    let sensors_twim_config = adafruit_clue_embassy::sensors_twim_config();
    let sensors_twim = twim::Twim::new(
        nrf_periph.TWISPI1,
        Irqs,
        sensors_i2c_sda!(nrf_periph),
        sensors_i2c_scl!(nrf_periph),
        sensors_twim_config,
    );
    // Make a shared TWIM bus for the sensors.
    let sensors_twim_bus = NoopMutex::new(RefCell::new(sensors_twim));

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

    let mut humidity_delay = Delay {};
    loop {
        red.set_high();
        white.set_low();

        // temp/pressure
        let temp = temp_pressure
            .read_temperature(&mut temp_pressure_twim)
            .unwrap();
        let pressure = temp_pressure
            .read_pressure(&mut temp_pressure_twim)
            .unwrap();

        // humidity
        let humid = humidity
            .measure(sht3x::Repeatability::High, &mut humidity_delay)
            .unwrap()
            .humidity;
        let mut beacon_bytes = [0x00; 14];
        // Unknown manufacturer in bytes 0 and 1
        beacon_bytes[0..2].copy_from_slice(&[0xFF; 2]);
        // Rest of bytes are sensor values, little-endian
        beacon_bytes[2..6].copy_from_slice(&temp.to_le_bytes());
        beacon_bytes[6..10].copy_from_slice(&pressure.to_le_bytes());
        beacon_bytes[10..14].copy_from_slice(&humid.to_le_bytes());
        let adv_data = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::LE_Only])
        .raw(AdvertisementDataType::MANUFACTURER_SPECIFIC_DATA, &beacon_bytes)
        .build();
        let adv = peripheral::NonconnectableAdvertisement::NonscannableUndirected {
            adv_data: &adv_data,
        };
        let ble_config = peripheral::Config::default();
        match with_timeout(
            Duration::from_secs(5),
            peripheral::advertise(sd, adv, &ble_config),
        )
        .await
        {
            Ok(Err(_)) => panic!(),
            _ => {}
        }
        red.set_low();
        white.set_high();
        Timer::after(Duration::from_millis(
            READ_INTERVAL_MS
                .load(portable_atomic::Ordering::Relaxed)
                .into(),
        ))
        .await;
    }
}

#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
