#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use adafruit_clue_embassy;
use adafruit_clue_embassy::{
    lcd_backlight, lcd_cs, lcd_dc, lcd_mosi, lcd_reset, lcd_sck, nrf_default_config, output_pin,
    sensors_i2c_scl, sensors_i2c_sda, LCD_XSIZE, LCD_YSIZE,
};
use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_nrf as _;
use embassy_nrf::{bind_interrupts, peripherals, spim, twim};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Delay, Duration, Timer};

use display_interface_spi::SPIInterfaceNoCS;

use embedded_graphics::{
    mono_font::{ascii::FONT_7X13, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::*,
    text::Text,
};

use mipidsi::Builder;

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

use core::fmt::Write;
use heapless::String;

bind_interrupts!(struct Irqs {
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<peripherals::TWISPI1>;
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let nrf_config = nrf_default_config(false);
    let nrf_periph = embassy_nrf::init(nrf_config);

    // Fire up the TFT.
    let mut _lcd_backlight = output_pin(lcd_backlight!(nrf_periph), true);
    let _lcd_cs_pin = output_pin(lcd_cs!(nrf_periph), false);
    let lcd_spi_config = adafruit_clue_embassy::lcd_spi_config();
    let lcd_spi = spim::Spim::new_txonly(
        nrf_periph.SPI3,
        Irqs,
        lcd_sck!(nrf_periph),
        lcd_mosi!(nrf_periph),
        lcd_spi_config,
    );
    let lcd_spi_interface = SPIInterfaceNoCS::new(lcd_spi, output_pin(lcd_dc!(nrf_periph), false));
    let mut lcd_delay = Delay {};
    let mut display = Builder::st7789(lcd_spi_interface)
        .with_display_size(LCD_XSIZE, LCD_YSIZE)
        .with_orientation(mipidsi::Orientation::LandscapeInverted(true))
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(
            &mut lcd_delay,
            Some(output_pin(lcd_reset!(nrf_periph), false)),
        )
        .unwrap();
    display.clear(Rgb565::BLACK).unwrap();

    // Make a shared TWIM bus for the sensors.
    let sensors_twim_config = adafruit_clue_embassy::sensors_twim_config();
    let sensors_twim = twim::Twim::new(
        nrf_periph.TWISPI1,
        Irqs,
        sensors_i2c_sda!(nrf_periph),
        sensors_i2c_scl!(nrf_periph),
        sensors_twim_config,
    );
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
        // prox/rgb/gesture
        let rgb = prox_rgb_gesture.read_light().unwrap();
        let mut rgbstring: String<64> = String::new();
        write!(
            rgbstring,
            "RGB ({},{},{},{})",
            rgb.red, rgb.green, rgb.blue, rgb.clear
        )
        .unwrap();
        print_text(&mut display, 80, &rgbstring);
        let circle_color = Rgb565::new(rgb.red as u8, rgb.green as u8, rgb.blue as u8);
        draw_circle(&mut display, 0, 35, 40, circle_color);

        // gyro/accel
        let (x, y, z) = gyro_accel.read_gyro().unwrap();
        let mut gyrostring: String<64> = String::new();
        write!(gyrostring, "GYRO ({:.4},{:.4},{:.4})", x, y, z).unwrap();
        print_text(&mut display, 100, &gyrostring);

        // temp/pressure
        let temp = temp_pressure
            .read_temperature(&mut temp_pressure_twim)
            .unwrap();
        let pressure = temp_pressure
            .read_pressure(&mut temp_pressure_twim)
            .unwrap();
        let mut tempstring: String<64> = String::new();
        write!(tempstring, "TEMP {}", temp).unwrap();
        print_text(&mut display, 120, &tempstring);
        let mut pressstring: String<64> = String::new();
        write!(pressstring, "PRESSURE {}", pressure).unwrap();
        print_text(&mut display, 140, &pressstring);

        // humidity
        let h = humidity
            .measure(sht3x::Repeatability::High, &mut humidity_delay)
            .unwrap();
        let mut humstring: String<64> = String::new();
        write!(humstring, "HUMID {} TEMP {}", h.humidity, h.temperature).unwrap();
        print_text(&mut display, 160, &humstring);

        // magnetometer
        let xyz = magnet.get_mag_axes_mgauss().unwrap();
        let mut magstring: String<64> = String::new();
        write!(magstring, "MAG ({},{},{})", xyz.x, xyz.y, xyz.z).unwrap();
        print_text(&mut display, 180, &magstring);

        Timer::after(Duration::from_millis(500)).await;
    }
}

fn print_text<D>(disp: &mut D, y: i32, s: &str)
where
    D: embedded_graphics::draw_target::DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    Rectangle::new(Point::new(0, y), Size::new(240, 20))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(disp)
        .unwrap();
    Text::new(
        s,
        Point::new(10, y + 10),
        MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE),
    )
    .draw(disp)
    .unwrap();
}

fn draw_circle<D>(disp: &mut D, x: i32, y: i32, d: u32, c: Rgb565)
where
    D: embedded_graphics::draw_target::DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    Circle::new(Point::new(x, y), d)
        .into_styled(PrimitiveStyle::with_fill(c))
        .draw(disp)
        .unwrap();
}

#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
