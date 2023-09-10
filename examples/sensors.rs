#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use embassy_nrf as _;
use adafruit_clue_embassy::{
    output_pin,
    nrf_default_config,
    TFT_XSIZE,
    TFT_YSIZE,
    tft_cs,
    tft_sck,
    tft_mosi,
    tft_dc,
    tft_reset,
    tft_backlight,
    white_led,
    sensors_i2c_scl,
    sensors_i2c_sda,
};
use adafruit_clue_embassy;
use embassy_time::{Delay, Duration, Timer};
use embassy_executor::Spawner;
use embassy_nrf::{bind_interrupts, peripherals, spim, twim};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_sync::blocking_mutex::NoopMutex;

use display_interface_spi::SPIInterfaceNoCS;

use embedded_graphics::{
    mono_font::{ascii::FONT_7X13, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
    primitives::*,
};

use mipidsi::Builder;

// accelerometer/Gyro
//use lsm6ds33::Lsm6ds33;
//use lsm6ds33::{AccelerometerBandwidth, AccelerometerOutput, AccelerometerScale};
//use lsm6ds33::{GyroscopeFullScale, GyroscopeOutput};

// proximity/gesture/color
//use apds9960::Apds9960;

// pressure/temperature
//use bmp280_rs;

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

    let mut _tft_backlight = output_pin(tft_backlight!(nrf_periph), true);
    let _tft_cs_pin = output_pin(tft_cs!(nrf_periph), false);
    let tft_spi_config = adafruit_clue_embassy::tft_spi_config();
    let tft_spi = spim::Spim::new_txonly(
        nrf_periph.SPI3,
        Irqs,
        tft_sck!(nrf_periph),
        tft_mosi!(nrf_periph),
        tft_spi_config);
    let tft_spi_interface =
        SPIInterfaceNoCS::new(tft_spi, output_pin(tft_dc!(nrf_periph), false));
    let mut tft_delay = Delay {};
    let mut display = Builder::st7789(tft_spi_interface)
        .with_display_size(TFT_XSIZE, TFT_YSIZE)
        .with_orientation(mipidsi::Orientation::LandscapeInverted(true))
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(&mut tft_delay,
              Some(output_pin(tft_reset!(nrf_periph), false)))
        .unwrap();
    display.clear(Rgb565::BLACK).unwrap();

    let sensors_twim_config = adafruit_clue_embassy::sensors_twim_config();
    let sensors_twim =
        twim::Twim::new(nrf_periph.TWISPI1,
                        Irqs,
                        sensors_i2c_sda!(nrf_periph),
                        sensors_i2c_scl!(nrf_periph),
                        sensors_twim_config);
    let sensors_twim_bus = NoopMutex::new(RefCell::new(sensors_twim));
    let humidity_twim = I2cDevice::new(&sensors_twim_bus);
    let mut humidity = sht3x::SHT3x::new(humidity_twim, sht3x::Address::Low);

    let magnet_twim = I2cDevice::new(&sensors_twim_bus);
    let mut magnet = lis3mdl::Lis3mdl::new(magnet_twim, lis3mdl::Address::Addr1C).unwrap();

    let mut led = output_pin(white_led!(nrf_periph), false);

    let clear_text_rect = |x, y| {
        Rectangle::new(Point::new(x, y), Size::new(240, 20))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
    };
    let mut humidity_delay = Delay {};
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;

        // humidity
        let h = humidity
            .measure(sht3x::Repeatability::High, &mut humidity_delay)
            .unwrap();
        let mut humstring: String<64> = String::new();
        write!(humstring, "HUMID {} TEMP {}", h.humidity, h.temperature).unwrap();
        clear_text_rect(0, 120).draw(&mut display).unwrap();
        text(0, 120, &humstring).draw(&mut display).unwrap();

        // magnetometer
        let xyz = magnet.get_mag_axes_mgauss().unwrap();
        let mut magstring: String<64> = String::new();
        write!(magstring, "MAG ({},{},{})", xyz.x, xyz.y, xyz.z).unwrap();
        clear_text_rect(0, 140).draw(&mut display).unwrap();
        text(0, 140, &magstring).draw(&mut display).unwrap();

        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

fn text(x: i32, y: i32, s: &str) -> Text<MonoTextStyle<Rgb565>> {
    let text_style = MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE);
    Text::new(s, Point::new(x + 10, y + 10), text_style)
}

#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
