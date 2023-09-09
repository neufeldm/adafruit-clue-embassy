#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_nrf as _;
use embassy_adafruit_clue::{output_pin, nrf_default_config, TFT_XSIZE, TFT_YSIZE, tft_cs, tft_sck, tft_mosi, tft_dc, tft_reset, tft_backlight, white_led };
use embassy_adafruit_clue;
use embassy_time::{Delay, Duration, Timer};
use embassy_executor::Spawner;
use embassy_nrf::{bind_interrupts, peripherals, spim};
use display_interface_spi::SPIInterfaceNoCS;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
    primitives::*,
};
use mipidsi::Builder;

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let nrf_config = nrf_default_config(false);
    let nrf_periph = embassy_nrf::init(nrf_config);
    let mut _backlight = output_pin(tft_backlight!(nrf_periph), true);
    let _cs_pin = output_pin(tft_cs!(nrf_periph), false);

    let spi_config = embassy_adafruit_clue::tft_spi_config();
    let spi = spim::Spim::new_txonly(
        nrf_periph.SPI3,
        Irqs,
        tft_sck!(nrf_periph),
        tft_mosi!(nrf_periph),
        spi_config);

    let di =
        SPIInterfaceNoCS::new(spi, output_pin(tft_dc!(nrf_periph), false));

    let mut delay = Delay {};
    let mut display = Builder::st7789(di)
        .with_display_size(TFT_XSIZE, TFT_YSIZE)
        .with_orientation(mipidsi::Orientation::LandscapeInverted(true))
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(&mut delay,
              Some(output_pin(tft_reset!(nrf_periph), false)))
        .unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    Text::new("Hello Kitty!", Point::new(10, 210), text_style)
        .draw(&mut display)
        .unwrap();
    let make_circle = |x, y, color: Rgb565| {
        Circle::new(Point::new(x, y), 64).into_styled(PrimitiveStyle::with_fill(color))
    };
    let mut circle_colors = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE, Rgb565::WHITE];
    let mut led = output_pin(white_led!(nrf_periph), false);
    loop {
        make_circle(128, 64, circle_colors[0])
            .draw(&mut display)
            .unwrap();
        make_circle(64, 64, circle_colors[1])
            .draw(&mut display)
            .unwrap();
        make_circle(64, 128, circle_colors[2])
            .draw(&mut display)
           .unwrap();
        make_circle(128, 128, circle_colors[3])
           .draw(&mut display)
            .unwrap();
        let last_color = circle_colors[circle_colors.len() - 1];
        for i in (1..circle_colors.len()).rev() {
            circle_colors[i] = circle_colors[i - 1];
        }
        circle_colors[0] = last_color;
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}


#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
