#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_nrf as _;
use embassy_adafruit_clue::{nrf_default_config, TFT_XSIZE, TFT_YSIZE, tft_sck, tft_mosi, tft_dc, tft_reset, tft_backlight, white_led };
use embassy_adafruit_clue;
use embassy_nrf::spis::MODE_3;
//use embassy_nrf::peripherals::SPI3;
use embassy_time::{Delay, Duration, Timer};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
//use embassy_nrf::interrupt;
//use embassy_embedded_hal::shared_bus::spi::SpiDevice;
//use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
//use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice as SpiDevice_blocking;
//use embassy_sync::mutex::Mutex;
//use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
//use static_cell::StaticCell;
use embassy_nrf::{bind_interrupts, peripherals, spim};

use display_interface_spi::SPIInterfaceNoCS;
//use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use mipidsi::Builder;

//use cortex_m_rt;
//use embedded_hal::blocking::delay::DelayMs;
//use nrf52840_hal::Delay;
//use nrf52840_hal::Timer;

//use display_interface_spi::SPIInterfaceNoCS;
//use embedded_graphics::pixelcolor::Rgb565;
//use embedded_graphics::prelude::*;
//use embedded_graphics::primitives::*;
//use embedded_graphics::{
//    mono_font::{ascii::FONT_6X10, MonoTextStyle},
//    text::Text,
//};
bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

//struct MyAnyPin(AnyPin);
//impl OutputPin for MyAnyPin {
//    type Error;
//    fn set_low(&mut self) -> Result<(), Self::Error> {
//        self.0.
//    }
//    fn set_high(&mut self) -> Result<(), Self::Error>;
//
//   fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> { ... }
//}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let nrf_config = nrf_default_config(false);
    let nrf_periph = embassy_nrf::init(nrf_config);
    let mut backlight = Output::new(tft_backlight!(nrf_periph), Level::High, OutputDrive::Standard);
    backlight.set_high();
    //static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, spim::Spim<SPI3>>> = StaticCell::new();
    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M8; // M32?
    spi_config.orc = 122;
    spi_config.mode = MODE_3;

    let spi = spim::Spim::new_txonly(nrf_periph.SPI3, Irqs, tft_sck!(nrf_periph), tft_mosi!(nrf_periph), spi_config);
    //let spi_bus = Mutex::<ThreadModeRawMutex, _>::new(spi);
    //let spi_bus = SPI_BUS.init(spi_bus);

    //let cs_pin = Output::new(tft_cs!(nrf_periph), Level::Low, OutputDrive::Standard);
    //let spi_dev = SpiDevice::new(spi_bus, tft_cs!(nrf_periph));
    //let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss1, 60_000_000_u32, Mode::Mode0).unwrap();
    let di = SPIInterfaceNoCS::new(spi,Output::new(tft_dc!(nrf_periph), Level::Low, OutputDrive::Standard));
    let mut delay = Delay {};
    let mut display = Builder::st7789(di)
        // width and height are switched on purpose because of the orientation
        .with_display_size(TFT_XSIZE, TFT_YSIZE)
        .with_orientation(mipidsi::Orientation::Landscape(false))
        .with_invert_colors(mipidsi::ColorInversion::Normal)
        .init(&mut delay, Some(Output::new(tft_reset!(nrf_periph), Level::Low, OutputDrive::Standard)))
        .unwrap();
    //display.set_orientation(Orientation::Landscape).unwrap();
    display.clear(Rgb565::GREEN).unwrap();
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    Text::new("Hello Kitty!", Point::new(50, 50), text_style)
        .draw(&mut display)
        .unwrap();
    //let make_circle = |x, y, color: Rgb565| {
    //    Circle::new(Point::new(x, y), 64).into_styled(PrimitiveStyle::with_fill(color))
    //};
    //let mut circle_colors = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE, Rgb565::WHITE];
    let mut led = Output::new(white_led!(nrf_periph), Level::Low, OutputDrive::Standard);
    loop {
    //    make_circle(128, 64, circle_colors[0])
    //        .draw(&mut display)
    //        .unwrap();
    //    make_circle(64, 64, circle_colors[1])
    //        .draw(&mut display)
    //        .unwrap();
    //    make_circle(64, 128, circle_colors[2])
    //        .draw(&mut display)
    //       .unwrap();
    //    make_circle(128, 128, circle_colors[3])
    //       .draw(&mut display)
    //        .unwrap();
    //    let last_color = circle_colors[circle_colors.len() - 1];
    //    for i in (1..circle_colors.len()).rev() {
    //        circle_colors[i] = circle_colors[i - 1];
    //    }
    //    circle_colors[0] = last_color;
    //    timer.delay_ms(1000 as u32);
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
