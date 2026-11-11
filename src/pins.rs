//! Embassy Async/NRF Softdevice pin macros for Adafruit CLUE:
//! https://www.adafruit.com/product/4500

use embassy_nrf::Peri;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull};

    /// Take a raw board pin and turn it into an output.
pub fn output_pin<'a>(pin: Peri<'a, impl Pin>, high: bool) -> Output<'a> {
    Output::new(
        pin,
        if high { Level::High } else { Level::Low },
        OutputDrive::Standard,
    )
}

/// Take a raw board pin and turn it into an input.
pub fn input_pin<'a>(pin: Peri<'a, impl Pin>, up: bool) -> Input<'a> {
    Input::new(pin, if up { Pull::Up } else { Pull::Down })
}

// Core nRF52840 pin IDs mapped to Arduino pin numbers.
// Will also have convenience mappings from circuitpython
// and device-specific names to these.

// A2 HS AIN2
#[macro_export]
macro_rules! p0 {
    ($p:ident) => {
        $p.P0_04
    };
}

// A3 HS AIN3
#[macro_export]
macro_rules! p1 {
    ($p:ident) => {
        $p.P0_05
    };
}

// A4 LS AIN1
#[macro_export]
macro_rules! p2 {
    ($p:ident) => {
        $p.P0_03
    };
}

// A5 LS AIN4
#[macro_export]
macro_rules! p3 {
    ($p:ident) => {
        $p.P0_28
    };
}

// A6 LS AIN0
#[macro_export]
macro_rules! p4 {
    ($p:ident) => {
        $p.P0_02
    };
}

// BUTTON_A LS
// this is also arduino D5
#[macro_export]
macro_rules! p5 {
    ($p:ident) => {
        $p.P1_02
    };
}

// D6 HS
#[macro_export]
macro_rules! p6 {
    ($p:ident) => {
        $p.P1_09
    };
}

// D7 HS
#[macro_export]
macro_rules! p7 {
    ($p:ident) => {
        $p.P0_07
    };
}

// D8 LS
#[macro_export]
macro_rules! p8 {
    ($p:ident) => {
        $p.P1_07
    };
}

// D9 HS
#[macro_export]
macro_rules! p9 {
    ($p:ident) => {
        $p.P0_27
    };
}

// A7 LS AIN6
#[macro_export]
macro_rules! p10 {
    ($p:ident) => {
        $p.P0_30
    };
}

// BUTTON_B LS
// this is also arduino D11
#[macro_export]
macro_rules! p11 {
    ($p:ident) => {
        $p.P1_10
    };
}

// A0 LS AIN7
#[macro_export]
macro_rules! p12 {
    ($p:ident) => {
        $p.P0_31
    };
}

// SCK HS (D13?)
#[macro_export]
macro_rules! p13 {
    ($p:ident) => {
        $p.P0_08
    };
}

// MISO HS (D14?)
#[macro_export]
macro_rules! p14 {
    ($p:ident) => {
        $p.P0_06
    };
}

// MOSI HS (D15?)
#[macro_export]
macro_rules! p15 {
    ($p:ident) => {
        $p.P0_26
    };
}

// A1 LS AIN5
#[macro_export]
macro_rules! p16 {
    ($p:ident) => {
        $p.P0_29
    };
}

// L LS
// red LED 
#[macro_export]
macro_rules! p17 {
    ($p:ident) => {
        $p.P1_01
    };
}

// NEOPIXEL HS
// this is also arduino D18
#[macro_export]
macro_rules! p18 {
    ($p:ident) => {
        $p.P0_16
    };
}

// SCL HS
// sensors I2C
#[macro_export]
macro_rules! p19 {
    ($p:ident) => {
        $p.P0_25
    };
}

// SDA HS
// sensors I2C
#[macro_export]
macro_rules! p20 {
    ($p:ident) => {
        $p.P0_24
    };
}

// TFT_SCK HS
#[macro_export]
macro_rules! p29 {
    ($p:ident) => {
        $p.P0_14
    };
}

// TFT_MOSI HS
#[macro_export]
macro_rules! p30 {
    ($p:ident) => {
        $p.P0_15
    };
}

// TFT_CS HS
#[macro_export]
macro_rules! p31 {
    ($p:ident) => {
        $p.P0_12
    };
}

// TFT_DC HS
#[macro_export]
macro_rules! p32 {
    ($p:ident) => {
        $p.P0_13
    };
}

// TFT_RESET LS
#[macro_export]
macro_rules! p33 {
    ($p:ident) => {
        $p.P1_03
    };
}

// TFT_BACKLIGHT LS
#[macro_export]
macro_rules! p34 {
    ($p:ident) => {
        $p.P1_05
    };
}

// MICROPHONE_DATA HS XL1
#[macro_export]
macro_rules! p35 {
    ($p:ident) => {
        $p.P0_00
    };
}

// MICROPHONE_CLOCK HS XL2
#[macro_export]
macro_rules! p36 {
    ($p:ident) => {
        $p.P0_01
    };
}

// WHITE_LEDS LS NFC2
#[macro_export]
macro_rules! p43 {
    ($p:ident) => {
        $p.P0_10
    };
}

// PROXIMITY_LIGHT_INTERRUPT LS NFC1
#[macro_export]
macro_rules! p44 {
    ($p:ident) => {
        $p.P0_09
    };
}

// ACCELEROMETER_GYRO_INTERRUPT LS
#[macro_export]
macro_rules! p45 {
    ($p:ident) => {
        $p.P1_06
    };
}

// SPEAKER QSPI
#[macro_export]
macro_rules! p46 {
    ($p:ident) => {
        $p.P1_00
    };
}
