//! Embassy Async/NRF Softdevice pin macros for Adafruit CLUE:
//! https://www.adafruit.com/product/4500

/// Get pin for red LED on back of Clue.
#[macro_export]
macro_rules! red_led {
    ($p:ident) => {
        $p.P1_01
    };
}

/// Get pin for white LEDs on front of Clue.
#[macro_export]
macro_rules! white_led {
    ($p:ident) => {
        $p.P0_10
    };
}

/// Get pin for button "A" (left) on front of Clue.
#[macro_export]
macro_rules! button_a {
    ($p:ident) => {
        $p.P1_02
    };
}

/// Get pin for button "B" (right) on front of Clue.
#[macro_export]
macro_rules! button_b {
    ($p:ident) => {
        $p.P1_10
    };
}

/// SDA pin for I2C bus connected to sensors.
#[macro_export]
macro_rules! sensors_i2c_sda {
    ($p:ident) => {
        $p.P0_24
    };
}

/// SCL pin for I2C bus connected to sensors.
#[macro_export]
macro_rules! sensors_i2c_scl {
    ($p:ident) => {
        $p.P0_25
    };
}

/// Pin connected to neopixel on Clue.
#[macro_export]
macro_rules! neopixel {
    ($p:ident) => {
        $p.P0_16
    };
}

/// Pin connected to the PDM microphone data line.
#[macro_export]
macro_rules! pdm_data {
    ($p:ident) => {
        $p.P0_00
    };
}

/// Pin connected to the PDM microphone clock.
#[macro_export]
macro_rules! pdm_clock {
    ($p:ident) => {
        $p.P0_01
    };
}

/// Pin connected to the speaker on the Clue.
#[macro_export]
macro_rules! speaker {
    ($p:ident) => {
        $p.P1_00
    };
}

/// Pin connected to the accelerometer/gyro.
#[macro_export]
macro_rules! accel_gyro_irq {
    ($p:ident) => {
        $p.P1_06
    };
}

/// Pin connected to the proximity/light sensor.
#[macro_export]
macro_rules! prox_light_irq {
    ($p:ident) => {
        $p.P0_09
    };
}

/// Pin connected to the LCD SPI SCK line.
#[macro_export]
macro_rules! lcd_sck {
    ($p:ident) => {
        $p.P0_14
    };
}

/// Pin connected to the LCD SPI MOSI line.
#[macro_export]
macro_rules! lcd_mosi {
    ($p:ident) => {
        $p.P0_15
    };
}

/// Pin connected to the LCD SPI CS line.
#[macro_export]
macro_rules! lcd_cs {
    ($p:ident) => {
        $p.P0_12
    };
}

/// Pin connected to the LCD SPI DC line.
#[macro_export]
macro_rules! lcd_dc {
    ($p:ident) => {
        $p.P0_13
    };
}

/// Pin connected to the LCD reset line.
#[macro_export]
macro_rules! lcd_reset {
    ($p:ident) => {
        $p.P1_03
    };
}

/// Pin connected to the LCD backlight (high to turn on).
#[macro_export]
macro_rules! lcd_backlight {
    ($p:ident) => {
        $p.P1_05
    };
}

/// CircuitPython pin a0
#[macro_export]
macro_rules! a0 {
    ($p:ident) => {
        $p.P0_31
    };
}

/// CircuitPython pin a1
#[macro_export]
macro_rules! a1 {
    ($p:ident) => {
        $p.P0_29
    };
}

/// CircuitPython pin a2
#[macro_export]
macro_rules! a2 {
    ($p:ident) => {
        $p.P0_04
    };
}

/// CircuitPython pin a3
#[macro_export]
macro_rules! a3 {
    ($p:ident) => {
        $p.P0_05
    };
}

/// CircuitPython pin a4
#[macro_export]
macro_rules! a4 {
    ($p:ident) => {
        $p.P0_03
    };
}

/// CircuitPython pin a5
#[macro_export]
macro_rules! a5 {
    ($p:ident) => {
        $p.P0_28
    };
}

/// CircuitPython pin a6
#[macro_export]
macro_rules! a6 {
    ($p:ident) => {
        $p.P0_02
    };
}

/// CircuitPython pin a7
#[macro_export]
macro_rules! a7 {
    ($p:ident) => {
        $p.P0_30
    };
}

/// CircuitPython pin d6
#[macro_export]
macro_rules! d6 {
    ($p:ident) => {
        $p.P1_09
    };
}

/// CircuitPython pin d7
#[macro_export]
macro_rules! d7 {
    ($p:ident) => {
        $p.P0_07
    };
}

/// CircuitPython pin d8
#[macro_export]
macro_rules! d8 {
    ($p:ident) => {
        $p.P1_07
    };
}

/// CircuitPython pin d9
#[macro_export]
macro_rules! d9 {
    ($p:ident) => {
        $p.P0_27
    };
}

/// CircuitPython pin d13
#[macro_export]
macro_rules! d13 {
    ($p:ident) => {
        $p.P0_08
    };
}

/// CircuitPython pin d14
#[macro_export]
macro_rules! d14 {
    ($p:ident) => {
        $p.P0_06
    };
}

/// CircuitPython pin d15
#[macro_export]
macro_rules! d15 {
    ($p:ident) => {
        $p.P0_26
    };
}
