//! Embassy Async/NRF Softdevice Board support for Adafruit CLUE:
//! https://www.adafruit.com/product/4500

#![no_std]

use embassy_nrf;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::{spim, twim};
use nrf_softdevice;
//use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
//use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
//use nrf_softdevice::ble::gatt_server::RegisterError;
//use nrf_softdevice::ble::{gatt_server, Connection, Uuid};
//use nrf_softdevice::Softdevice;

/// Take a raw board pin and turn it into an output.
pub fn output_pin<'a, P: Pin>(pin: P, high: bool) -> Output<'a, P> {
    Output::new(
        pin,
        if high { Level::High } else { Level::Low },
        OutputDrive::Standard,
    )
}

/// Take a raw board pin and turn it into an input.
pub fn input_pin<'a, P: Pin>(pin: P, up: bool) -> Input<'a, P> {
    Input::new(pin, if up { Pull::Up } else { Pull::Down })
}

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

pub const LCD_XSIZE: u16 = 240;
pub const LCD_YSIZE: u16 = 240;

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

/// Get a reasonable default Embassy NRF config for the Clue.
/// When using the softdevice be sure to set "softdevice" to true.
pub fn nrf_default_config(softdevice: bool) -> embassy_nrf::config::Config {
    let mut nrf_config = embassy_nrf::config::Config::default();
    if softdevice {
        nrf_config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
        nrf_config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    }
    nrf_config
}

/// Get a reasonable default Embassy NRF softdevice config for the Clue.
pub fn nrf_softdevice_default_config() -> nrf_softdevice::Config {
    nrf_softdevice::Config {
        clock: Some(nrf_softdevice::raw::nrf_clock_lf_cfg_t {
            // XXX does the clue have an xtal for
            // XXX its low frequency clock?
            source: nrf_softdevice::raw::NRF_CLOCK_LF_SRC_RC as u8,
            // 0 for xtal
            rc_ctiv: 16,
            // 0 for xtal
            rc_temp_ctiv: 2,
            // XXX if clue has xtal, find accuracy
            accuracy: nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(nrf_softdevice::raw::ble_gap_conn_cfg_t {
            // number of concurrent connections allowed
            conn_count: nrf_softdevice::raw::BLE_GAP_CONN_COUNT_DEFAULT as u8,
            // Length of a connection event, unit 1.25ms, min. of 6
            //raw::BLE_GAP_EVENT_LENGTH_DEFAULT
            //raw::BLE_GAP_EVENT_LENGTH_MIN
            event_length: nrf_softdevice::raw::BLE_GAP_EVENT_LENGTH_DEFAULT as u16,
        }),
        // Max. attribute size
        conn_gatt: Some(nrf_softdevice::raw::ble_gatt_conn_cfg_t {
            att_mtu: nrf_softdevice::raw::BLE_GATT_ATT_MTU_DEFAULT as u16,
        }),
        // attribute table size
        // needs to scale depending on how much "stuff" you've got,
        // number of characteristics
        // raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
        // raw::BLE_GATTS_ATTR_TAB_SIZE_MIN;
        gatts_attr_tab_size: Some(nrf_softdevice::raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: nrf_softdevice::raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(nrf_softdevice::raw::ble_gap_cfg_role_count_t {
            // max number of advertising sets
            //raw::BLE_GAP_ADV_SET_COUNT_MAX;
            adv_set_count: nrf_softdevice::raw::BLE_GAP_ADV_SET_COUNT_DEFAULT as u8,
            periph_role_count: nrf_softdevice::raw::BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT as u8,
            central_role_count: nrf_softdevice::raw::BLE_GAP_ROLE_COUNT_CENTRAL_DEFAULT as u8,
            central_sec_count: nrf_softdevice::raw::BLE_GAP_ROLE_COUNT_CENTRAL_SEC_DEFAULT as u8,
            _bitfield_1: nrf_softdevice::raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        ..Default::default()
    }
}

/// Get a reasonable config for the SPI attached to the LCD.
pub fn lcd_spi_config() -> spim::Config {
    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M32; // M32?
    spi_config.orc = 122;
    spi_config.mode = spim::MODE_3;
    spi_config
}

/// Get a reasonable config for the TWI attached to the sensors.
pub fn sensors_twim_config() -> twim::Config {
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    twim_config
}

// Sensor I2C IDs
pub const I2C_GYROACCEL: u8 = 0x6A;
pub const I2C_MAGNETOMETER: u8 = 0x1c;
pub const I2C_GESTURE: u8 = 0x39;
pub const I2C_HUMIDITY: u8 = 0x44;
pub const I2C_TEMPPRESSURE: u8 = 0x77;

pub const ADAFRUIT_COMPANY_UUID: u16 = 0x0822;
// adafruit base UUID ADAFxxxx-C332-42A8-93BD-25E905756CB8

pub const NORDIC_UART_SERVICE_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
];
pub const NORDIC_UART_SERVICE_RX_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,
];
pub const NORDIC_UART_SERVICE_TX_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,
];
// 1 byte for the opcode, 2 bytes for the handle
pub const NORDIC_UART_MTU: u16 = nrf_softdevice::raw::BLE_GATT_ATT_MTU_DEFAULT as u16 - 3;

//pub const ADAFRUIT_TEMPERATURE_SERVICE: &str = "ADAF0100-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_ACCELEROMETER_SERVICE: &str = "ADAF0200-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_LIGHT_SENSOR_SERVICE: &str = "ADAF0300-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_GYROSCOPE_SERVICE: &str = "ADAF0400-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_MAGNETOMETER_SERVICE: &str = "ADAF0500-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_BUTTON_SERVICE: &str = "ADAF0600-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_HUMIDITY_SERVICE: &str = "ADAF0700-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_BAROMETRIC_PRESSURE_SERVICE: &str = "ADAF0800-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_PIXEL_SERVICE: &str = "ADAF0900-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_COLOR_SENSOR_SERVICE: &str = "ADAF0A00-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_MICROPHONE_SERVICE: &str = "ADAF0B00-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_TONE_SERVICE: &str = "ADAF0C00-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_QUATERNION_SERVICE: &str = "ADAF0D00-C332-42A8-93BD-25E905756CB8";
//pub const ADAFRUIT_PROXIMITY_SERVICE: &str = "ADAF0E00-C332-42A8-93BD-25E905756CB8";
