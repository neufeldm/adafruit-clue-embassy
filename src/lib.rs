//! Embassy Async/NRF Softdevice Board support for Adafruit CLUE:
//! https://www.adafruit.com/product/4500

#![no_std]

use embassy_nrf;
use nrf_softdevice;

#[macro_export]
macro_rules! red_led {
    ($p:ident) => {
        $p.P1_01
    }
}

#[macro_export]
macro_rules! white_led {
    ($p:ident) => {
        $p.P0_10
    }
}

#[macro_export]
macro_rules! button_a {
    ($p:ident) => {
        $p.P1_02
    }
}

#[macro_export]
macro_rules! button_b {
    ($p:ident) => {
        $p.P1_10
    }
}

#[macro_export]
macro_rules! sensors_i2c_sda {
    ($p:ident) => {
        $p.P0_24
    }
}

#[macro_export]
macro_rules! sensors_i2c_scl {
    ($p:ident) => {
        $p.P0_25
    }
}

#[macro_export]
macro_rules! neopixel {
    ($p:ident) => {
        $p.P0_16
    }
}

#[macro_export]
macro_rules! pdm_data {
    ($p:ident) => {
        $p.P0_00
    }
}

#[macro_export]
macro_rules! pdm_clock {
    ($p:ident) => {
        $p.P0_01
    }
}

#[macro_export]
macro_rules! speaker {
    ($p:ident) => {
        $p.P1_00
    }
}


#[macro_export]
macro_rules! accel_gyro_irq {
    ($p:ident) => {
        $p.P1_06
    }
}


#[macro_export]
macro_rules! prox_light_irq {
    ($p:ident) => {
        $p.P0_09
    }
}

#[macro_export]
macro_rules! tft_sck {
    ($p:ident) => {
        $p.P0_14
    }
}

#[macro_export]
macro_rules! tft_mosi {
    ($p:ident) => {
        $p.P0_15
    }
}

#[macro_export]
macro_rules! tft_cs {
    ($p:ident) => {
        $p.P0_12
    }
}

#[macro_export]
macro_rules! tft_dc {
    ($p:ident) => {
        $p.P0_13
    }
}

#[macro_export]
macro_rules! tft_reset {
    ($p:ident) => {
        $p.P1_03
    }
}

#[macro_export]
macro_rules! tft_backlight {
    ($p:ident) => {
        $p.P1_05
    }
}

#[macro_export]
macro_rules! a0 {
    ($p:ident) => {
        $p.P0_31
    }
}

#[macro_export]
macro_rules! a1 {
    ($p:ident) => {
        $p.P0_29
    }
}

#[macro_export]
macro_rules! a2 {
    ($p:ident) => {
        $p.P0_04
    }
}

#[macro_export]
macro_rules! a3 {
    ($p:ident) => {
        $p.P0_05
    }
}

#[macro_export]
macro_rules! a4 {
    ($p:ident) => {
        $p.P0_03
    }
}

#[macro_export]
macro_rules! a5 {
    ($p:ident) => {
        $p.P0_28
    }
}

#[macro_export]
macro_rules! a6 {
    ($p:ident) => {
        $p.P0_02
    }
}

#[macro_export]
macro_rules! a7 {
    ($p:ident) => {
        $p.P0_30
    }
}

#[macro_export]
macro_rules! d6 {
    ($p:ident) => {
        $p.P1_09
    }
}

#[macro_export]
macro_rules! d7 {
    ($p:ident) => {
        $p.P0_07
    }
}

#[macro_export]
macro_rules! d8 {
    ($p:ident) => {
        $p.P1_07
    }
}

#[macro_export]
macro_rules! d9 {
    ($p:ident) => {
        $p.P0_27
    }
}

#[macro_export]
macro_rules! d13 {
    ($p:ident) => {
        $p.P0_08
    }
}

#[macro_export]
macro_rules! d14 {
    ($p:ident) => {
        $p.P0_06
    }
}

#[macro_export]
macro_rules! d15 {
    ($p:ident) => {
        $p.P0_26
    }
}

pub fn nrf_default_config(softdevice: bool) -> embassy_nrf::config::Config {
    let mut nrf_config = embassy_nrf::config::Config::default();
    if softdevice {
        nrf_config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
        nrf_config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    }
    nrf_config
}

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
        conn_gatt: Some(nrf_softdevice::raw::ble_gatt_conn_cfg_t { att_mtu: nrf_softdevice::raw::BLE_GATT_ATT_MTU_DEFAULT as u16 }),
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
