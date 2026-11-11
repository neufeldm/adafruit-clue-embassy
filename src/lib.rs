//! Embassy Async/NRF Softdevice Board support for Adafruit CLUE:
//! https://www.adafruit.com/product/4500

#![no_std]

pub mod pins;
pub mod board;
use embassy_nrf as _;
use embassy_nrf::{bind_interrupts, peripherals, spim, twim};
use embassy_time::{Delay, Duration, Timer};
use nrf_softdevice;
// This is the generic display interface that we can
// hand back if a user wants to get the LCD
use mipidsi::{Display, Builder, models, interface::SpiInterface, options::Orientation, options::ColorInversion, options::Rotation};

// Serial for the sensors
// Pins for the buttons
// Pins for the LEDs
// USB, bluetooth?



/// Get a reasonable default Embassy NRF config for the Clue.
/// When using the softdevice be sure to set "softdevice" to true.
pub fn embassy_nrf_default_config(softdevice: bool) -> embassy_nrf::config::Config {
    let mut nrf_config = embassy_nrf::config::Config::default();
    if softdevice {
        nrf_config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
        nrf_config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    }
    nrf_config
}

/// Get a reasonable default NRF softdevice config for the Clue.
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

/// Get a reasonable config for the TWI attached to the sensors.
pub fn sensors_twim_config() -> twim::Config {
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    twim_config
}
/// Get a reasonable config for the SPI attached to the TFT.
pub fn tft_spi_config() -> spim::Config {
    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M32; // M32?
    spi_config.orc = 122;
    spi_config.mode = spim::MODE_3;
    spi_config
}
