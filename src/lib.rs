//! Embassy Async/NRF Softdevice Board support for Adafruit CLUE:
//! https://www.adafruit.com/product/4500

#![no_std]

use embassy_nrf;
use embassy_nrf::{spim, twim};
use embassy_nrf::gpio::{Level, Input, Pull, Output, OutputDrive, Pin};
use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
use nrf_softdevice::ble::gatt_server::RegisterError;
use nrf_softdevice::ble::{gatt_server, Connection, Uuid};
use nrf_softdevice::Softdevice;
use nrf_softdevice;

pub fn output_pin<'a, P: Pin>(pin: P, high: bool) -> Output<'a, P> {
    Output::new(pin, if high { Level::High } else { Level::Low }, OutputDrive::Standard)
}

pub fn input_pin<'a, P: Pin>(pin: P, up: bool) -> Input<'a, P> {
    Input::new(pin, if up { Pull::Up } else { Pull::Down })
}

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

pub const TFT_XSIZE: u16 = 240;
pub const TFT_YSIZE: u16 = 240;

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

pub fn tft_spi_config() -> spim::Config {
    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M32; // M32?
    spi_config.orc = 122;
    spi_config.mode = spim::MODE_3;
    spi_config
}

pub fn sensors_twim_config() -> twim::Config {
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    twim_config
}

pub const ADAFRUIT_COMPANY_UUID: u16 = 0x0822;
// adafruit base UUID ADAFxxxx-C332-42A8-93BD-25E905756CB8

pub const NUS_SERVICE_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
];
pub const NUS_SERVICE_RX_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,
];
pub const NUS_SERVICE_TX_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,
];
// 1 byte for the opcode, 2 bytes for the handle
pub const NUS_MTU: u16 = nrf_softdevice::raw::BLE_GATT_ATT_MTU_DEFAULT as u16 - 3;
pub trait NordicUARTRX {
    fn rx(&self, data: &[u8]) -> ();
}

pub struct NordicUARTService<RX: NordicUARTRX> {
    rx_value_handle: u16,
    _rx_cccd_handle: u16,
    tx_value_handle: u16,
    _tx_cccd_handle: u16,
    rx_handler: RX,
}

impl<RX: NordicUARTRX> NordicUARTService<RX>
{
    pub fn new(sd: &mut Softdevice, rx_handler: RX) -> Result<Self, RegisterError> {
        let service_uuid = Uuid::new_128(&NUS_SERVICE_UUID);

        let mut service_builder = ServiceBuilder::new(sd, service_uuid)?;

        let rx_uuid = Uuid::new_128(&NUS_SERVICE_RX_UUID);
        let rx_attr = Attribute::new(&[0u8; NUS_MTU as usize]).variable_len(NUS_MTU);
        let rx_metadata = Metadata::new(Properties::new().write_without_response().write());
        let rx_characteristic_builder =
            service_builder.add_characteristic(rx_uuid, rx_attr, rx_metadata)?;
        let rx_characteristic_handles = rx_characteristic_builder.build();

        let tx_uuid = Uuid::new_128(&NUS_SERVICE_TX_UUID);
        let tx_attr = Attribute::new(&[0u8; NUS_MTU as usize]).variable_len(NUS_MTU);
        let tx_metadata = Metadata::new(Properties::new().notify());
        let tx_characteristic_builder =
            service_builder.add_characteristic(tx_uuid, tx_attr, tx_metadata)?;
        let tx_characteristic_handles = tx_characteristic_builder.build();

        let _service_handle = service_builder.build();

        Ok(NordicUARTService {
            rx_value_handle: rx_characteristic_handles.value_handle,
            _rx_cccd_handle: rx_characteristic_handles.cccd_handle,
            tx_value_handle: tx_characteristic_handles.value_handle,
            _tx_cccd_handle: tx_characteristic_handles.cccd_handle,
            rx_handler: rx_handler,
        })
    }

    pub fn tx_notify(
        &self,
        conn: &Connection,
        buf: &[u8],
    ) -> Result<(), gatt_server::NotifyValueError> {
        gatt_server::notify_value(conn, self.tx_value_handle, buf)
    }

    pub fn on_write(&self, _conn: &Connection, handle: u16, data: &[u8]) {
        if handle == self.rx_value_handle && !data.is_empty() {
            self.rx_handler.rx(data);
        }
    }
}
