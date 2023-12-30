#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![macro_use]

use embassy_nrf as _; // time driver
                      //use embassy_nrf as _; // time driver
                      //use panic_probe as _;

use core::mem;
use embassy_time::{Duration, Timer};

use embassy_executor::Spawner;
use futures::future::{select, Either};
use futures::pin_mut;

use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
use nrf_softdevice::ble::gatt_server::{RegisterError, WriteOp};
use nrf_softdevice::ble::{gatt_server, peripheral, Connection, Uuid};
use nrf_softdevice::{raw, Softdevice};

use adafruit_clue_embassy;
use adafruit_clue_embassy::{lcd_backlight, nrf_default_config, output_pin, red_led, white_led};

// 1 byte for the opcode, 2 bytes for the handle
pub const NUS_MTU: u16 = raw::BLE_GATT_ATT_MTU_DEFAULT as u16 - 3;

pub const NUS_SERVICE_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
];
pub const NUS_SERVICE_RX_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,
];
pub const NUS_SERVICE_TX_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,
];

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
    //sd.run_with_callback(f);
}

async fn tx_ble<'a>(conn: &'a Connection, nus: &'a NordicUARTService) {
    // unnecessary initialization, but at least only happens onece
    let hello: [u8; 6] = [b'H', b'o', b'w', b'd', b'y', b'!'];
    loop {
        let _ = nus.tx_notify(conn, &hello);
        Timer::after(Duration::from_secs(4)).await;
    }
}

pub struct NordicUARTService {
    _rx_value_handle: u16,
    _rx_cccd_handle: u16,
    tx_value_handle: u16,
    _tx_cccd_handle: u16,
}

impl NordicUARTService {
    pub fn new(sd: &mut Softdevice) -> Result<Self, RegisterError> {
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
            _rx_value_handle: rx_characteristic_handles.value_handle,
            _rx_cccd_handle: rx_characteristic_handles.cccd_handle,
            tx_value_handle: tx_characteristic_handles.value_handle,
            _tx_cccd_handle: tx_characteristic_handles.cccd_handle,
        })
    }

    pub fn tx_notify(
        &self,
        conn: &Connection,
        buf: &[u8],
    ) -> Result<(), gatt_server::NotifyValueError> {
        gatt_server::notify_value(conn, self.tx_value_handle, buf)
    }
}

struct Server {
    nus: NordicUARTService,
}

impl Server {
    pub fn new(sd: &mut Softdevice) -> Result<Self, RegisterError> {
        let nus = NordicUARTService::new(sd)?;
        Ok(Self { nus })
    }
}

impl gatt_server::Server for Server {
    type Event = ();

    fn on_write(
        &self,
        _conn: &Connection,
        _handle: u16,
        _op: WriteOp,
        _offset: usize,
        _data: &[u8],
    ) -> Option<Self::Event> {
        None
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let nrf_config = nrf_default_config(true);
    let nrf_periph = embassy_nrf::init(nrf_config);
    let mut white = output_pin(white_led!(nrf_periph), false);
    let mut red = output_pin(red_led!(nrf_periph), false);
    // Turn off the LCD backlight since we aren't using it.
    output_pin(lcd_backlight!(nrf_periph), false);

    let mut sd_config = adafruit_clue_embassy::nrf_softdevice_default_config();
    let sd = Softdevice::enable(&sd_config);

    sd_config.gap_device_name =
        // XXX maybe make a macro for this?
        Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"AdafruitClue" as *const u8 as _,
            current_len: 12,
            max_len: 12,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        });

    let server = Server::new(sd).unwrap();
    spawner.spawn(softdevice_task(sd)).unwrap();

    //raw::BLE_GAP_AD_TYPE
    #[rustfmt::skip]
    let adv_data = &[
        0x02, raw::BLE_GAP_AD_TYPE_FLAGS as u8, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        0x03, raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8, 0x09, 0x18,
        0x0d, raw::BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME as u8, b'A', b'd', b'a', b'f', b'r', b'u', b'i', b't', b'C', b'l', b'u', b'e',
    ];
    #[rustfmt::skip]
    let scan_data = &[
        0x03, raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8, 0x09, 0x18,
    ];

    loop {
        red.set_low();
        white.set_high();

        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data,
            scan_data,
        };
        let conn = peripheral::advertise_connectable(sd, adv, &config)
            .await
            .unwrap();
        red.set_high();
        white.set_low();

        let gatt_fut = gatt_server::run(&conn, &server, |_| {});
        let tx_ble_fut = tx_ble(&conn, &server.nus);
        pin_mut!(tx_ble_fut);
        pin_mut!(gatt_fut);
        let _ = match select(gatt_fut, tx_ble_fut).await {
            Either::Left((_, _)) => {
                //info!("Connection dropped!");
            }
            Either::Right((_, _)) => {
                //info!("ble tx failed!");
            }
        };
    }
}

#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
