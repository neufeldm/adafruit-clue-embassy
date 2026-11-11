
use embassy_nrf::{Peri, Peripherals, spim, twim};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull, AnyPin};
use embassy_nrf::peripherals::{
    TWISPI0, TWISPI1,
    SPI2, SPI3,
    PDM,
    PWM0, PWM1, PWM2, PWM3,
    UARTE0, UARTE1,
    USBD,
    TIMER0, TIMER1, TIMER2, TIMER3, TIMER4,
    GPIOTE_CH0, GPIOTE_CH1, GPIOTE_CH3, GPIOTE_CH4, GPIOTE_CH5, GPIOTE_CH6, GPIOTE_CH7,
    PPI_CH0, PPI_CH1, PPI_CH2, PPI_CH3, PPI_CH4, PPI_CH5, PPI_CH6, PPI_CH7, PPI_CH8, PPI_CH9, PPI_CH10, PPI_CH11, PPI_CH12, PPI_CH13, PPI_CH14, PPI_CH15,
    PPI_CH16, PPI_CH17, PPI_CH18, PPI_CH19, PPI_CH20, PPI_CH21, PPI_CH22, PPI_CH23, PPI_CH24, PPI_CH25, PPI_CH26, PPI_CH27, PPI_CH28, PPI_CH29, PPI_CH30, PPI_CH31,
    PPI_GROUP0, PPI_GROUP1, PPI_GROUP2, PPI_GROUP3, PPI_GROUP4, PPI_GROUP5,
    NVMC
    
};
//use embassy_nrf::peripherals::{
//    P0_00, P0_01, P0_02, P0_03, P0_04, P0_05, P0_06, P0_08, P0_09, P0_10, P0_12, P0_13, P0_16, P0_17, P0_20, P0_26,
//    P1_00, P1_02, P1_08, PPI_CH0, PPI_CH1, PWM0, PWM1, PWM2, PWM3, RNG, SAADC, TIMER0, TWISPI0, TWISPI1, SPI2, SPI3, UARTE0,
//    UARTE1,PDM,
//};
use embassy_time::Delay;
use nrf_softdevice;
use crate::pins::{input_pin, output_pin};
//use embedded_hal_bus::spi::ExclusiveDevice;
use mipidsi;

// Sensor I2C IDs
pub const I2C_GYROACCEL: u8 = 0x6A;
pub const I2C_MAGNETOMETER: u8 = 0x1c;
pub const I2C_GESTURE: u8 = 0x39;
pub const I2C_HUMIDITY: u8 = 0x44;
pub const I2C_TEMPPRESSURE: u8 = 0x77;

pub const TFT_XSIZE: u16 = 240;
pub const TFT_YSIZE: u16 = 240;

pub type Button = Input<'static>;
pub type LED = Output<'static>;
pub type Periph<P> = Peri<'static, P>;
pub type OptPeriph<P> = Option<Periph<P>>;


pub struct TFTPeripherals {
    pub twispi: Periph<TWISPI0>,
    pub sck: AnyPin,
    pub mosi: AnyPin,
    pub cs: AnyPin,
    pub dc: AnyPin,
    pub reset: AnyPin,
    pub backlight: AnyPin,
}

// XXX going to split this into the arduino pin IDs
// XXX and devices for our structs, then can provide
// XXX convenience accessors for the circutpython
// XXX names, and methods that scoop the correct
// XXX pins/devices together to build wrappers

pub struct Pins {
    // A2 HS AIN2
    pub p0: Option<AnyPin>,
    // A3 HS AIN3
    pub p1: Option<AnyPin>,
    // A4 LS AIN1
    pub p2: Option<AnyPin>,
    // A5 LS AIN4
    pub p3: Option<AnyPin>,
    // A6 LS AIN0
    pub p4: Option<AnyPin>,
    // BUTTON_A LS (D5) p5
    pub button_a: Option<AnyPin>,
    // D6 HS
    pub p6: Option<AnyPin>,
    // D7 HS
    pub p7: Option<AnyPin>,
    // D8 LS
    pub p8: Option<AnyPin>,
    // D9 HS
    pub p9: Option<AnyPin>,
    // A7 LS AIN6
    pub p10: Option<AnyPin>,
    // BUTTON_B LS (D11) p11
    pub button_b: Option<AnyPin>,
    // A0 LS AIN7
    pub p12: Option<AnyPin>,
    // SCK HS (D13?)
    pub p13: Option<AnyPin>,
    // MISO HS (D14?)
    pub p14: Option<AnyPin>,
    // MOSI HS (D15?)
    pub p15: Option<AnyPin>,
    // A1 LS AIN5
    pub p16: Option<AnyPin>,
    // L LS (red LED) p17
    pub red_led: Option<AnyPin>,
    // NEOPIXEL HS (D18) p18
    pub neopixel: Option<AnyPin>,
    // SCL HS (sensors I2C) p19
    pub sensors_i2c_scl: Option<AnyPin>,
    // SDA HS (sensors I2C)
    pub sensors_i2c_sda: Option<AnyPin>,
    // TFT_SCK HS p29
    pub tft_sck: Option<AnyPin>,
    // TFT_MOSI HS p30
    pub tft_mosi: Option<AnyPin>,
    // TFT_CS HS p31
    pub tft_cs: Option<AnyPin>,
    // TFT_DC HS p32
    pub tft_dc: Option<AnyPin>,
    // TFT_RESET LS p33
    pub tft_reset: Option<AnyPin>,
    // TFT_BACKLIGHT LS p34
    pub tft_backlight: Option<AnyPin>,
    // MICROPHONE_DATA HS XL1 p35
    pub microphone_data: Option<AnyPin>,
    // MICROPHONE_CLOCK HS XL2 p36
    pub microphone_clock: Option<AnyPin>,
    // WHITE_LEDS LS NFC2 p43
    pub white_leds: Option<AnyPin>,
    // PROXIMITY_LIGHT_INTERRUPT LS NFC1 p44
    pub proximity_light_interrupt: Option<AnyPin>,
    // ACCELEROMETER_GYRO_INTERRUPT LS p45
    pub accelerometer_gyro_interrupt: Option<AnyPin>,
    // SPEAKER QSPI p46
    pub speaker: Option<AnyPin>,
}

pub struct PWM(
    pub OptPeriph<PWM0>,
    pub OptPeriph<PWM1>,
    pub OptPeriph<PWM2>,
    pub OptPeriph<PWM3>,
);

pub struct Serial {
    pub twispi0: OptPeriph<TWISPI0>,
    pub twispi1: OptPeriph<TWISPI1>,
    pub spi2: OptPeriph<SPI2>,
    pub spi3: OptPeriph<SPI3>,
}

pub struct UARTE(
    pub OptPeriph<UARTE0>,
    pub OptPeriph<UARTE1>,
);

pub struct Timers(
    pub OptPeriph<TIMER0>,
    pub OptPeriph<TIMER1>,
    pub OptPeriph<TIMER2>,
    pub OptPeriph<TIMER3>,
    pub OptPeriph<TIMER4>,
);

pub struct GPIOTE(
    pub OptPeriph<GPIOTE_CH0>,
    pub OptPeriph<GPIOTE_CH1>,
    pub OptPeriph<GPIOTE_CH3>,
    pub OptPeriph<GPIOTE_CH4>,
    pub OptPeriph<GPIOTE_CH5>,
    pub OptPeriph<GPIOTE_CH6>,
    pub OptPeriph<GPIOTE_CH7>,
);

pub struct Board {
    pub pins: Pins,
    pub serial: Serial,
    pub uarte: UARTE,
    pub pwm: PWM,
    pub timers: Timers,
    pub pdm: OptPeriph<PDM>,
    pub usbd: OptPeriph<USBD>,
    pub gpiote: GPIOTE,
    pub ppi: PPI,
    pub ppi_group: PPIGroup,
    pub nvmc: OptPeriph<NVMC>,
}

macro_rules! opt_pin {
    ($pin:ident, $periph:ident) => {
        Some((*crate::$pin!($periph)).into())
    }
}
macro_rules! opt_ppi {
    ($ppi:literal, $periph:ident) => {
        paste::paste! {
            Some($periph.[<PPI_CH $ppi>])
        }
    };
}

impl Board {
    pub fn new(p: Peripherals) -> Self {
        Board {
            pins: Pins {
                p0: opt_pin!(p0, p),
                p1: opt_pin!(p1, p),
                p2: opt_pin!(p2, p),
                p3: opt_pin!(p3, p),
                p4: opt_pin!(p4, p),
                button_a: opt_pin!(button_a, p),
                p6: opt_pin!(p6, p),
                p7: opt_pin!(p7, p),
                p8: opt_pin!(p8, p),
                p9: opt_pin!(p9, p),
                p10: opt_pin!(p10, p),
                button_b: opt_pin!(button_b, p),
                p12: opt_pin!(p12, p),
                p13: opt_pin!(p13, p),
                p14: opt_pin!(p14, p),
                p15: opt_pin!(p15, p),
                p16: opt_pin!(p16, p),
                red_led: opt_pin!(red_led, p),
                neopixel: opt_pin!(neopixel, p),
                sensors_i2c_scl: opt_pin!(sensors_i2c_scl, p),
                sensors_i2c_sda: opt_pin!(sensors_i2c_sda, p),
                tft_sck: opt_pin!(tft_sck, p),
                tft_mosi: opt_pin!(tft_mosi, p),
                tft_cs: opt_pin!(tft_cs, p),
                tft_dc: opt_pin!(tft_dc, p),
                tft_reset: opt_pin!(tft_reset, p),
                tft_backlight: opt_pin!(tft_backlight, p),
                microphone_data: opt_pin!(microphone_data, p),
                microphone_clock: opt_pin!(microphone_clock, p),
                white_leds: opt_pin!(white_leds, p),
                proximity_light_interrupt: opt_pin!(proximity_light_interrupt, p),
                accelerometer_gyro_interrupt: opt_pin!(accelerometer_gyro_interrupt, p),
                speaker: opt_pin!(speaker, p),
            },
            serial: Serial {
                twispi0: Some(p.TWISPI0),
                twispi1: Some(p.TWISPI1),
                spi2: Some(p.SPI2),
                spi3: Some(p.SPI3),
            },
            uarte: UARTE(Some(p.UARTE0), Some(p.UARTE1)),
            pwm: PWM(Some(p.PWM0), Some(p.PWM1), Some(p.PWM2), Some(p.PWM3)),
            timers: Timers(Some(p.TIMER0), Some(p.TIMER1), Some(p.TIMER2), Some(p.TIMER3), Some(p.TIMER4)),
            pdm: Some(p.PDM),
            usbd: Some(p.USBD),
            gpiote: GPIOTE(Some(p.GPIOTE_CH0), Some(p.GPIOTE_CH1), Some(p.GPIOTE_CH3), Some(p.GPIOTE_CH4), Some(p.GPIOTE_CH5), Some(p.GPIOTE_CH6), Some(p.GPIOTE_CH7)),
            ppi: PPI(opt_ppi!(0,p), opt_ppi!(1,p), opt_ppi!(2,p), opt_ppi!(3,p), opt_ppi!(4,p), opt_ppi!(5,p), opt_ppi!(6,p), opt_ppi!(7,p), opt_ppi!(8,p), opt_ppi!(9,p), opt_ppi!(10,p), opt_ppi!(11,p), opt_ppi!(12,p), opt_ppi!(13,p), opt_ppi!(14,p), opt_ppi!(15,p), opt_ppi!(16,p), opt_ppi!(17,p), opt_ppi!(18,p), opt_ppi!(19,p), opt_ppi!(20,p), opt_ppi!(21,p), opt_ppi!(22,p), opt_ppi!(23,p), opt_ppi!(24,p), opt_ppi!(25,p), opt_ppi!(26,p), opt_ppi!(27,p), opt_ppi!(28,p), opt_ppi!(29,p), opt_ppi!(30,p), opt_ppi!(31,p)),
            ppi_group: PPIGroup(Some(p.PPI_GROUP0), Some(p.PPI_GROUP1), Some(p.PPI_GROUP2), Some(p.PPI_GROUP3), Some(p.PPI_GROUP4), Some(p.PPI_GROUP5)),
            nvmc: Some(p.NVMC),
        }
    }
    pub fn new_from_config(nrf_config: embassy_nrf::config::Config) -> Self {
        let p = embassy_nrf::init(nrf_config);
        Self::new(p)
    }
    pub fn new_default_config(use_softdevice: bool) -> Self {
        Self::new_from_config(Self::embassy_nrf_default_config(use_softdevice))
    }
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
    pub fn get_tft_peripherals(&mut self) -> Option<TFTPeripherals> {
        if self.serial.twispi0.is_none() ||
           self.pins.tft_sck.is_none() ||
           self.pins.tft_mosi.is_none() ||
           self.pins.tft_cs.is_none() ||
           self.pins.tft_dc.is_none() ||
           self.pins.tft_reset.is_none() ||
           self.pins.tft_backlight.is_none() {
            return None;
        }
        Some(TFTPeripherals {
            twispi: self.serial.twispi0.take()?,
            sck: self.pins.tft_sck.take()?,
            mosi: self.pins.tft_mosi.take()?,
            cs: self.pins.tft_cs.take()?,
            dc: self.pins.tft_dc.take()?,
            reset: self.pins.tft_reset.take()?,
            backlight: self.pins.tft_backlight.take()?,
        })
    }

}

pub struct PPI (
    pub OptPeriph<PPI_CH0>, pub OptPeriph<PPI_CH1>, pub OptPeriph<PPI_CH2>, pub OptPeriph<PPI_CH3>, pub OptPeriph<PPI_CH4>, pub OptPeriph<PPI_CH5>, pub OptPeriph<PPI_CH6>, pub OptPeriph<PPI_CH7>, pub OptPeriph<PPI_CH8>, pub OptPeriph<PPI_CH9>, pub OptPeriph<PPI_CH10>, pub OptPeriph<PPI_CH11>, pub OptPeriph<PPI_CH12>, pub OptPeriph<PPI_CH13>, pub OptPeriph<PPI_CH14>, pub OptPeriph<PPI_CH15>,
    pub OptPeriph<PPI_CH16>, pub OptPeriph<PPI_CH17>, pub OptPeriph<PPI_CH18>, pub OptPeriph<PPI_CH19>, pub OptPeriph<PPI_CH20>, pub OptPeriph<PPI_CH21>, pub OptPeriph<PPI_CH22>, pub OptPeriph<PPI_CH23>, pub OptPeriph<PPI_CH24>, pub OptPeriph<PPI_CH25>, pub OptPeriph<PPI_CH26>, pub OptPeriph<PPI_CH27>, pub OptPeriph<PPI_CH28>, pub OptPeriph<PPI_CH29>, pub OptPeriph<PPI_CH30>, pub OptPeriph<PPI_CH31>,
);

pub struct PPIGroup (
    pub OptPeriph<PPI_GROUP0>, pub OptPeriph<PPI_GROUP1>, pub OptPeriph<PPI_GROUP2>, pub OptPeriph<PPI_GROUP3>, pub OptPeriph<PPI_GROUP4>, pub OptPeriph<PPI_GROUP5>,
);