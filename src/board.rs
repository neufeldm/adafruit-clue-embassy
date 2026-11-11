
use embassy_nrf::{Peri, Peripherals, spim, twim};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::peripherals::{
    P0_00, P0_01, P0_02, P0_03, P0_04, P0_05, P0_06, P0_07, P0_08, P0_09, P0_10, P0_11, P0_12, P0_13, P0_14, P0_15, P0_16, P0_17, P0_19, P0_20, P0_21, P0_22, P0_23, P0_24, P0_25, P0_26, P0_27, P0_28, P0_29, P0_30, P0_31,
    P1_00, P1_01, P1_02, P1_03, P1_04, P1_05, P1_06, P1_07, P1_08, P1_09, P1_10, P1_11, P1_12, P1_13, P1_14, P1_15,
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
use embedded_hal_bus::spi::ExclusiveDevice;
use mipidsi;

// Sensor I2C IDs
pub const I2C_GYROACCEL: u8 = 0x6A;
pub const I2C_MAGNETOMETER: u8 = 0x1c;
pub const I2C_GESTURE: u8 = 0x39;
pub const I2C_HUMIDITY: u8 = 0x44;
pub const I2C_TEMPPRESSURE: u8 = 0x77;

pub const LCD_XSIZE: u16 = 240;
pub const LCD_YSIZE: u16 = 240;

pub type Button = Input<'static>;
pub type LED = Output<'static>;


// XXX going to split this into the arduino pin IDs
// XXX and devices for our structs, then can provide
// XXX convenience accessors for the circutpython
// XXX names, and methods that scoop the correct
// XXX pins/devices together to build wrappers
pub type OptPeriph<P> = Option<Peri<'static, P>>;
pub struct Pins {
    // A2 HS AIN2
    p0: OptPeriph<P0_04>,
    // A3 HS AIN3
    p1: OptPeriph<P0_05>,
    // A4 LS AIN1
    p2: OptPeriph<P0_03>,
    // A5 LS AIN4
    p3: OptPeriph<P0_28>,
    // A6 LS AIN0
    p4: OptPeriph<P0_02>,
    // BUTTON_A LS (D5)
    p5: OptPeriph<P1_02>,
    // D6 HS
    p6: OptPeriph<P1_09>,
    // D7 HS
    p7: OptPeriph<P0_07>,
    // D8 LS
    p8: OptPeriph<P1_07>,
    // D9 HS
    p9: OptPeriph<P0_27>,
    // A7 LS AIN6
    p10: OptPeriph<P0_30>,
    // BUTTON_B LS (D11)
    p11: OptPeriph<P1_10>,
    // A0 LS AIN7
    p12: OptPeriph<P0_31>,
    // SCK HS (D13?)
    p13: OptPeriph<P0_08>,
    // MISO HS (D14?)
    p14: OptPeriph<P0_06>,
    // MOSI HS (D15?)
    p15: OptPeriph<P0_26>,
    // A1 LS AIN5
    p16: OptPeriph<P0_29>,
    // L LS (red LED)
    p17: OptPeriph<P1_01>,
    // NEOPIXEL HS (D18)
    p18: OptPeriph<P0_16>,
    // SCL HS (sensors I2C)
    p19: OptPeriph<P0_25>,
    // SDA HS (sensors I2C)
    p20: OptPeriph<P0_24>,
    // TFT_SCK HS
    p29: OptPeriph<P0_14>,
    // TFT_MOSI HS
    p30: OptPeriph<P0_15>,
    // TFT_CS HS
    p31: OptPeriph<P0_12>,
    // TFT_DC HS
    p32: OptPeriph<P0_13>,
    // TFT_RESET LS
    p33: OptPeriph<P1_03>,
    // TFT_BACKLIGHT LS
    p34: OptPeriph<P1_05>,
    // MICROPHONE_DATA HS XL1
    p35: OptPeriph<P0_00>,
    // MICROPHONE_CLOCK HS XL2
    p36: OptPeriph<P0_01>,
    // WHITE_LEDS LS NFC2
    p43: OptPeriph<P0_10>,
    // PROXIMITY_LIGHT_INTERRUPT LS NFC1
    p44: OptPeriph<P0_09>,
    // ACCELEROMETER_GYRO_INTERRUPT LS
    p45: OptPeriph<P1_06>,
    // SPEAKER QPSI
    p46: OptPeriph<P1_00>,
}

pub struct PWM(
    OptPeriph<PWM0>,
    OptPeriph<PWM1>,
    OptPeriph<PWM2>,
    OptPeriph<PWM3>,
);

pub struct Serial {
    pub twispi0: OptPeriph<TWISPI0>,
    pub twispi1: OptPeriph<TWISPI1>,
    pub spi2: OptPeriph<SPI2>,
    pub spi3: OptPeriph<SPI3>,
}

pub struct UARTE(
    OptPeriph<UARTE0>,
    OptPeriph<UARTE1>,
);

pub struct Timers(
    OptPeriph<TIMER0>,
    OptPeriph<TIMER1>,
    OptPeriph<TIMER2>,
    OptPeriph<TIMER3>,
    OptPeriph<TIMER4>,
);

pub struct GPIOTE(
    OptPeriph<GPIOTE_CH0>,
    OptPeriph<GPIOTE_CH1>,
    OptPeriph<GPIOTE_CH3>,
    OptPeriph<GPIOTE_CH4>,
    OptPeriph<GPIOTE_CH5>,
    OptPeriph<GPIOTE_CH6>,
    OptPeriph<GPIOTE_CH7>,
);

pub struct Board {
    pins: Pins,
    serial: Serial,
    uarte: UARTE,
    pwm: PWM,
    timers: Timers,
    pdm: OptPeriph<PDM>,
    usbd: OptPeriph<USBD>,
    gpiote: GPIOTE,
    ppi: PPI,
    ppi_group: PPIGroup,
    nvmc: OptPeriph<NVMC>,
}

macro_rules! opt_pin {
    ($pin:ident, $periph:ident) => {
        Some(crate::$pin!($periph))
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
                p5: opt_pin!(p5, p),
                p6: opt_pin!(p6, p),
                p7: opt_pin!(p7, p),
                p8: opt_pin!(p8, p),
                p9: opt_pin!(p9, p),
                p10: opt_pin!(p10, p),
                p11: opt_pin!(p11, p),
                p12: opt_pin!(p12, p),
                p13: opt_pin!(p13, p),
                p14: opt_pin!(p14, p),
                p15: opt_pin!(p15, p),
                p16: opt_pin!(p16, p),
                p17: opt_pin!(p17, p),
                p18: opt_pin!(p18, p),
                p19: opt_pin!(p19, p),
                p20: opt_pin!(p20, p),
                p29: opt_pin!(p29, p),
                p30: opt_pin!(p30, p),
                p31: opt_pin!(p31, p),
                p32: opt_pin!(p32, p),
                p33: opt_pin!(p33, p),
                p34: opt_pin!(p34, p),
                p35: opt_pin!(p35, p),
                p36: opt_pin!(p36, p),
                p43: opt_pin!(p43, p),
                p44: opt_pin!(p44, p),
                p45: opt_pin!(p45, p),
                p46: opt_pin!(p46, p),
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

}

pub struct PPI (
    OptPeriph<PPI_CH0>, OptPeriph<PPI_CH1>, OptPeriph<PPI_CH2>, OptPeriph<PPI_CH3>, OptPeriph<PPI_CH4>, OptPeriph<PPI_CH5>, OptPeriph<PPI_CH6>, OptPeriph<PPI_CH7>, OptPeriph<PPI_CH8>, OptPeriph<PPI_CH9>, OptPeriph<PPI_CH10>, OptPeriph<PPI_CH11>, OptPeriph<PPI_CH12>, OptPeriph<PPI_CH13>, OptPeriph<PPI_CH14>, OptPeriph<PPI_CH15>,
    OptPeriph<PPI_CH16>, OptPeriph<PPI_CH17>, OptPeriph<PPI_CH18>, OptPeriph<PPI_CH19>, OptPeriph<PPI_CH20>, OptPeriph<PPI_CH21>, OptPeriph<PPI_CH22>, OptPeriph<PPI_CH23>, OptPeriph<PPI_CH24>, OptPeriph<PPI_CH25>, OptPeriph<PPI_CH26>, OptPeriph<PPI_CH27>, OptPeriph<PPI_CH28>, OptPeriph<PPI_CH29>, OptPeriph<PPI_CH30>, OptPeriph<PPI_CH31>,
);

pub struct PPIGroup (
    OptPeriph<PPI_GROUP0>, OptPeriph<PPI_GROUP1>, OptPeriph<PPI_GROUP2>, OptPeriph<PPI_GROUP3>, OptPeriph<PPI_GROUP4>, OptPeriph<PPI_GROUP5>,
);