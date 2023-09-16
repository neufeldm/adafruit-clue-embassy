#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use adafruit_clue_embassy;
use adafruit_clue_embassy::{
    button_a, button_b, input_pin, nrf_default_config, output_pin, red_led, white_led,
};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_executor::Spawner;
use embassy_nrf as _; // time driver
use embassy_nrf::gpio::{AnyPin, Input, Pin};
use embassy_time::{Duration, Timer};

static BLINK_1_ON_MS: AtomicU32 = AtomicU32::new(100);
static BLINK_2_ON_MS: AtomicU32 = AtomicU32::new(100);

#[embassy_executor::task]
async fn blink(pin_1: AnyPin, pin_2: AnyPin) {
    let mut led_1 = output_pin(pin_1, false);
    let mut led_2 = output_pin(pin_2, false);
    loop {
        led_1.set_high();
        led_2.set_low();
        Timer::after(Duration::from_millis(
            BLINK_1_ON_MS.load(Ordering::Relaxed).into(),
        ))
        .await;
        led_1.set_low();
        led_2.set_high();
        Timer::after(Duration::from_millis(
            BLINK_2_ON_MS.load(Ordering::Relaxed).into(),
        ))
        .await;
    }
}
const ON_INTERVALS_MS: [u32; 5] = [100, 200, 400, 800, 1600];
#[embassy_executor::task(pool_size = 2)]
async fn button_task(mut pin: Input<'static, AnyPin>, on_ms: &'static AtomicU32) {
    let mut cur_int: usize = 0;
    loop {
        on_ms.store(ON_INTERVALS_MS[cur_int], Ordering::Relaxed);
        pin.wait_for_rising_edge().await;
        cur_int += 1;
        if cur_int >= ON_INTERVALS_MS.len() {
            cur_int = 0;
        }
    }
}
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let nrf_config = nrf_default_config(false);
    let nrf_periph = embassy_nrf::init(nrf_config);
    spawner
        .spawn(blink(
            red_led!(nrf_periph).degrade(),
            white_led!(nrf_periph).degrade(),
        ))
        .unwrap();

    let button_a = input_pin(button_a!(nrf_periph).degrade(), true);
    spawner
        .spawn(button_task(button_a, &BLINK_1_ON_MS))
        .unwrap();

    let button_b = input_pin(button_b!(nrf_periph).degrade(), true);
    spawner
        .spawn(button_task(button_b, &BLINK_2_ON_MS))
        .unwrap();
}

#[panic_handler] // panicking behavior
unsafe fn panic(_pinfo: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
