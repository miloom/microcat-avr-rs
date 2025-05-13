use crate::{CoreClock, State};
use atmega_hal::port::mode::{Input, Output};
use atmega_hal::port::{Pin, PD0, PD1};
use atmega_hal::prelude::_unwrap_infallible_UnwrapInfallible;
use atmega_hal::{pac, Usart};
use avr_device::atmega328p::USART0;
use avr_device::{interrupt, interrupt::Mutex};
use core::cell;
use ufmt::uwriteln;

static MILLIS_COUNTER: Mutex<cell::Cell<u32>> = Mutex::new(cell::Cell::new(0));

const MILLIS_INCREMENT: u32 = PRESCALER * TIMER_COUNTS / 16000;
const PRESCALER: u32 = 1024;
const TIMER_COUNTS: u32 = 125;

pub fn init(tc0: &pac::TC0) {
    // Configure the timer for the above interval (in CTC mode)
    // and enable its interrupt.
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    #[expect(
        clippy::cast_possible_truncation,
        reason = "We want to only get the lowest 8 bits"
    )]
    tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));
    tc0.tccr0b.write(|w| match PRESCALER {
        8 => w.cs0().prescale_8(),
        64 => w.cs0().prescale_64(),
        256 => w.cs0().prescale_256(),
        1024 => w.cs0().prescale_1024(),
        #[expect(
            clippy::panic,
            reason = "If we fail to match prescaler we are unable to run our main loop"
        )]
        _ => panic!(),
    });
    tc0.timsk0.write(|w| w.ocie0a().set_bit());

    // Reset the global millisecond counter
    interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    });
}

#[avr_device::interrupt(atmega328p)]
#[expect(
    clippy::single_call_fn,
    reason = "Interrupts are only triggered by internal interrupts"
)]
fn TIMER0_COMPA() {
    interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter.wrapping_add(MILLIS_INCREMENT));
    });
}

#[expect(
    clippy::single_call_fn,
    reason = "Currently only used once, but kept for convenience"
)]
pub fn millis() -> u32 {
    interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
}
