use avr_device::atmega328p::tc1::tccr1b::CS1_A;
use avr_device::atmega328p::TC1;

macro_rules! const_int_cast {
    ($value:expr, $to:ty) => {{
        const OUT: $to = {
            // Validate 128-bit is not used
            #[expect(
                clippy::indexing_slicing,
                reason = "We want it to panic if we use 128-bit types"
            )]
            const _: () = [()][(core::mem::size_of::<$to>() == 16
                || core::mem::size_of_val(&$value) == 16) as usize];
            // Bounds check
            const _: () = assert!(
                ($value as i128) >= <$to>::MIN as i128 && ($value as i128) <= <$to>::MAX as i128,
                "const_int_cast: value out of bounds"
            );

            $value as $to
        };
        OUT
    }};
}

pub const fn calc_overflow(clock_hz: u32, target_hz: u32, prescale: u32) -> u32 {
    /*
    https://github.com/Rahix/avr-hal/issues/75
    reversing the formula F = 16 MHz / (256 * (1 + 15624)) = 4 Hz
     */
    clock_hz / target_hz / prescale - 1
}

pub fn rig_timer(tmr1: &TC1) {
    /*
     https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
     section 15.11
    */
    use atmega_hal::clock::Clock as _;

    const CORE_CLOCK_FREQ: u32 = crate::CoreClock::FREQ;
    const CLOCK_SOURCE: CS1_A = CS1_A::PRESCALE_256;
    const CLOCK_DIVISOR: u32 = match CLOCK_SOURCE {
        CS1_A::PRESCALE_8 => 8,
        CS1_A::PRESCALE_64 => 64,
        CS1_A::PRESCALE_256 => 256,
        CS1_A::PRESCALE_1024 => 1024,
        CS1_A::DIRECT | CS1_A::NO_CLOCK | CS1_A::EXT_FALLING | CS1_A::EXT_RISING => 1,
    };

    #[expect(
        clippy::cast_possible_truncation,
        reason = "We manually checked that the cast will not truncate"
    )]
    const TICKS: u16 = calc_overflow(CORE_CLOCK_FREQ, 10, CLOCK_DIVISOR) as u16;

    tmr1.tccr1a.write(|w| w.wgm1().bits(0b00));
    tmr1.tccr1b.write(|w| {
        w.cs1()
            //.prescale_256()
            .variant(CLOCK_SOURCE)
            .wgm1()
            .bits(0b01)
    });
    tmr1.ocr1a.write(|w| w.bits(TICKS));
    tmr1.timsk1.write(|w| w.ocie1a().set_bit()); //enable this specific interrupt
}
