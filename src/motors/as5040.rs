use crate::State;
use atmega_hal::port::mode::Output;
use atmega_hal::port::Dynamic;
use atmega_hal::port::Pin;
use atmega_hal::prelude::_unwrap_infallible_UnwrapInfallible as _;
use embedded_hal::spi::SpiBus as _;
#[cfg(feature = "logging")]
use ufmt::uwriteln;

pub struct As5040 {
    pub cs: Pin<Output, Dynamic>,
    parity: u8,
    status: u8,
}

impl As5040 {
    // defines for 5 bit status value
    #[expect(
        dead_code,
        reason = "These are only needed when writing logic for return codes"
    )]
    const AS5040_STATUS_COF: u16 = 0x08;
    #[expect(
        dead_code,
        reason = "These are only needed when writing logic for return codes"
    )]
    const AS5040_STATUS_LIN: u16 = 0x04;
    #[expect(
        dead_code,
        reason = "These are only needed when writing logic for return codes"
    )]
    const AS5040_STATUS_MAGDEC: u16 = 0x01;
    #[expect(
        dead_code,
        reason = "These are only needed when writing logic for return codes"
    )]
    const AS5040_STATUS_MAGINC: u16 = 0x02;
    const AS5040_STATUS_OCF: u16 = 0x10;

    const fn even_parity(val: u8) -> u8 {
        let val = (val >> 1u8) ^ val;
        let val = (val >> 2u8) ^ val;
        let val = (val >> 4u8) ^ val;
        val & 1u8
    }

    pub const fn new(cs: Pin<Output, Dynamic>) -> Self {
        Self {
            cs,
            status: 0,
            parity: 0,
        }
    }

    #[expect(
        clippy::cast_sign_loss,
        reason = "Bitshift will ensure correct behaviour"
    )]
    fn single_read(&mut self, state: &mut State) -> i16 {
        let mut buffer = [0u8; 2];
        self.cs.set_low();
        state.spi.read(&mut buffer).unwrap_infallible();
        self.cs.set_high();
        #[expect(
            clippy::cast_possible_wrap,
            reason = "We use bitshift to guarantee safety"
        )]
        let value = (u16::from_be_bytes(buffer) >> 6i16) as i16;
        self.status = buffer[1] & 0x1F;
        self.parity = Self::even_parity((value >> 2i16) as u8)
            ^ Self::even_parity((value & 3i16) as u8)
            ^ Self::even_parity(self.status);
        value
    }
}

impl Encoder for As5040 {
    #[expect(clippy::cast_possible_truncation, reason = "Expected behaviour")]
    fn init(&mut self, state: &mut State) -> Result<(), ()> {
        let mut count = 0u32;
        loop {
            self.single_read(state);
            #[cfg(feature = "logging")]
            uwriteln!(&mut state.serial, "Status: {:x}\r", self.status).unwrap_infallible();
            if self.status & Self::AS5040_STATUS_OCF as u8 != 0 {
                break;
            }
            if count > 30 {
                return Err(());
            }
            count = count.saturating_add(1);
        }
        Ok(())
    }

    fn read(&mut self, state: &mut State) -> i16 {
        self.single_read(state)
    }
}

pub trait Encoder {
    fn init(&mut self, state: &mut State) -> Result<(), ()>;
    fn read(&mut self, state: &mut State) -> i16;
}
