#![allow(dead_code)]
use crate::State;
use arduino_hal::delay_ms;
use arduino_hal::hal::port::Dynamic;
use arduino_hal::port::mode::Output;
use arduino_hal::port::Pin;
use arduino_hal::prelude::_unwrap_infallible_UnwrapInfallible;
use embedded_hal::spi::SpiBus;
#[cfg(feature = "logging")]
use ufmt::uwriteln;

pub trait Encoder {
    fn init(&mut self, state: &mut State) -> Result<(), ()>;
    fn read(&mut self, state: &mut State) -> i16;
}

pub(crate) struct As5040 {
    pub cs: Pin<Output, Dynamic>,
    status: u8,
    parity: u8,
}

impl As5040 {
    // defines for 5 bit _status value
    const AS5040_STATUS_OCF: u16 = 0x10;
    const AS5040_STATUS_COF: u16 = 0x08;
    const AS5040_STATUS_LIN: u16 = 0x04;
    const AS5040_STATUS_MAGINC: u16 = 0x02;
    const AS5040_STATUS_MAGDEC: u16 = 0x01;

    pub fn new(cs: Pin<Output, Dynamic>) -> Self {
        Self {
            cs,
            status: 0,
            parity: 0,
        }
    }
    fn single_read(&mut self, state: &mut State) -> i16 {
        let mut buffer = [0u8; 2];
        self.cs.set_low();
        state.spi.read(&mut buffer).unwrap_infallible();
        self.cs.set_high();
        let value = (u16::from_be_bytes(buffer) >> 6) as i16;
        self.status = buffer[1] & 0x1F;
        self.parity = Self::even_parity((value >> 2) as u8)
            ^ Self::even_parity((value & 3) as u8)
            ^ Self::even_parity(self.status);
        value
    }

    fn even_parity(val: u8) -> u8 {
        let val = (val >> 1) ^ val;
        let val = (val >> 2) ^ val;
        let val = (val >> 4) ^ val;
        val & 1
    }
}

impl Encoder for As5040 {
    fn init(&mut self, state: &mut State) -> Result<(), ()> {
        let mut count = 0;
        loop {
            self.single_read(state);
            #[cfg(feature = "logging")]
            uwriteln!(&mut state.serial, "Status: {:x}\r", self.status).unwrap();
            if self.status & Self::AS5040_STATUS_OCF as u8 != 0 {
                break;
            }
            if count > 30 {
                return Err(());
            }
            delay_ms(1);
            count += 1;
        }
        Ok(())
    }

    fn read(&mut self, state: &mut State) -> i16 {
        self.single_read(state)
    }
}
