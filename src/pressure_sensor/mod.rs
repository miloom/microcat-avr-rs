use crate::State;
use atmega_hal::i2c::Error as I2cError;
#[cfg(feature = "logging")]
use atmega_hal::prelude::_unwrap_infallible_UnwrapInfallible as _;
#[cfg(feature = "logging")]
use embedded_hal::i2c::{Error as _, ErrorKind, NoAcknowledgeSource};
use ms5837_02ba::{Ms5837_02ba, SensorData};

pub struct PressureSensor {
    ms5837_02ba: Ms5837_02ba,
}

impl PressureSensor {
    pub fn new(state: &mut State) -> Result<Self, I2cError> {
        let ms5837_02ba = Ms5837_02ba::new(&mut state.i2c)?;
        Ok(Self { ms5837_02ba })
    }

    pub fn read(&self, state: &mut State) -> Option<SensorData> {
        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading pressure...\r").unwrap_infallible();
        let temp = self.ms5837_02ba.read(&mut state.i2c);
        if let Ok(data) = temp {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(
                &mut state.serial,
                "Got OK {}\r",
                if data.is_none() { "NONE" } else { "SOME" }
            )
            .unwrap_infallible();
            return data;
        }
        #[cfg(feature = "logging")]
        if let Some(error_ret) = temp.err() {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(
                &mut state.serial,
                "Got error {} {}\r",
                match error_ret.0.kind() {
                    ErrorKind::Bus => {
                        "Bus"
                    }
                    ErrorKind::ArbitrationLoss => {
                        "ArbitrationLoss"
                    }
                    ErrorKind::NoAcknowledge(source) => {
                        match source {
                            NoAcknowledgeSource::Address => "No ACK Address",
                            NoAcknowledgeSource::Data => "No ACK Data",
                            NoAcknowledgeSource::Unknown => "No ACK Unknown",
                        }
                    }
                    ErrorKind::Overrun => {
                        "Overrun"
                    }
                    ErrorKind::Other => {
                        "Other"
                    }
                    _ => {
                        ""
                    }
                },
                error_ret.1
            )
            .unwrap_infallible();
        }
        None
    }
}
