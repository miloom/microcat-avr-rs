use crate::State;
use atmega_hal::i2c::Error as I2cError;
#[cfg(feature = "log_info")]
use atmega_hal::prelude::_unwrap_infallible_UnwrapInfallible as _;
#[cfg(feature = "log_info")]
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
        let temp = self.ms5837_02ba.read(&mut state.i2c);
        if let Ok(data) = temp {
            return data;
        }
        None
    }
}
