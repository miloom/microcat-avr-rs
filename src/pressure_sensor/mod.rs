use crate::State;
use ms5837_02ba::{Ms5837_02ba, SensorData};

pub struct PressureSensor {
    ms5837_02ba: Ms5837_02ba,
}

impl PressureSensor {
    pub fn new(state: &mut State) -> Result<PressureSensor, atmega_hal::i2c::Error> {
        let ms5837_02ba = Ms5837_02ba::new(&mut state.i2c)?;
        Ok(PressureSensor { ms5837_02ba })
    }

    pub fn read(&self, state: &mut State) -> Option<SensorData> {
        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading pressure...\r").unwrap();
        let temp = self.ms5837_02ba.read(&mut state.i2c);
        if let Ok(data) = temp {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(
                &mut state.serial,
                "Got OK {}\r",
                if data.is_none() { "NONE" } else { "SOME" }
            )
            .unwrap();
            return data;
        } else {
            let _error_ret = temp.err().unwrap();

            #[cfg(feature = "logging")]
            ufmt::uwriteln!(
                &mut state.serial,
                "Got error {} {}\r",
                match _error_ret.0.kind() {
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
                _error_ret.1
            )
            .unwrap();
        }
        None
    }
}
