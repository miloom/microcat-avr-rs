use crate::State;
use arduino_hal::prelude::*;
use heapless::Vec;
use micropb::{MessageDecode, MessageEncode, PbDecoder, PbEncoder};
#[cfg(feature = "logging")]
use ufmt::uwriteln;

mod proto {
    #![allow(clippy::all)]
    #![allow(nonstandard_style, unused, irrefutable_let_patterns)]
    include!(concat!(env!("OUT_DIR"), "/proto.rs"));
}

pub struct MotorCommand {
    pub location: crate::MotorLocation,
    pub position: i32,
    pub frequency: u32,
    pub amplitude: u32,
}

pub enum Command {
    MotorCommand(MotorCommand),
}

pub fn read_serial(state: &mut State) -> Option<Command> {
    // let data = nb::block!(state.serial.read());
    while let Ok(data) = state.serial.read() {
        if data == 0 {
            state.serial_buf[state.serial_buf_idx] = 0;

            let cobs_decoded_data = decode_cobs(&state.serial_buf[..=state.serial_buf_idx]);

            if let Some(cobs_decoded_data) = cobs_decoded_data {
                if let Some(msg) = decode_proto(&cobs_decoded_data) {
                    match msg.data {
                        Some(proto::message_::Message_::Data::MotorTarget(target)) => {
                            let location = match target.location {
                                proto::motor_::Location::FrontLeft => {
                                    crate::MotorLocation::FrontLeft
                                }
                                proto::motor_::Location::FrontRight => {
                                    crate::MotorLocation::FrontRight
                                }
                                proto::motor_::Location::BackLeft => crate::MotorLocation::RearLeft,
                                proto::motor_::Location::BackRight => {
                                    crate::MotorLocation::RearRight
                                }
                                _ => {
                                    unreachable!();
                                }
                            };
                            return Some(Command::MotorCommand(MotorCommand {
                                frequency: target.frequency,
                                amplitude: target.amplitude,
                                position: target.target_position,
                                location,
                            }));
                        }
                        Some(proto::message_::Message_::Data::Encoder(_)) => {}
                        Some(proto::message_::Message_::Data::Imu(_)) => {}
                        Some(proto::message_::Message_::Data::MotorPosition(_)) => {}
                        Some(proto::message_::Message_::Data::ToneDetectorStatus(_)) => {}
                        None => {}
                    };
                }
            } else {
                #[cfg(feature = "logging")]
                uwriteln!(&mut state.serial, "Failed to decode using cobs\r").unwrap_infallible();
            }

            state.serial_buf_idx = 0;
        } else {
            state.serial_buf[state.serial_buf_idx] = data;
            state.serial_buf_idx += 1;
        }
    }
    None
}

pub struct ImuReading {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16, // x_angular_rate = gyro_x / 131 LSB(º/s)
    pub gyro_y: i16, // y_angular_rate = gyro_y / 131 LSB(º/s)
    pub gyro_z: i16, // z_angular_rate = gyro_z / 131 LSB(º/s)
}

pub struct MotorPosition {
    pub location: crate::MotorLocation,
    pub position: i16,
}

pub enum ToneDetectorLocation {
    Right,
    Left,
}

pub struct ToneDetectorStatus {
    pub location: ToneDetectorLocation,
    pub is_high: bool,
}

pub enum Telemetry {
    Imu(ImuReading),
    MotorPosition(MotorPosition),
    ToneDetector(ToneDetectorStatus),
}

pub fn write(state: &mut State, telemetry: Telemetry) {
    let message = proto::message_::Message {
        data: Some(match telemetry {
            Telemetry::Imu(reading) => {
                let mut data = proto::imu_::Telemetry::default();
                data.set_accel(proto::imu_::AccelData {
                    x: reading.accel_x as i32,
                    y: reading.accel_y as i32,
                    z: reading.accel_z as i32,
                });
                data.set_gyro(proto::imu_::GyroData {
                    x: reading.gyro_x as i32,
                    y: reading.gyro_y as i32,
                    z: reading.gyro_z as i32,
                });
                proto::message_::Message_::Data::Imu(data)
            }
            Telemetry::MotorPosition(position) => {
                let data = proto::motor_::MotorPosition {
                position: position.position as i32,
                location: match position.location {
                    crate::MotorLocation::RearLeft => {
                        proto::motor_::Location::BackLeft
                    },
                    crate::MotorLocation::FrontLeft => {
                        proto::motor_::Location::FrontLeft
                    },
                    crate::MotorLocation::FrontRight => {
                        proto::motor_::Location::FrontRight
                    },
                    crate::MotorLocation::RearRight => {
                        proto::motor_::Location::BackRight
                    }
                }};
                proto::message_::Message_::Data::MotorPosition(data)
            }
            Telemetry::ToneDetector(tone_detector) => {
                let data = proto::tone_detector_::ToneDetectorStatus {
                    location: match tone_detector.location { 
                        ToneDetectorLocation::Left => {
                            proto::tone_detector_::Location::Left
                        }
                        ToneDetectorLocation::Right => {
                            proto::tone_detector_::Location::Right
                        }
                    },
                    is_high: tone_detector.is_high
                };
                proto::message_::Message_::Data::ToneDetectorStatus(data)
            }
        }),
    };
    let mut encoder = PbEncoder::new(Vec::<u8, 64>::new());
    if message.encode(&mut encoder).is_ok() {
        let writer = encoder.into_writer();
        let (cobs_encoded, len) = encode_cobs(&writer);
        for data in cobs_encoded.iter().take(len) {
            state.serial.write_byte(*data);
        }
    }
}

fn encode_cobs(data: &[u8]) -> (Vec<u8, 128>, usize) {
    let mut temporary = [0, 128];
    let len = cobs::encode(data, &mut temporary);
    (
        Vec::from_slice(&temporary[..len]).unwrap_or(Vec::new()),
        len,
    )
}

fn decode_cobs(data: &[u8]) -> Option<Vec<u8, 128>> {
    let mut temporary = [0; 128];
    if let Ok(len) = cobs::decode(data, &mut temporary) {
        return Some(Vec::from_slice(&temporary[..len]).unwrap_or(Vec::new()));
    }
    None
}

fn decode_proto(data: &[u8]) -> Option<proto::message_::Message> {
    let mut decoder = PbDecoder::new(data);
    let mut msg = proto::message_::Message::default();
    if let Ok(()) = msg.decode(&mut decoder, data.len()) {
        return Some(msg);
    }
    None
}
