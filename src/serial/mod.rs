#[expect(
    clippy::all,
    clippy::as_underscore,
    clippy::absolute_paths,
    clippy::arithmetic_side_effects,
    clippy::shadow_same,
    clippy::question_mark_used,
    clippy::arbitrary_source_item_ordering,
    clippy::missing_trait_methods,
    clippy::default_trait_access,
    clippy::too_many_lines,
    clippy::module_name_repetitions,
    clippy::default_numeric_fallback,
    clippy::semicolon_inside_block,
    clippy::derive_partial_eq_without_eq,
    clippy::missing_const_for_fn,
    reason = "No use in linting generated code"
)]
#[expect(
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    unused_imports,
    non_camel_case_types,
    reason = "No use in linting generated code"
)]
mod proto;

use crate::millis::millis;
use crate::serial::proto::sync_::TimeMeasurement;
use crate::serial::Telemetry::TimeSync;
use crate::State;
use atmega_hal::prelude::*;
use cobs::decode;
use heapless::Vec;
use micropb::{MessageDecode as _, MessageEncode as _, PbDecoder, PbEncoder};
#[cfg(feature = "log_info")]
use ufmt::uwriteln;

pub enum Command {
    MotorCommand(MotorCommand),
}

pub struct ImuReading {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16, // x_angular_rate = gyro_x / 131 LSB(º/s)
    pub gyro_y: i16, // y_angular_rate = gyro_y / 131 LSB(º/s)
    pub gyro_z: i16, // z_angular_rate = gyro_z / 131 LSB(º/s)
}

pub struct MotorCommand {
    pub amplitude: u32,
    pub frequency: u32,
    pub location: crate::MotorLocation,
    pub position: i32,
}

pub struct MotorPosition {
    pub location: crate::MotorLocation,
    pub position: i16,
}

pub struct PressureValues {
    pub pressure: i32,    // 0.01 mBar
    pub temperature: i32, // 0.01 deg_C
}

pub enum Telemetry {
    MotorPosition(MotorPosition),
    TimeSync(proto::message_::Message_::Data),
    Time(TimeMeasurement),
}

pub enum ToneDetectorLocation {
    Left,
    Right,
}

pub struct ToneDetectorStatus {
    pub is_high: bool,
    pub location: ToneDetectorLocation,
}

fn decode_cobs(data: &[u8]) -> Option<([u8; 128], usize)> {
    let mut temporary = [0; 128];
    let result = decode(data, &mut temporary);
    if let Ok(size) = result {
        return Some((temporary, size));
    }
    None
}

fn decode_proto(data: &[u8]) -> Option<proto::message_::Message> {
    let mut decoder = PbDecoder::new(data);
    let mut msg = proto::message_::Message::default();
    if msg.decode(&mut decoder, data.len()).is_ok() {
        return Some(msg);
    }
    None
}

fn encode_cobs(data: &[u8]) -> ([u8; 128], usize) {
    let mut temporary = [0; 128];
    let len = cobs::encode(data, &mut temporary);
    if let Some(last) = temporary.get_mut(len) {
        *last = 0;
        return (temporary, len + 1);
    }
    (temporary, 0)
}

#[expect(
    clippy::single_call_fn,
    reason = "Currently only used once, but kept for convenience"
)]
pub fn read_serial(state: &mut State, time_count: &mut u32) -> Option<Command> {
    while let Ok(data) = state.serial.read() {
        if data == 0 {
            if let Some(last) = state.serial_buf.get_mut(state.serial_buf_idx) {
                *last = 0;
                state.serial_buf_idx += 1;
            } else {
                state.serial_buf_idx = 0;
                #[cfg(feature = "log_info")]
                uwriteln!(&mut state.serial, "Failed to write last byte for message\r")
                    .unwrap_infallible();
                return None;
            }

            let cobs_decoded_data =
                if let Some(bytes) = state.serial_buf.get(..state.serial_buf_idx) {
                    #[cfg(feature = "log_info")]
                    uwriteln!(&mut state.serial, "Decoding with COBS\r").unwrap_infallible();
                    decode_cobs(bytes)
                } else {
                    state.serial_buf_idx = 0;
                    return None;
                };

            if let Some((cobs_decoded_data, cobs_decoded_size)) = cobs_decoded_data {
                #[cfg(feature = "log_info")]
                uwriteln!(&mut state.serial, "Decoding with proto\r").unwrap_infallible();
                if let Some(msg) = decode_proto(&cobs_decoded_data[..cobs_decoded_size]) {
                    match msg.data {
                        Some(proto::message_::Message_::Data::MotorTarget(target)) => {
                            #[cfg(feature = "log_info")]
                            uwriteln!(&mut state.serial, "Motor target received\r")
                                .unwrap_infallible();
                            let location = match target.location {
                                proto::motor_::Location::FrontLeft => {
                                    #[cfg(feature = "log_info")]
                                    uwriteln!(&mut state.serial, "Front left\r")
                                        .unwrap_infallible();
                                    write(
                                        state,
                                        Telemetry::Time(TimeMeasurement {
                                            time_ms: i64::from(millis()),
                                            count: *time_count,
                                        }),
                                    );
                                    *time_count += 1;
                                    crate::MotorLocation::FrontLeft
                                }
                                proto::motor_::Location::FrontRight => {
                                    #[cfg(feature = "log_info")]
                                    uwriteln!(&mut state.serial, "Front Right\r")
                                        .unwrap_infallible();
                                    crate::MotorLocation::FrontRight
                                }
                                proto::motor_::Location::BackLeft => {
                                    #[cfg(feature = "log_info")]
                                    uwriteln!(&mut state.serial, "Back left\r").unwrap_infallible();
                                    crate::MotorLocation::RearLeft
                                }
                                proto::motor_::Location::BackRight => {
                                    #[cfg(feature = "log_info")]
                                    uwriteln!(&mut state.serial, "Back Right\r")
                                        .unwrap_infallible();
                                    crate::MotorLocation::RearRight
                                }
                                _ => {
                                    state.serial_buf_idx = 0;
                                    #[cfg(feature = "log_info")]
                                    uwriteln!(
                                        &mut state.serial,
                                        "Failed to decode motor location\r"
                                    )
                                    .unwrap_infallible();
                                    return None;
                                }
                            };
                            #[cfg(feature = "log_info")]
                            uwriteln!(
                                &mut state.serial,
                                "freq: {} amp: {} pos: {}\r",
                                target.frequency,
                                target.amplitude,
                                target.target_position
                            )
                            .unwrap_infallible();
                            return Some(Command::MotorCommand(MotorCommand {
                                frequency: target.frequency,
                                amplitude: target.amplitude,
                                position: target.target_position,
                                location,
                            }));
                        }

                        Some(
                            proto::message_::Message_::Data::MotorPosition(_)
                            | proto::message_::Message_::Data::ResponseSync(_)
                            | proto::message_::Message_::Data::Time(_),
                        )
                        | None => {}
                        Some(proto::message_::Message_::Data::InitSync(msg)) => {
                            let t2 = millis();
                            write(
                                state,
                                TimeSync(proto::message_::Message_::Data::ResponseSync(
                                    proto::sync_::Response {
                                        delay_ms: i64::from(t2) - msg.t1_ms,
                                        t3_ms: i64::from(millis()),
                                    },
                                )),
                            );
                        }
                    };
                } else {
                    #[cfg(feature = "log_info")]
                    uwriteln!(&mut state.serial, "Failed to decode using proto\r")
                        .unwrap_infallible();
                }
            } else {
                #[cfg(feature = "log_info")]
                uwriteln!(&mut state.serial, "Failed to decode using cobs\r").unwrap_infallible();
            }

            state.serial_buf_idx = 0;
        } else if let Some(last) = state.serial_buf.get_mut(state.serial_buf_idx) {
            *last = data;
            state.serial_buf_idx += 1;
        } else {
            #[cfg(feature = "log_info")]
            uwriteln!(&mut state.serial, "Failed to write byte {:#04X}\r", data)
                .unwrap_infallible();
        }
    }
    None
}

pub fn write(state: &mut State, telemetry: Telemetry) {
    {
        let message = proto::message_::Message {
            data: Some(match telemetry {
                Telemetry::MotorPosition(position) => {
                    let data = proto::motor_::MotorPosition {
                        position: i32::from(position.position),
                        location: match position.location {
                            crate::MotorLocation::RearLeft => proto::motor_::Location::BackLeft,
                            crate::MotorLocation::FrontLeft => proto::motor_::Location::FrontLeft,
                            crate::MotorLocation::FrontRight => proto::motor_::Location::FrontRight,
                            crate::MotorLocation::RearRight => proto::motor_::Location::BackRight,
                        },
                    };
                    proto::message_::Message_::Data::MotorPosition(data)
                }
                TimeSync(data) => data,
                Telemetry::Time(data) => proto::message_::Message_::Data::Time(data),
            }),
        };
        let mut encoder = PbEncoder::new(Vec::<u8, 64>::new());
        if message.encode(&mut encoder).is_ok() {
            let writer = encoder.into_writer();
            let (cobs_encoded, len) = crate::serial::encode_cobs(&writer);
            for data in cobs_encoded.iter().take(len) {
                state.serial.write_byte(*data);
            }
        }
    }
}
