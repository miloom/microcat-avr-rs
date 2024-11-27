use crate::State;
use arduino_hal::prelude::*;
use heapless::Vec;
use micropb::{MessageDecode, PbDecoder};
use ufmt::{uwrite, uwriteln};

mod proto {

    #![allow(clippy::all)]
    #![allow(nonstandard_style, unused, irrefutable_let_patterns)]

    include!(concat!(env!("OUT_DIR"), "/proto.rs"));
}

pub fn read_serial(state: &mut State) {
    // let data = nb::block!(state.serial.read());
    while let Ok(data) = state.serial.read() {
        if data == 0 {
            state.serial_buf[state.serial_buf_idx] = 0;

            let cobs_decoded_data = decode_cobs(&state.serial_buf[..=state.serial_buf_idx]);

            if let Some(cobs_decoded_data) = cobs_decoded_data {
                if let Some(msg) = decode_proto(&cobs_decoded_data) {
                    match msg.data {
                        Some(proto::message_::Message_::Data::Motor(target)) => {
                            uwriteln!(
                                &mut state.serial,
                                "Found motor message {}\r",
                                target.target_position as i64
                            )
                            .unwrap_infallible();
                        }
                        Some(proto::message_::Message_::Data::Encoder(_)) => {}
                        Some(proto::message_::Message_::Data::Telemetry(_)) => {}
                        None => {}
                    };
                }
                for val in cobs_decoded_data {
                    uwrite!(&mut state.serial, "{:02x} ", val).unwrap_infallible();
                }
                uwriteln!(&mut state.serial, "{}\r", state.serial_buf_idx).unwrap_infallible();
            } else {
                uwriteln!(&mut state.serial, "Failed to decode using cobs\r").unwrap_infallible();
            }

            state.serial_buf_idx = 0;
        } else {
            state.serial_buf[state.serial_buf_idx] = data;
            state.serial_buf_idx += 1;
        }
    }
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
