use arduino_hal::hal::port::{Pin, mode::Input};
use arduino_hal::port::mode::Floating;

pub struct ToneDetector {
    pin: Pin<Input<Floating>>
}

impl ToneDetector {
    pub fn new(pin: Pin<Input<Floating>>) -> Self {
        Self { pin }
    }
    
    pub fn read(&self) -> bool {
        self.pin.is_high()
    }
}