use atmega_hal::port::mode::Floating;
use atmega_hal::port::{mode::Input, Pin};

pub struct ToneDetector {
    pin: Pin<Input<Floating>>,
}

impl ToneDetector {
    pub fn new(pin: Pin<Input<Floating>>) -> Self {
        Self { pin }
    }

    pub fn read(&self) -> bool {
        self.pin.is_high()
    }
}
