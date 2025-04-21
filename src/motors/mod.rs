use crate::millis::millis;
use crate::motors::pid::IntegerPID;
use crate::State;
#[cfg(feature = "logging")]
use arduino_hal::prelude::_unwrap_infallible_UnwrapInfallible;
use as5040::{As5040, Encoder};
use atmega_hal::port::mode::Output;
use atmega_hal::port::Pin;
use core::f32::consts::PI;
use drv8830::WriteRegister;
use micromath::F32Ext;
use strum::EnumIter;
#[cfg(feature = "logging")]
use ufmt::uwriteln;
mod as5040;
mod pid;

#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumIter)]
pub enum MotorLocation {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

fn fmodf(x: f32, y: f32) -> f32 {
    x - y * (x / y).floor()
}

impl MotorLocation {
    #[allow(unused)]
    pub const fn to_str(self) -> &'static str {
        match self {
            MotorLocation::FrontLeft => "FrontLeft",
            MotorLocation::FrontRight => "FrontRight",
            MotorLocation::RearLeft => "RearLeft",
            MotorLocation::RearRight => "RearRight",
        }
    }
    pub const fn as_index(self) -> usize {
        match self {
            MotorLocation::FrontLeft => 0,
            MotorLocation::FrontRight => 1,
            MotorLocation::RearLeft => 2,
            MotorLocation::RearRight => 3,
        }
    }

    pub const fn i2c_address(self) -> u8 {
        match self {
            MotorLocation::FrontLeft => 0x61,
            MotorLocation::FrontRight => 0x63,
            MotorLocation::RearLeft => 0x64,
            MotorLocation::RearRight => 0x60,
        }
    }
}

pub struct MotorController {
    encoder: As5040,
    location: MotorLocation,
    pub frequency: u32, // mHz
    pub amplitude: u32,
    reversed: bool,
    pub target_position: i32,
    pid: IntegerPID,
}

impl MotorController {
    const MAX_SERVO_SPEED: i16 = 64;
    pub fn new(location: MotorLocation, cs: Pin<Output>) -> Self {
        Self {
            encoder: As5040::new(cs),
            location,
            frequency: 200,
            amplitude: 100,
            reversed: false,
            target_position: 0,
            pid: IntegerPID::new(0.9, 0.05, 0.0, 100, -100, 100),
        }
    }

    pub fn init(&mut self, state: &mut State) {
        if self.encoder.init(state).is_err() {
            #[cfg(feature = "logging")]
            uwriteln!(
                &mut state.serial,
                "Failed to init {} encoder \r",
                self.location.to_str()
            )
            .unwrap_infallible();
        }
    }

    pub fn get_position(&mut self, state: &mut State) -> i16 {
        self.encoder.read(state)
    }

    pub fn update(&mut self, state: &mut State) {
        let frequency_float = self.frequency as f32;
        let amplitude_float = self.amplitude as f32;
        // calculate generalized angular position based on amplitude and frequency
        let time = millis() as f32 / 1000.0;
        let period = 1.0 / frequency_float;
        let temp_time = fmodf(time, period);
        let gen_xt: f32 = amplitude_float / 2.0 * (2.0 * PI * frequency_float * temp_time).sin();

        let desired_position = self.target_position + gen_xt as i32;
        let desired_position = if self.reversed {
            -desired_position
        } else {
            desired_position
        };
        let shortest_path = desired_position as i16 - self.encoder.read(state);
        const ENCODER_MAX: i16 = 512; // Assumes range 0 - max
        let shortest_path = ((shortest_path + ENCODER_MAX / 2) % ENCODER_MAX + ENCODER_MAX)
            % ENCODER_MAX
            - ENCODER_MAX / 2;
        let control = self.pid.compute(shortest_path as i32);
        self.drive(state, control as i16);
    }

    fn drive(&self, state: &mut State, speed: i16) {
        // Before we do anything, we'll want to
        //  clear the fault status. To do that
        //  write 0x80 to register 0x01 on the
        //  DRV8830.
        let _ = drv8830::Fault {
            clear: true,
            ..Default::default()
        }
        .write(&mut state.i2c, self.location.i2c_address());

        let limited_speed = speed.abs().min(Self::MAX_SERVO_SPEED) as f32;
        let speed_target = limited_speed / 100.0;
        let mut control = if speed_target.abs() < 0.1 {
            drv8830::Control::BRAKE
        } else if speed > 0 {
            drv8830::Control::FORWARD
        } else {
            drv8830::Control::REVERSE
        };
        control.speed_mult = speed_target;
        let _ = control.write(&mut state.i2c, self.location.i2c_address());
    }
}

pub struct MotorSystem {
    controllers: [Option<MotorController>; 4], // Use Option to track initialization
}

impl MotorSystem {
    pub const fn new() -> Self {
        Self {
            controllers: [None, None, None, None],
        }
    }

    pub fn initialize(&mut self, position: MotorLocation, pin: Pin<Output>, state: &mut State) {
        let index = position.as_index();
        self.controllers[index] = Some(MotorController::new(position, pin));
        self.controllers[index].as_mut().unwrap().init(state);
    }

    pub fn get_motor_mut(&mut self, position: MotorLocation) -> Option<&mut MotorController> {
        let index = position.as_index();
        self.controllers[index].as_mut()
    }
}
