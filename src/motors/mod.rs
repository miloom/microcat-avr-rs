mod as5040;
mod pid;

use crate::millis::millis;
use crate::motors::pid::IntegerPID;
use crate::State;
use as5040::{As5040, Encoder as _};
use atmega_hal::port::mode::Output;
use atmega_hal::port::Pin;
#[cfg(feature = "logging")]
use atmega_hal::prelude::_unwrap_infallible_UnwrapInfallible as _;
use core::f32::consts::PI;
use core::ops::{Div as _, Mul as _, Neg as _};
use drv8830::WriteRegister as _;
use micromath::F32Ext as _;
use strum::EnumIter;
#[cfg(feature = "logging")]
use ufmt::uwriteln;

pub struct MotorController {
    pub amplitude: u32,
    encoder: As5040,
    pub frequency: u32, // mHz
    location: MotorLocation,
    pid: IntegerPID,
    reversed: bool,
    pub target_position: i32,
}

impl MotorController {
    const MAX_SERVO_SPEED: i16 = 64;

    fn drive(&self, state: &mut State, speed: i16) {
        // Before we do anything, we'll want to
        //  clear the fault status. To do that
        //  write 0x80 to register 0x01 on the
        //  DRV8830.
        _ = drv8830::Fault {
            clear: true,
            ..Default::default()
        }
        .write(&mut state.i2c, self.location.i2c_address());

        let limited_speed = f32::from(speed.abs().min(Self::MAX_SERVO_SPEED));
        let speed_target = limited_speed.div(100.0);
        let mut control = if speed_target.abs() < 0.1 {
            drv8830::Control::BRAKE
        } else if speed > 0 {
            drv8830::Control::FORWARD
        } else {
            drv8830::Control::REVERSE
        };
        control.speed_mult = speed_target;
        _ = control.write(&mut state.i2c, self.location.i2c_address());
    }

    pub fn get_position(&mut self, state: &mut State) -> i16 {
        self.encoder.read(state)
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

    #[expect(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        reason = "Performance impact for checked cast is too high"
    )]
    pub fn update(&mut self, state: &mut State) {
        const ENCODER_MAX: i16 = 512; // Assumes range 0 - max
        const TWO_PI: f32 = PI * 2.0;

        let frequency_float = self.frequency as f32;
        let amplitude_float = self.amplitude as f32;
        // calculate generalized angular position based on amplitude and frequency
        let time = (millis() as f32).div(1000.0);
        let period = frequency_float.inv();
        let temp_time = fmodf(time, period);
        let gen_xt: f32 = amplitude_float
            .div(2.0)
            .mul(TWO_PI.mul(frequency_float).mul(temp_time).sin());

        let desired_position = self.target_position.wrapping_add(gen_xt as i32);
        let desired_position = if self.reversed {
            desired_position.neg()
        } else {
            desired_position
        };
        let shortest_path = (desired_position as i16).wrapping_sub(self.encoder.read(state));
        if let Some(val) = Self::wrap_angle(shortest_path, i32::from(ENCODER_MAX)) {
            let control = self.pid.compute(val);
            self.drive(state, control as i16);
        }
    }

    fn wrap_angle(value: i16, max: i32) -> Option<i32> {
        if max <= 0i32 {
            return None; // invalid max
        }

        let half_max = max >> 1i32;
        let val = i32::from(value);

        // Normalize angle into [0, MAX)
        let mut angle = val.rem_euclid(max);

        // Wrap into [-MAX/2, MAX/2]
        if angle > half_max {
            angle = angle.checked_sub(max)?;
        } else if angle < half_max.neg() {
            angle = angle.checked_add(max)?;
        }

        Some(angle)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumIter)]
pub enum MotorLocation {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

impl MotorLocation {
    pub const fn as_index(self) -> usize {
        match self {
            Self::FrontLeft => 0,
            Self::FrontRight => 1,
            Self::RearLeft => 2,
            Self::RearRight => 3,
        }
    }

    pub const fn i2c_address(self) -> u8 {
        match self {
            Self::FrontLeft => 0x61,
            Self::FrontRight => 0x63,
            Self::RearLeft => 0x64,
            Self::RearRight => 0x60,
        }
    }

    #[cfg_attr(
        not(feature = "logging"),
        expect(dead_code, reason = "Used for easier logging")
    )]
    pub const fn to_str(self) -> &'static str {
        match self {
            Self::FrontLeft => "FrontLeft",
            Self::FrontRight => "FrontRight",
            Self::RearLeft => "RearLeft",
            Self::RearRight => "RearRight",
        }
    }
}

pub struct MotorSystem {
    controllers: [Option<MotorController>; 4], // Use Option to track initialization
}

impl MotorSystem {
    pub fn get_motor_mut(&mut self, position: MotorLocation) -> Option<&mut MotorController> {
        let index = position.as_index();
        if let Some(motor) = self.controllers.get_mut(index) {
            return motor.as_mut();
        }
        None
    }

    pub fn initialize(&mut self, position: MotorLocation, pin: Pin<Output>, state: &mut State) {
        let index = position.as_index();
        if let Some(motor) = self.controllers.get_mut(index) {
            *motor = Some(MotorController::new(position, pin));
            if let Some(motor) = motor {
                motor.init(state);
            }
        }
    }

    pub const fn new() -> Self {
        Self {
            controllers: [None, None, None, None],
        }
    }
}

#[expect(
    clippy::float_arithmetic,
    reason = "This function will have to work with floating point numbers"
)]
fn fmodf(x: f32, y: f32) -> f32 {
    x - y * (x / y).floor()
}
