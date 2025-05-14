#![expect(
    clippy::arbitrary_source_item_ordering,
    reason = "Many locations make more sense to be ordered by P, I, D"
)]

use core::ops::{Div as _, Neg as _};

pub struct IntegerPID {
    kp: i32,         // Proportional coefficient (scaled)
    ki: i32,         // Integral coefficient (scaled)
    kd: i32,         // Derivative coefficient (scaled)
    scale: i16,      // Scale factor for fixed-point arithmetic
    prev_error: i32, // Previous error (for derivative calculation)
    integral: i32,   // Accumulated integral term
    output_min: i32, // Minimum output limit (scaled)
    output_max: i32, // Maximum output limit (scaled)
    p_max: i32,      // Maximum value of the proportional part (scaled)
    i_max: i32,      // Maximum value of the integral part (scaled)
    d_max: i32,      // Maximum value of the derivative part (scaled)
}

impl IntegerPID {
    #[expect(
        clippy::cast_possible_truncation,
        reason = "This truncating is expected"
    )]
    pub fn new(kp: f32, ki: f32, kd: f32, scale: i16, output_min: i32, output_max: i32) -> Self {
        Self {
            kp: (kp * f32::from(scale)) as i32,
            ki: (ki * f32::from(scale)) as i32,
            kd: (kd * f32::from(scale)) as i32,
            scale,
            prev_error: 0,
            integral: 0,
            output_min: output_min.saturating_mul(i32::from(scale)),
            output_max: output_max.saturating_mul(i32::from(scale)),
            p_max: output_max.saturating_mul(i32::from(scale)),
            i_max: output_max.saturating_mul(i32::from(scale)),
            d_max: output_max.saturating_mul(i32::from(scale)),
        }
    }

    #[expect(dead_code, reason = "This is a convenience function for future")]
    #[expect(
        clippy::cast_possible_truncation,
        reason = "This truncating is expected"
    )]
    pub fn p(&mut self, kp: f32, max: f32) -> &mut Self {
        self.kp = (kp * f32::from(self.scale)) as i32;
        self.p_max = (max * f32::from(self.scale)) as i32;
        self
    }

    #[expect(dead_code, reason = "This is a convenience function for future")]
    #[expect(
        clippy::cast_possible_truncation,
        reason = "This truncating is expected"
    )]
    pub fn i(&mut self, ki: f32, max: f32) -> &mut Self {
        self.ki = (ki * f32::from(self.scale)) as i32;
        self.i_max = (max * f32::from(self.scale)) as i32;
        self
    }

    #[expect(dead_code, reason = "This is a convenience function for future")]
    #[expect(
        clippy::cast_possible_truncation,
        reason = "This truncating is expected"
    )]
    pub fn d(&mut self, kd: f32, max: f32) -> &mut Self {
        self.kd = (kd * f32::from(self.scale)) as i32;
        self.d_max = (max * f32::from(self.scale)) as i32;
        self
    }

    #[expect(
        clippy::cast_precision_loss,
        reason = "Precision loss is acceptable to keep performance"
    )]
    pub fn compute(&mut self, error: i32) -> f32 {
        // Proportional term
        let p_term = (self.kp * error).clamp(self.p_max.neg(), self.p_max);

        // Integral term (with clamping to prevent integral windup)
        self.integral = (self.integral + error).clamp(self.i_max.neg(), self.i_max);
        let i_term = (self.ki * self.integral).clamp(self.i_max.neg(), self.i_max);

        // Derivative term
        let d_term =
            (self.kd * error.saturating_sub(self.prev_error)).clamp(self.d_max.neg(), self.d_max);
        self.prev_error = error;

        // Compute the output
        let mut output = p_term + i_term + d_term;

        // Clamp the output to the limits
        output = output.clamp(self.output_min, self.output_max);

        (output as f32).div(f32::from(self.scale))
    }
}
