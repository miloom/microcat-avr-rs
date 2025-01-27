#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

extern crate core;

use arduino_hal::{I2c, Spi, Usart};
use arduino_hal::hal::port::*;
use arduino_hal::pac::USART0;
use arduino_hal::port::mode::{Input, Output};
use arduino_hal::spi;
use panic_halt as _;
use strum::IntoEnumIterator;
use crate::serial::{read_serial, Command};

use motors::{MotorLocation, MotorSystem};

struct State {
    serial: Usart<USART0, Pin<Input, PD0>, Pin<Output, PD1>>,
    serial_buf: [u8; 128],
    serial_buf_idx: usize,
    i2c: I2c,
    spi: Spi,
}

mod serial;
mod motors;
mod millis;

#[arduino_hal::entry]
fn main() -> ! {

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let serial = arduino_hal::default_serial!(dp, pins, 115200);
    millis::init(dp.TC0);

    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100_000,
    );
    let (spi, d10) = arduino_hal::Spi::new(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12.into_pull_up_input(),
        pins.d10.into_output(),
        spi::Settings{
            data_order: spi::DataOrder::MostSignificantFirst,
            clock: spi::SerialClockRate::OscfOver16,
            mode: embedded_hal::spi::Mode {
                polarity: embedded_hal::spi::Polarity::IdleHigh,
                phase: embedded_hal::spi::Phase::CaptureOnSecondTransition,
            }
        },
    );

    let mut led = pins.a0.into_output();
    let mut motor_power = pins.a3.into_output();
    motor_power.set_high();

    let mut state = State {
        serial,
        serial_buf: [0; 128],
        serial_buf_idx: 0,
        i2c,
        spi,
    };

    let mut motor_system = MotorSystem::new();
    motor_system.initialize(MotorLocation::FrontLeft, pins.d8.into_output().downgrade(), &mut state);
    motor_system.initialize(MotorLocation::FrontRight, pins.d7.into_output().downgrade(), &mut state);
    motor_system.initialize(MotorLocation::RearLeft, pins.d9.into_output().downgrade(), &mut state);
    unsafe {
        // We will never remove the pin from this motor and the motor will never turn it into input so this is safe.
        motor_system.initialize(MotorLocation::RearLeft, d10.into_pin_unchecked().downgrade(), &mut state);
    }



    ufmt::uwriteln!(&mut state.serial, "Starting...\r").unwrap();



    let mut loop_counter = 0;
    loop {
        if loop_counter == 0 {
            led.toggle();
        }
        loop_counter += 1;
        if let Some(command) = read_serial(&mut state) {
            match command {
                Command::MotorCommand(command) => {
                    if let Some(motor) = motor_system.get_motor_mut(command.location) {
                        motor.target_position = command.position;
                        motor.amplitude = command.amplitude;
                        motor.frequency = command.frequency;
                    }
                }
            };
        }
        for location in MotorLocation::iter() {
            if let Some(motor) = motor_system.get_motor_mut(location) {
                motor.update(&mut state);
            }
        }
        arduino_hal::delay_ms(10);
    }
}

