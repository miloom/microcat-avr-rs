#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

extern crate core;

use crate::serial::{read_serial, Command, PressureValues, Telemetry};
use arduino_hal::hal::port::*;
use arduino_hal::i2c::Direction;
use arduino_hal::pac::USART0;
use arduino_hal::port::mode::{Input, Output};
use arduino_hal::prelude::_unwrap_infallible_UnwrapInfallible;
use arduino_hal::spi;
use arduino_hal::{I2c, Spi, Usart};
use panic_halt as _;
use strum::IntoEnumIterator;

use motors::{MotorLocation, MotorSystem};

struct State {
    serial: Usart<USART0, Pin<Input, PD0>, Pin<Output, PD1>>,
    serial_buf: [u8; 128],
    serial_buf_idx: usize,
    i2c: I2c,
    spi: Spi,
}

mod imu;
mod millis;
mod motors;
mod pressure_sensor;
mod serial;
mod tone_detector;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut led = pins.a0.into_output();

    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);
    led.set_high();

    millis::init(dp.TC0);
    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut serial, "Millis done...\r").unwrap();

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
        spi::Settings {
            data_order: spi::DataOrder::MostSignificantFirst,
            clock: spi::SerialClockRate::OscfOver16,
            mode: embedded_hal::spi::Mode {
                polarity: embedded_hal::spi::Polarity::IdleHigh,
                phase: embedded_hal::spi::Phase::CaptureOnSecondTransition,
            },
        },
    );
    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut serial, "SPI done...\r").unwrap();

    let mut motor_power = pins.a3.into_output();
    motor_power.set_high();
    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut serial, "Motor Power done...\r").unwrap();

    let mut state = State {
        serial,
        serial_buf: [0; 128],
        serial_buf_idx: 0,
        i2c,
        spi,
    };

    // let mut motor_system = MotorSystem::new();
    /*
    motor_system.initialize(
        MotorLocation::FrontLeft,
        pins.d8.into_output().downgrade(),
        &mut state,
    );
    motor_system.initialize(
        MotorLocation::FrontRight,
        pins.d7.into_output().downgrade(),
        &mut state,
    );
    motor_system.initialize(
        MotorLocation::RearLeft,
        pins.d9.into_output().downgrade(),
        &mut state,
    );
    unsafe {
        // We will never remove the pin from this motor and the motor will never turn it into input so this is safe.
        motor_system.initialize(
            MotorLocation::RearLeft,
            d10.into_pin_unchecked().downgrade(),
            &mut state,
        );
    }
    */

    let imu = imu::Imu::new(&mut state);
    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut state.serial, "IMU Done...\r").unwrap();

    let right_tone_detector = tone_detector::ToneDetector::new(pins.d4.downgrade());
    let left_tone_detector = tone_detector::ToneDetector::new(pins.d5.downgrade());
    #[cfg(feature = "logging")]
    state
        .i2c
        .i2cdetect(&mut state.serial, Direction::Write)
        .unwrap_infallible();

    let pressure_sensor = pressure_sensor::PressureSensor::new(&mut state).unwrap();

    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut state.serial, "Starting...\r").unwrap();

    let mut loop_counter = 0;
    loop {
        if loop_counter == 0 {
            led.toggle();
        }
        loop_counter += 1;
        /*
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
                let position = serial::Telemetry::MotorPosition(serial::MotorPosition {
                    location,
                    position: motor.get_position(&mut state),
                });
                serial::write(&mut state, position);
            }
        }
         */
        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading IMU...\r").unwrap();
        if let Ok(imu_measurements) = imu.read(&mut state) {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "IMU Read...\r").unwrap();
            serial::write(
                &mut state,
                serial::Telemetry::Imu(serial::ImuReading {
                    accel_x: imu_measurements.accel_x,
                    accel_y: imu_measurements.accel_y,
                    accel_z: imu_measurements.accel_z,
                    gyro_x: imu_measurements.gyro_x,
                    gyro_y: imu_measurements.gyro_y,
                    gyro_z: imu_measurements.gyro_z,
                }),
            );
        } else {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "Reading IMU failed\r").unwrap();
        }
        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading ToneDetectors...\r").unwrap();
        serial::write(
            &mut state,
            serial::Telemetry::ToneDetector(serial::ToneDetectorStatus {
                location: serial::ToneDetectorLocation::Left,
                is_high: left_tone_detector.read(),
            }),
        );
        serial::write(
            &mut state,
            serial::Telemetry::ToneDetector(serial::ToneDetectorStatus {
                location: serial::ToneDetectorLocation::Right,
                is_high: right_tone_detector.read(),
            }),
        );

        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading PressureSensor...\r").unwrap();
        if let Some(pressure_data) = pressure_sensor.read(&mut state) {
            serial::write(
                &mut state,
                Telemetry::PressureData(PressureValues {
                    pressure: pressure_data.pressure,
                    temperature: pressure_data.temperature,
                }),
            )
        }
        arduino_hal::delay_ms(10);
    }
}
