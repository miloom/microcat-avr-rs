#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(clippy::allow_attributes_without_reason)]

extern crate core;

mod imu;
mod millis;
mod motors;
mod pressure_sensor;
mod serial;
mod timer;
mod tone_detector;

use atmega_hal::adc::AdcSettings;
use atmega_hal::clock;
#[cfg(feature = "logging")]
use atmega_hal::i2c::Direction;
use atmega_hal::pac::USART0;
use atmega_hal::port::mode::{Input, Output};
use atmega_hal::port::{Pin, PD0, PD1};
#[cfg(feature = "logging")]
use atmega_hal::prelude::_unwrap_infallible_UnwrapInfallible as _;
use atmega_hal::spi;
use atmega_hal::usart::Baudrate;
use atmega_hal::{adc, adc::channel};
use atmega_hal::{I2c, Spi, Usart};
use avr_device::atmega328p::TC1;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use embedded_hal::spi::{Mode, Phase, Polarity};
use panic_halt as _;
use serial::{read_serial, Command, PressureValues, Telemetry};
use strum::IntoEnumIterator as _;

use crate::timer::rig_timer;
use motors::{MotorLocation, MotorSystem};

static LOOP_INTERRUPT: AtomicBool = AtomicBool::new(false);

type Adc = adc::Adc<CoreClock>;
type CoreClock = clock::MHz16;
struct State {
    i2c: I2c<CoreClock>,
    serial: Usart<USART0, Pin<Input, PD0>, Pin<Output, PD1>, CoreClock>,
    serial_buf: [u8; 128],
    serial_buf_idx: usize,
    spi: Spi,
}

#[avr_device::entry]
#[expect(
    clippy::too_many_lines,
    reason = "Main function is allowed to be large"
)]
#[expect(
    clippy::single_call_fn,
    reason = "Main will always be called only once"
)]
fn main() -> ! {
    const AVCC: u32 = 5000;

    #[expect(
        clippy::unwrap_used,
        reason = "We require peripherals to be able to run any logic"
    )]
    let dp = atmega_hal::Peripherals::take().unwrap();
    let pins = atmega_hal::pins!(dp);
    let mut led = pins.pc0.into_output();

    #[cfg_attr(
        not(feature = "logging"),
        expect(unused_mut, reason = "This mut is only needed with logging macro")
    )]
    let mut serial = Usart::new(
        dp.USART0,
        pins.pd0,
        pins.pd1.into_output(),
        Baudrate::<CoreClock>::new(115_200),
    );
    led.set_high();

    millis::init(&dp.TC0);
    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut serial, "Millis done...\r").unwrap_infallible();

    let tmr1: TC1 = dp.TC1;
    rig_timer(&tmr1);

    // Enable interrupts globally
    // SAFETY: Enabling interrupts is unsafe
    unsafe {
        use avr_device::interrupt;
        interrupt::enable();
    };

    let i2c = I2c::new(
        dp.TWI,
        pins.pc4.into_pull_up_input(),
        pins.pc5.into_pull_up_input(),
        100_000,
    );
    let (spi, d10) = Spi::new(
        dp.SPI,
        pins.pb5.into_output(),
        pins.pb3.into_output(),
        pins.pb4.into_pull_up_input(),
        pins.pb2.into_output(),
        spi::Settings {
            data_order: spi::DataOrder::MostSignificantFirst,
            clock: spi::SerialClockRate::OscfOver16,
            mode: Mode {
                polarity: Polarity::IdleHigh,
                phase: Phase::CaptureOnSecondTransition,
            },
        },
    );
    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut serial, "SPI done...\r").unwrap_infallible();

    let mut motor_power = pins.pc3.into_output();
    motor_power.set_high();
    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut serial, "Motor Power done...\r").unwrap_infallible();

    let mut state = State {
        serial,
        serial_buf: [0; 128],
        serial_buf_idx: 0,
        i2c,
        spi,
    };
    let mut motor_system = MotorSystem::new();
    motor_system.initialize(
        MotorLocation::FrontLeft,
        pins.pb0.into_output().downgrade(),
        &mut state,
    );
    motor_system.initialize(
        MotorLocation::FrontRight,
        pins.pd7.into_output().downgrade(),
        &mut state,
    );
    motor_system.initialize(
        MotorLocation::RearLeft,
        pins.pb1.into_output().downgrade(),
        &mut state,
    );
    // SAFETY:  We will never remove the pin from this motor and the motor will never turn it into input so this is safe.
    unsafe {
        motor_system.initialize(
            MotorLocation::RearLeft,
            d10.into_pin_unchecked().downgrade(),
            &mut state,
        );
    };

    let imu = imu::Imu::new(&mut state);
    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut state.serial, "IMU Done...\r").unwrap_infallible();

    let right_tone_detector = tone_detector::ToneDetector::new(pins.pd4.downgrade());
    let left_tone_detector = tone_detector::ToneDetector::new(pins.pd5.downgrade());
    #[cfg(feature = "logging")]
    state
        .i2c
        .i2cdetect(&mut state.serial, Direction::Write)
        .unwrap_infallible();

    let pressure_sensor = pressure_sensor::PressureSensor::new(&mut state).ok();

    let mut adc = Adc::new(dp.ADC, AdcSettings::default());

    #[cfg(feature = "logging")]
    ufmt::uwriteln!(&mut state.serial, "Starting...\r").unwrap_infallible();

    let mut loop_counter = 0u32;
    loop {
        while !LOOP_INTERRUPT.load(Ordering::SeqCst) {
            use core::hint;
            hint::spin_loop();
        }
        LOOP_INTERRUPT.store(false, Ordering::SeqCst);
        if loop_counter == 0 {
            led.toggle();
        }
        loop_counter = loop_counter.wrapping_add(1);
        loop_counter %= 100;

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

        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading IMU...\r").unwrap_infallible();
        if let Ok(imu_measurements) = imu.read(&mut state) {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "IMU Read...\r").unwrap_infallible();
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
            ufmt::uwriteln!(&mut state.serial, "Reading IMU failed\r").unwrap_infallible();
        }
        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading ToneDetectors...\r").unwrap_infallible();
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
        ufmt::uwriteln!(&mut state.serial, "Reading PressureSensor...\r").unwrap_infallible();
        if let Some(pressure_sensor) = &pressure_sensor {
            if let Some(pressure_data) = pressure_sensor.read(&mut state) {
                serial::write(
                    &mut state,
                    Telemetry::PressureData(PressureValues {
                        pressure: pressure_data.pressure,
                        temperature: pressure_data.temperature,
                    }),
                );
            }
        }

        {
            let value = adc.read_blocking(&channel::ADC6);
            if let Some(temp) = u32::from(value).checked_mul(AVCC) {
                if let Some(battery_mv) = temp.checked_div(1023) {
                    serial::write(&mut state, Telemetry::BatteryVoltage(battery_mv));
                } else {
                    #[cfg(feature = "logging")]
                    ufmt::uwriteln!(&mut state.serial, "Failed division to mV\r")
                        .unwrap_infallible();
                }
            } else {
                #[cfg(feature = "logging")]
                ufmt::uwriteln!(&mut state.serial, "Failed multiplication with VCCC\r")
                    .unwrap_infallible();
            }
        }
    }
}

#[avr_device::interrupt(atmega328p)]
#[expect(
    clippy::single_call_fn,
    reason = "We need a separate function for an interrupt"
)]
fn TIMER1_COMPA() {
    LOOP_INTERRUPT.store(true, Ordering::SeqCst);
}
