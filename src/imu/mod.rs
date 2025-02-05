use crate::State;
use arduino_hal::i2c::Error;
use core::marker::PhantomData;
use icm20608g::structs::*;

pub struct ImuMeasurements {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16, // x_angular_rate = gyro_x / 131 LSB(º/s)
    pub gyro_y: i16, // y_angular_rate = gyro_y / 131 LSB(º/s)
    pub gyro_z: i16, // z_angular_rate = gyro_z / 131 LSB(º/s)
}

pub struct Imu {
    _phantom: PhantomData<()>,
}

impl Imu {
    pub fn new(state: &mut State) -> Imu {
        if let Ok(mut pwr_mgmt) = PowerManagement1::new(&mut state.i2c) {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "IMU read power management\r").unwrap();
            // Bring IMU out of sleep
            pwr_mgmt.sleep = false;
            if pwr_mgmt.write(&mut state.i2c).is_ok() {
                #[cfg(feature = "logging")]
                ufmt::uwriteln!(&mut state.serial, "IMU brought out of sleep\r").unwrap();
            }
        }

        if let Ok(mut cfg) = Config::new(&mut state.i2c) {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "IMU read gyroscope and temp config\r").unwrap();
            // Set the gyroscope and temp sensor to slowest mode highest res
            cfg.dlpf_cfg = 6;
            if cfg.write(&mut state.i2c).is_ok() {
                #[cfg(feature = "logging")]
                ufmt::uwriteln!(&mut state.serial, "IMU wrote gyroscope and temp config\r")
                    .unwrap();
            }
        }

        if let Ok(mut gyro_config) = GyroConfig::new(&mut state.i2c) {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "IMU read gyroscope settings\r").unwrap();
            // Set the gyroscope range to 250dps
            gyro_config.full_scale_select = 0;
            // Make sure to use the gyro rate that is set above and not override it.
            gyro_config.fchoice_b = 0b0;
            if gyro_config.write(&mut state.i2c).is_ok() {
                #[cfg(feature = "logging")]
                ufmt::uwriteln!(&mut state.serial, "IMU wrote gyroscope settings\r").unwrap();
            }
        }

        if let Ok(mut accel_config) = AccelConfig1::new(&mut state.i2c) {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "IMU read accel range\r").unwrap();
            // Set the accelerometer range to (+-4g)
            accel_config.full_scale_select = 0b01;
            if accel_config.write(&mut state.i2c).is_ok() {
                #[cfg(feature = "logging")]
                ufmt::uwriteln!(&mut state.serial, "IMU wrote accel range\r").unwrap();
            }
        }

        if let Ok(mut accel_config) = AccelConfig2::new(&mut state.i2c) {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "IMU read accel settings\r").unwrap();
            // Set accelerometer to average 32 samples
            accel_config.dec2_cfg = 0b11;
            // This and next line set low speed measurements and low pass filter to filter out most noise
            accel_config.accel_fchoice_b = false;
            accel_config.dlpf_cfg = 6;
            if accel_config.write(&mut state.i2c).is_ok() {
                #[cfg(feature = "logging")]
                ufmt::uwriteln!(&mut state.serial, "IMU wrote accel settings\r").unwrap();
            }
        }

        if let Ok(mut gyro_offset) = GyroOffset::new(&mut state.i2c) {
            #[cfg(feature = "logging")]
            ufmt::uwriteln!(&mut state.serial, "IMU read gyroscope offsets\r").unwrap();
            gyro_offset.xg_offs = 0;
            gyro_offset.yg_offs = 0;
            gyro_offset.zg_offs = 0;
            if gyro_offset.write(&mut state.i2c).is_ok() {
                #[cfg(feature = "logging")]
                ufmt::uwriteln!(&mut state.serial, "IMU wrote gyroscope offset\r").unwrap();
            }
        }
        Imu {
            _phantom: Default::default(),
        }
    }

    pub fn read(&self, state: &mut State) -> Result<ImuMeasurements, Error> {
        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading accel data \r").unwrap();

        let accel_measurements = AccelMeasurements::new(&mut state.i2c)?;
        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Reading gyro data \r").unwrap();
        let gyro_measurements = GyroscopeMeasurements::new(&mut state.i2c)?;
        #[cfg(feature = "logging")]
        ufmt::uwriteln!(&mut state.serial, "Done IMU\r").unwrap();
        Ok(ImuMeasurements {
            accel_x: accel_measurements.x,
            accel_y: accel_measurements.y,
            accel_z: accel_measurements.z,
            gyro_x: gyro_measurements.x,
            gyro_y: gyro_measurements.y,
            gyro_z: gyro_measurements.z,
        })
    }
}
