use esp_bno055::BNO055OperationMode;
use esp_idf_hal::{
    i2c::{I2cConfig, I2cDriver},
    prelude::*,
    peripherals::Peripherals,
};
use std::thread::sleep;
use std::time::{Duration, SystemTime};
use esp_idf_sys as _;
use esp_idf_svc::sys::link_patches;
use anyhow::Error as AnyError;
use mint::Vector3;


#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperationMode {
    Config = 0x00,
    AccOnly = 0x01,
    MagOnly = 0x02,
    GyroOnly = 0x03,
    AccMag = 0x04,
    AccGyro = 0x05,
    MagGyro = 0x06,
    AMG = 0x07,
    IMU = 0x08,
    Compass = 0x09,
    M4G = 0x0A,
    NdofFmcOff = 0x0B,
    Ndof = 0x0C,
}

fn main() -> anyhow::Result<(), AnyError> {
    link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // ログレベルを警告レベルに設定
    unsafe {
        esp_idf_sys::esp_log_level_set(
            b"uart\0".as_ptr() as *const _,
            esp_idf_sys::esp_log_level_t_ESP_LOG_WARN,
        );
    }

    let peripherals = Peripherals::take()?;
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio5;
    let scl = peripherals.pins.gpio6;

    let i2c_config = I2cConfig::new()
        .baudrate(100.kHz().into())
        .sda_enable_pullup(true)
        .scl_enable_pullup(true);

    let i2c_driver = I2cDriver::new(i2c, sda, scl, &i2c_config)?;

    let mut imu = bno055::Bno055::new(i2c_driver, false)?;
    imu.init()?;
    imu.set_mode(BNO055OperationMode::AMG)?;
    let status = imu.get_calibration_status()?;
    log::info!("Calibration status: {:?}", status);

    let mut acc = Vector3 { x: 0, y: 0, z: 0 };
    let mut gyro = Vector3 { x: 0, y: 0, z: 0 };
    let start = SystemTime::now();
    let mut time = SystemTime::now().duration_since(start)?;
    log::info!("Start time is : {:?}", time);
    loop {
        match imu.accel_data_fixed() {
            Ok(val) => {
                acc = val;
                log::info!("Acceleration is : {:?}", acc);
                time = SystemTime::now().duration_since(start)?;
                log::info!("Time is : {:?}", time);

                sleep(Duration::from_millis(10));
            }
            Err(e) => {
                log::error!("Error: {:?}", e);
            }
        }
        match imu.gyro_data_fixed() {
            Ok(val) => {
                gyro = val;
                log::info!("Gyro is : {:?}", gyro);
                time = SystemTime::now().duration_since(start)?;
                log::info!("Time is : {:?}", time);
                sleep(Duration::from_millis(10));
            }
            Err(e) => {
                log::error!("Error: {:?}", e);
            }
        }
    }
}
