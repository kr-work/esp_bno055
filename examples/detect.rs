use esp_idf_hal::{
    delay::Delay,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    prelude::*,
};
use esp_idf_svc::sys::link_patches;

fn detect(i2c: &mut I2cDriver) -> Vec<u8> {
    let mut buffer = [0u8; 1];
    let timeout = 1;
    let delay = Delay::default();
    let mut found_devices = Vec::new();

    // log::info!("Scanning I2C bus...");

    for addr in 0..128 {
        match i2c.read(addr, &mut buffer, timeout) {
            Ok(_) => {
                // log::info!("Found device at address: 0x{:02X}", addr);
                found_devices.push(addr);
            }
            Err(_) => {
                // log::info!("No device at address: 0x{:02X}", addr);
            }
        }
        delay.delay_ms(10u32);
    }

    // log::info!("I2C scan complete");
    log::info!(
        "Found devices: {:?}",
        found_devices
            .iter()
            .map(|addr| format!("0x{:02X}", addr))
            .collect::<Vec<String>>()
    );

    found_devices
}

fn main() -> anyhow::Result<()> {
    link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // ログレベルを警告レベルに設定

    let peripherals = Peripherals::take()?;
    // ピン設定
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio5;
    let scl = peripherals.pins.gpio6;

    let i2c_config = I2cConfig::new()
        .baudrate(100.kHz().into())
        .sda_enable_pullup(true)
        .scl_enable_pullup(true);

    let mut i2c_driver = I2cDriver::new(i2c, sda, scl, &i2c_config)?;
    let device = detect(&mut i2c_driver);
    Ok(())
}
