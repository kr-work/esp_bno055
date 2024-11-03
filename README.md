## BNO055
[Here](https://crates.io/crates/bno055/0.1.0) is the crate we used as a reference in creating this library. This crate was created because the referenced program outputs an error in the crate dependency when ESP is used.

The microcontroller used in this project is the ESP32 S3.
Originally, ESP32 was used, but it could not recognize the sensor, so we changed it (probably due to clock stretching).

## Usage
This crate uses [esp-idf-hal](https://crates.io/crates/esp-idf-hal/0.44.1) to communicate via I2C.

```rust
let peripherals = Peripherals::take()?;
let i2c = peripherals.i2c0;
let sda = peripherals.pins.gpio5;
let scl = peripherals.pins.gpio6;
```
The first step is to set the sda and scl pins.

```rust
let i2c_config = I2cConfig::new()
    .baudrate(100.kHz().into())
    .sda_enable_pullup(true)
    .scl_enable_pullup(true);

let i2c_driver = I2cDriver::new(i2c, sda, scl, &i2c_config)?;

let mut imu = esp_bno055::Bno055::new(i2c_driver, false)?;
```
After setting the necessary settings for I2C communication, such as baudrate and pull-up resistor, in the variable i2c_config, a driver is generated using the pins and the above settings. call with this driver and the bool value as arguments.

This IMU defaults to 0x28, with an alternate address of 0x29. 0x28 can be used by selecting true as the bool value, and 0x29 by selecting false.


If you feel that I2C communication is not possible even with crates, please use examples/detect.rs. You can check the connection.

## Attention
When using I2C to connect multiple sensors, I have to use sda and scl to set up multiple drivers, but there should be an error due to Rust ownership issues. In this case, we were able to use Arc,Mutex to handle the problem.In that case, we will publish it as a new crate in due course.
