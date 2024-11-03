use anyhow::Error as AnyError;
use esp_idf_hal::{
    i2c::I2cDriver,
    delay::BLOCK,
};
use esp_idf_sys as _; // ESP-IDFバインディングの初期化
use mint::Vector3;
use byteorder::{ByteOrder, LittleEndian};
use std::{
    time::Duration,
    thread::sleep,
    marker::PhantomData,
};

const CHIP_ID: u8 = 0xfb;

const PAGE_0: u8 = 0x00;

pub const CONFIG_MODE: u8 = 0x00;
pub const ACCONLY_MODE: u8 = 0x01;
pub const MAGONLY_MODE: u8 = 0x02;
pub const GYRONLY_MODE: u8 = 0x03;
pub const ACCMAG_MODE: u8 = 0x04;
pub const ACCGYRO_MODE: u8 = 0x05;
pub const MAGGYRO_MODE: u8 = 0x06;
pub const AMG_MODE: u8 = 0x07;
pub const IMUPLUS_MODE: u8 = 0x08;
pub const COMPASS_MODE: u8 = 0x09;
pub const M4G_MODE: u8 = 0x0A;
pub const NDOF_FMC_OFF_MODE: u8 = 0x0B;
pub const NDOF_MODE: u8 = 0x0C;

const ACCEL_2G: u8 = 0x00;
const ACCEL_4G: u8 = 0x01;
const ACCEL_8G: u8 = 0x02;
const ACCEL_16G: u8 = 0x03;
const ACCEL_7_81HZ: u8 = 0x00;
const ACCEL_15_63HZ: u8 = 0x04;
const ACCEL_31_25HZ: u8 = 0x08;
const ACCEL_62_5HZ: u8 = 0x0C;
const ACCEL_125HZ: u8 = 0x10;
const ACCEL_250HZ: u8 = 0x14;
const ACCEL_500HZ: u8 = 0x18;
const ACCEL_1000HZ: u8 = 0x1C;
const ACCEL_NORMAL_MODE: u8 = 0x00;
const ACCEL_SUSPEND_MODE: u8 = 0x20;
const ACCEL_LOWPOWER1_MODE: u8 = 0x40;
const ACCEL_STANDBY_MODE: u8 = 0x60;
const ACCEL_LOWPOWER2_MODE: u8 = 0x80;
const ACCEL_DEEPSUSPEND_MODE: u8 = 0xA0;

const GYRO_2000_DPS: u8 = 0x00;
const GYRO_1000_DPS: u8 = 0x01;
const GYRO_500_DPS: u8 = 0x02;
const GYRO_250_DPS: u8 = 0x03;
const GYRO_125_DPS: u8 = 0x04;
const GYRO_523HZ: u8 = 0x00;
const GYRO_230HZ: u8 = 0x08;
const GYRO_116HZ: u8 = 0x10;
const GYRO_47HZ: u8 = 0x18;
const GYRO_23HZ: u8 = 0x20;
const GYRO_12HZ: u8 = 0x28;
const GYRO_64HZ: u8 = 0x30;
const GYRO_32HZ: u8 = 0x38;
const GYRO_NORMAL_MODE: u8 = 0x00;
const GYRO_FASTPOWERUP_MODE: u8 = 0x01;
const GYRO_DEEPSUSPEND_MODE: u8 = 0x02;
const GYRO_SUSPEND_MODE: u8 = 0x03;
const GYRO_ADVANCEDPOWERSAVE_MODE: u8 = 0x04;

const MAGNET_2HZ: u8 = 0x00;
const MAGNET_6HZ: u8 = 0x01;
const MAGNET_8HZ: u8 = 0x02;
const MAGNET_10HZ: u8 = 0x03;
const MAGNET_15HZ: u8 = 0x04;
const MAGNET_20HZ: u8 = 0x05;
const MAGNET_25HZ: u8 = 0x06;
const MAGNET_30HZ: u8 = 0x07;
const MAGNET_LOWPOWER_MODE: u8 = 0x00;
const MAGNET_REGULAR_MODE: u8 = 0x08;
const MAGNET_ENHANCEDREGULAR_MODE: u8 = 0x10;
const MAGNET_ACCURACY_MODE: u8 = 0x18;
const MAGNET_NORMAL_MODE: u8 = 0x00;
const MAGNET_SLEEP_MODE: u8 = 0x20;
const MAGNET_SUSPEND_MODE: u8 = 0x40;
const MAGNET_FORCEMODE_MODE: u8 = 0x60;

const POWER_NORMAL: u8 = 0x00;
const POWER_NORMAL: u8 = 0x00;
const POWER_LOW: u8 = 0x01;
const POWER_SUSPEND: u8 = 0x02;

const MODE_REGISTER: u8 = 0x3D;
const PAGE_REGISTER: u8 = 0x07;
const POWER_NORMAL: u8 = 0x00;
const POWER_LOW: u8 = 0x01;
const POWER_SUSPEND: u8 = 0x02;

const MODE_REGISTER: u8 = 0x3D;
const PAGE_REGISTER: u8 = 0x07;
const ACCEL_CONFIG_REGISTER: u8 = 0x08;
const POWER_NORMAL: u8 = 0x00;
const POWER_LOW: u8 = 0x01;
const POWER_SUSPEND: u8 = 0x02;

const MODE_REGISTER: u8 = 0x3D;
const PAGE_REGISTER: u8 = 0x07;
const ACCEL_CONFIG_REGISTER: u8 = 0x08;
const MAGNET_CONFIG_REGISTER: u8 = 0x09;
const GYRO_CONFIG_0_REGISTER: u8 = 0x0A;
const MAGNET_CONFIG_REGISTER: u8 = 0x09;
const GYRO_CONFIG_0_REGISTER: u8 = 0x0A;
const ACCEL_CONFIG_REGISTER: u8 = 0x08;
const MAGNET_CONFIG_REGISTER: u8 = 0x09;
const GYRO_CONFIG_0_REGISTER: u8 = 0x0A;
const GYRO_CONFIG_1_REGISTER: u8 = 0x0B;
const CALIBRATION_REGISTER: u8 = 0x35;
const OFFSET_ACCEL_REGISTER: u8 = 0x55;
const OFFSET_MAGNET_REGISTER: u8 = 0x5B;
const OFFSET_GYRO_REGISTER: u8 = 0x61;
const RADIUS_ACCEL_REGISTER: u8 = 0x67;
const RADIUS_MAGNET_REGISTER: u8 = 0x69;
const TRIGGER_REGISTER: u8 = 0x3F;
const POWER_REGISTER: u8 = 0x3E;
const ID_REGISTER: u8 = 0x00;

const AXIS_MAP_CONFIG_REGISTER: u8 = 0x41;
const AXIS_MAP_SIGN_REGISTER: u8 = 0x42;
const AXIS_REMAP_X: u8 = 0x00;
const AXIS_REMAP_Y: u8 = 0x01;
const AXIS_REMAP_Z: u8 = 0x02;
const AXIS_REMAP_POSITIVE: u8 = 0x00;
const AXIS_REMAP_NEGATIVE: u8 = 0x01;

const ACCEL_DATA_LSB: u8 = 0x08;


trait StructRead {
    fn read(&self) -> (f32, f32, f32);
}

struct ScaledReadOnlyStruct<T> {
    register_address: u8,
    scale: f32,
    phantom: PhantomData<T>,
}

impl<T> ScaledReadOnlyStruct<T>
where
    T: StructRead,
{
    fn new(register_address: u8, scale: f32) -> Self {
        ScaledReadOnlyStruct {
            register_address,
            scale,
            phantom: PhantomData,
        }
    }

    pub fn get(&self, device: &T) -> (f32, f32, f32) {
        let result = device.read();
        (
            result.0 * self.scale,
            result.1 * self.scale,
            result.2 * self.scale,
        )
    }
}

struct ReadOnlyStruct {
    register_address: u8,
}

impl ReadOnlyStruct {
    pub fn new(register_address: u8) -> Self {
        ReadOnlyStruct { register_address }
    }
}

pub struct BNO055<'a> {
    i2c: I2cDriver<'a>,
    mode: u8,
    accel_range: u8,
    gyro_range: u8,
    magnet_rate: u8,
    buffer: [u8; 2],
    address: u8,
}

impl<'a> BNO055<'a> {
    pub fn new(i2c: I2cDriver<'a>, address: u8) -> Result<Self, anyhow::Error> {
        let mut bno = BNO055 {
            i2c,
            mode: CONFIG_MODE,
            accel_range: ACCEL_4G,
            gyro_range: GYRO_2000_DPS,
            magnet_rate: MAGNET_20HZ,
            buffer: [0; 2],
            address: address,
        };
        self.set_page(PAGE_0)?;
        let chip_id = match bno.read_register(ID_REGISTER) {
            Ok(id) => {
                log::info!("Successfully read chip ID: {:#x}", id);
                id
            }
            Err(e) => {
                log::info!("Failed to read chip ID, error: {:?}", e);
                return Err(AnyError::from(e))
            }
        };
        if chip_id != CHIP_ID {
            log::info!("Invalid chip ID: {:#x}", chip_id);
            return Err(anyhow::anyhow!("Invalid chip ID"));
        }

        bno.set_mode(CONFIG_MODE)?;
        log::info!("Successfully set mode to CONFIG_MODE");
        bno.reset();
        bno.set_normal_mode();
        bno.write_register(PAGE_REGISTER, 0x00)?;
        bno.write_register(TRIGGER_REGISTER, 0x00)?;
        sleep(Duration::from_millis(10));
        bno.mode = NDOF_MODE;
        sleep(Duration::from_millis(10));
        log::info!("Successfully initialized BNO055");
        Ok(bno)
    }

    pub fn set_page(&mut self, page: u8) -> Result<(), AnyError> {
        let bits = self.mode | 
        self.i2c.write(self.address, &[PAGE_REGISTER, page], 1000).map_err(AnyError::from)?;
    }

    pub fn reset(&mut self) {
        self.mode = CONFIG_MODE;
        self.write_register(TRIGGER_REGISTER, 0x20);
        sleep(Duration::from_millis(80));
    }

    pub fn read_register(&mut self, register: u8) -> Result<u8, AnyError> {
        self.buffer[0] = register;
        self.i2c.write_read(self.address, &[register], &mut self.buffer, 1000).map_err(AnyError::from)?;
        log::info!("Successfully read register: {:?}", self.buffer);
        return Ok(self.buffer[1])
    }

    fn write_register(&mut self, register: u8, value: u8) -> Result<(), AnyError> {
        self.buffer[0] = register;
        self.buffer[1] = value;
        self.i2c.write(self.address, &self.buffer, 1000).map_err(AnyError::from)?;
        Ok(())
    }

    pub fn set_normal_mode(&mut self) {
        self.write_register(POWER_REGISTER, POWER_NORMAL);
        sleep(Duration::from_millis(10));
    }

    pub fn set_suspend_mode(&mut self) {
        self.write_register(POWER_REGISTER, POWER_SUSPEND);
    }

    pub fn get_mode(&mut self) -> Result<u8, AnyError> {
        let mode = self.read_register(MODE_REGISTER)?;
        Ok(mode & 0b00001111)
    }

    pub fn set_mode(&mut self, new_mode: u8) -> Result<(), AnyError> {
        self.write_register(MODE_REGISTER, new_mode)?;
        sleep(Duration::from_millis(30));
        Ok(())
    }

    pub fn get_calibration_status(&mut self) -> Result<(u8, u8, u8, u8), AnyError> {
        let calibration_data = self.read_register(CALIBRATION_REGISTER)?;
        let sys = (calibration_data >> 6) & 0x03;
        let gyro = (calibration_data >> 4) & 0x03;
        let accel = (calibration_data >> 2) & 0x03;
        let mag = calibration_data & 0x03;
        Ok((sys, gyro, accel, mag))
    }

    pub fn is_calibrated(&mut self) -> Result<bool, AnyError> {
        let (sys, gyro, accel, mag) = self.get_calibration_status()?;
        Ok(sys == 0x03 && gyro == 0x03 && accel == 0x03 && mag == 0x03)
    }

    pub fn get_external_crystal(&mut self) -> Result<bool, AnyError> {
        let last_mode = self.get_mode()?;
        self.set_mode(CONFIG_MODE);
        self.write_register(PAGE_REGISTER, 0x00)?;
        let value = self.read_register(TRIGGER_REGISTER)?;
        self.set_mode(last_mode);
        Ok(value == 0x80)
    }

    pub fn set_external_crystal(&mut self, value: bool) -> Result<(), AnyError> {
        let last_mode = self.get_mode()?;
        self.set_mode(CONFIG_MODE);
        self.write_register(PAGE_REGISTER, 0x00)?;
        self.write_register(TRIGGER_REGISTER, if value { 0x80 } else { 0x00 })?;
        self.set_mode(last_mode);
        sleep(Duration::from_millis(10));
        Ok(())
    }

    pub fn scale_vec(&mut self, raw: mint::Vector3<f32>, scaling: f32) -> mint::Vector3<f32> {
        mint::Vector3 {
            x: raw.x * scaling,
            y: raw.y * scaling,
            z: raw.z * scaling,
        }
    }

    pub fn read_vec(&mut self, register: u8) -> Result<mint::Vector3<f32>, AnyError> {
        let mut buffer: [u8; 6] = [0; 6];
        log::info!("Reading vector...");
        self.i2c.write_read(self.address, &[register], &mut buffer, 1000).map_err(|e| {
            log::info!("Error reading vector: {:?}", e);
            AnyError::from(e)
        })?;
        log::info!("Read Vector");

        let x = LittleEndian::read_i32(&buffer[0..2]) as f32;
        let y = LittleEndian::read_i32(&buffer[2..4]) as f32;
        let z = LittleEndian::read_i32(&buffer[4..6]) as f32;
        Ok(Vector3 { x, y, z })
    }

    pub fn read_temperature(&mut self) -> Result<f32, AnyError> {
        let value = self.read_register(0x34)?;
        Ok(value as f32)
    }

    pub fn read_accel(&mut self) -> Result<mint::Vector3<f32>, AnyError> {
        log::info!("Reading accel...");
        let accel = self.read_vec(ACCEL_DATA_LSB)?;
        log::info!("Raw accel: {:?}", accel);
        let scale_accel = self.scale_vec(accel, 1.0/100.0);
        Ok(scale_accel)
    }

    // pub fn get_accel_range(&mut self) -> Result<u8, AnyError> {
    //     self.write_register(PAGE_REGISTER, 0x01)?;
    //     let value = self.read_register(ACCEL_CONFIG_REGISTER)?;
    //     self.write_register(PAGE_REGISTER, 0x00)?;
    //     Ok(value & 0b00000011)
    // }

    // pub fn set_accel_range(&mut self, rng: u8) -> Result<(), AnyError> {
    //     self.write_register(PAGE_REGISTER, 0x01)?;
    //     let value = self.read_register(ACCEL_CONFIG_REGISTER)?;
    //     let masked_value = value & 0b11111100;
    //     self.write_register(ACCEL_CONFIG_REGISTER, masked_value | rng)?;
    //     self.write_register(PAGE_REGISTER, 0x00)?;
    //     Ok(())
    // }
}

