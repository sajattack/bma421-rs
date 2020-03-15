#![no_std]

mod register;
mod types;
mod config;

use core::fmt::Debug;
use core::convert::TryInto;
use core::convert::TryFrom;
use embedded_hal::blocking::i2c::{WriteRead, Write};
use embedded_hal::blocking::delay::DelayMs;

pub use register::Register;
pub use accelerometer;
use accelerometer::{{vector::{I16x3, F32x3}}, Accelerometer, RawAccelerometer};

pub use accelerometer::error::{Error, ErrorKind};

pub use types::*;
use config::{CONFIG_FILE, REMAINDER};

pub struct BMA421<I2C> {
    i2c: I2C,
}

pub const I2C_ADDR: u8 = 0x18;
pub const DEVICE_ID: u8 = 0x11;
pub const RESOLUTION: u8 = 12;

impl<I2C, E> BMA421 <I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug
{
    pub fn new(i2c: I2C, delay: &mut DelayMs<u8>) -> Result<Self, Error<E>> {
        // TODO occasionally getting i2c transmit errors in here, needs debugging
        let mut bma421 = BMA421 { i2c };
        let id = bma421.read_register(Register::CHIPID)?;
        if id != DEVICE_ID {
            return Err(Error::new(ErrorKind::Device))
        } else {
            bma421.soft_reset()?;
            delay.delay_ms(50);

            bma421.write_config_file(delay)?;
            delay.delay_ms(20);
          
            bma421.set_datarate(DataRate::Hz_50)?;
            bma421.set_range(Range::G4)?;
            bma421.set_bandwidth(Bandwidth::NORMAL_AVG4)?;
            bma421.set_perf_mode(PerfMode::CONTINUOUS)?;
            bma421.set_accel_enable(true)?;
            delay.delay_ms(40);

            Ok(bma421)
        }
    }

    fn write_config_file(&mut self, delay: &mut DelayMs<u8>) -> Result<(), Error<E>> {
        let mut power_conf = self.read_register(Register::POWER_CONF)?;
        power_conf &= !(1 << 0x01);
        self.write_register(Register::POWER_CONF, power_conf)?;
        delay.delay_ms(1);
        self.write_register(Register::INIT_CTRL, 0x00)?;
        let mut buf = [0u8;254];
        for i in 0..CONFIG_FILE.len() / 254 {
            buf.copy_from_slice(&config::CONFIG_FILE[i*254..(i+1)*254]);
            self.stream_transfer_write(&buf, (i*254) as u16)?;
        }
        let mut buf = [0u8; REMAINDER];
        buf.copy_from_slice(
            &CONFIG_FILE[(CONFIG_FILE.len() - REMAINDER)..CONFIG_FILE.len()]
        );
        self.stream_transfer_write(&buf, (CONFIG_FILE.len() - REMAINDER) as u16)?;
        self.write_register(Register::INIT_CTRL, 0x01)?;
        delay.delay_ms(150);
        let config_stream_status = self.read_register(Register::INTERNAL_STAT)?; 
        //if config_stream_status != 0x01 {
        //    Err(Error::new(ErrorKind::Device))
        //} else {
            let mut power_conf = self.read_register(Register::POWER_CONF)?;
            power_conf &= !(0 << 0x01);
            self.write_register(Register::POWER_CONF, power_conf)?;
            Ok(())
        //}
    }

    fn stream_transfer_write(
        &mut self,
        stream_data: &[u8],
        index: u16
    ) -> Result<(), Error<E>> {
        debug_assert!(index % 2 == 0, "index is not even");
        let asic_msb = ((index/2) >> 4) as u8;
        let asic_lsb = ((index/2) & 0x0f) as u8;
        self.write_register(Register::ASIC_LSB, asic_lsb)?;
        self.write_register(Register::ASIC_MSB, asic_msb)?;
        let mut buf = [0u8;255];
        //TODO fix dirty hack
        if stream_data.len() == 254 {
            buf[0] = Register::FEATURE_CONFIG.addr();
            buf[1..].copy_from_slice(stream_data);
        } else {
            let mut buf = [0u8;REMAINDER+1];
            buf[0] = Register::FEATURE_CONFIG.addr();
            buf[1..].copy_from_slice(stream_data);
        }
        self.i2c.write(I2C_ADDR, &buf)?;

        Ok(())
    }

    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.write_register(Register::CMD, 0xB6)?;
        Ok(())
    }

    pub fn set_accel_enable(&mut self, enable: bool) -> Result<(), Error<E>> {
        let mut power_ctrl_reg_data = self.read_register(Register::POWER_CTRL)?;
        power_ctrl_reg_data = 
            (power_ctrl_reg_data & !0x04) |
            (
                (if enable {1} else {0}) << 2
            );
        self.write_register(Register::POWER_CTRL, power_ctrl_reg_data)?;
        Ok(())
    }

    pub fn get_accel_enable(&mut self) -> Result<bool, Error<E>> {
        let power_ctrl_reg_data = self.read_register(Register::POWER_CTRL)?;
        let enabled = if (power_ctrl_reg_data & 0x04) >> 2 == 1 { true } else { false };
        Ok(enabled)
    }

    pub fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<E>> {
        let mut config_data = self.read_register(Register::ACCEL_CONFIG)?;
        config_data &= !0x0f;
        config_data |= datarate.bits();
        self.write_register(Register::ACCEL_CONFIG, config_data)?;
        Ok(())
    }
    
    pub fn get_datarate(&mut self) -> Result<DataRate, Error<E>> {
        let config_data = self.read_register(Register::ACCEL_CONFIG)?;
        let datarate = DataRate::try_from(config_data & 0x0f).map_err(|_| {
            Error::new(ErrorKind::Device)
        })?;
        Ok(datarate)
    }

    pub fn set_bandwidth(&mut self, bandwidth: Bandwidth) -> Result<(), Error<E>> {
        let mut config_data = self.read_register(Register::ACCEL_CONFIG)?;
        config_data |=  bandwidth.bits() << 4 ;
        self.write_register(Register::ACCEL_CONFIG, config_data)?;
        Ok(())
    }

    pub fn get_bandwidth(&mut self) -> Result<Bandwidth, Error<E>> {
        let config_data = self.read_register(Register::ACCEL_CONFIG)?;
        let bandwidth = Bandwidth::try_from((config_data & 0x70) >> 4).map_err(|_| {
            Error::new(ErrorKind::Device)
        })?;
        Ok(bandwidth)
    }

    pub fn set_perf_mode(&mut self, perf_mode: PerfMode) -> Result<(), Error<E>> {
        let mut config_data = self.read_register(Register::ACCEL_CONFIG)?;
        config_data |= perf_mode.bits() << 7;
        self.write_register(Register::ACCEL_CONFIG, config_data)?;
        Ok(())
    }

    pub fn get_perf_mode(&mut self) -> Result<PerfMode, Error<E>> {
        let config_data = self.read_register(Register::ACCEL_CONFIG)?;
        let perf_mode = PerfMode::try_from((config_data & 0x80) >> 7).map_err(|_| {
            Error::new(ErrorKind::Device)
        })?;
        Ok(perf_mode)
    }

    pub fn set_range(&mut self, range: Range) -> Result<(), Error<E>> {
        let mut config_data = self.read_register(Register::ACCEL_CONFIG)?;
        config_data |= range.bits() & 0x03;
        self.write_register(Register::ACCEL_RANGE, config_data)?;
        Ok(())
    }

    pub fn get_range(&mut self) -> Result<Range, Error<E>> {
        let config_data = self.read_register(Register::ACCEL_RANGE)?;
        let range = Range::try_from(config_data & 0x03).map_err(|_| {
            Error::new(ErrorKind::Device)
        })?;
        Ok(range)
    }

    pub fn set_step_counter_enable(&mut self, enable: bool) -> Result<(), Error<E>> {
        let mut feature_config = self.read_register(Register::FEATURE_CONFIG)?;
        feature_config = 
            (feature_config & !0x10) |
            (
                (if enable {1} else {0}) << 4
            );
        self.write_register(Register::FEATURE_CONFIG, feature_config)?;
        Ok(())
    }

    pub fn reset_step_count(&mut self) -> Result<(), Error<E>> {
        let mut feature_config = self.read_register(Register::FEATURE_CONFIG)?;
        feature_config = (feature_config & !0x04) | (1 << 2);
        self.write_register(Register::FEATURE_CONFIG, feature_config)?;
        Ok(())
    }

    pub fn get_step_count(&mut self) -> Result<u32, Error<E>> {
        let mut buf = [0u8;4];
        self.i2c
            .write_read(I2C_ADDR, &[Register::STEP_CNT_OUT_0.addr()], &mut buf)?;
            Ok(u32::from_le_bytes(buf))
    }

    pub fn read_register(&mut self, register: Register) -> Result<u8, E> {
        let mut data = [0];
        self.i2c
            .write_read(I2C_ADDR, &[register.addr()], &mut data)
            .and(Ok(data[0]))
    }

    fn read_accel_bytes(&mut self) -> Result<[u8;6], E> {
        let mut data = [0u8;6];
        self.i2c
            .write_read(I2C_ADDR, &[Register::ACC_DATA_8.addr()], &mut data)
            .and(Ok(data))
    }

    pub fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>>
    {
        if register.read_only() {
            return Err(Error::new(ErrorKind::Param))
        }
        self.i2c.write(I2C_ADDR, &[register.addr(), value]).map_err(|_| Error::new(ErrorKind::Bus))
    }
}

impl<I2C, E> RawAccelerometer<I16x3> for BMA421<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = E;
    fn accel_raw(&mut self) -> Result<I16x3, Error<E>> {
       let accel_bytes = self.read_accel_bytes()?;
       let x: i16 = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap()) / 16;
       let y: i16 = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap()) / 16;
       let z: i16 = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap()) / 16;
       Ok(I16x3::new(x, y, z))
    }
}

impl<I2C, E> Accelerometer for BMA421<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = E;

    /// Get normalized ±g reading from the accelerometer.
    fn accel_norm(&mut self) -> Result<F32x3, Error<E>> {
        let raw_data: I16x3 = self.accel_raw()?;
        let range: f32 = self.get_range()?.into();

        let x = (raw_data.x as f32 / ((2 << RESOLUTION-2)) as f32) * range;
        let y = (raw_data.y as f32 / ((2 << RESOLUTION-2)) as f32) * range;
        let z = (raw_data.z as f32 / ((2 << RESOLUTION-2)) as f32) * range;

        Ok(F32x3::new(x, y, z))
    }

    fn sample_rate(&mut self) -> Result<f32, Error<Self::Error>> {
        Ok(self.get_datarate()?.into())
    }
}

