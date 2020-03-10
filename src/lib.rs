#![no_std]
#![allow(non_camel_case_types)]

mod register;

use core::fmt::Debug;
use core::convert::TryInto;
use embedded_hal::blocking::i2c::{WriteRead, Write};
use embedded_hal::blocking::delay::DelayMs;

pub use register::Register;
pub use accelerometer;
use accelerometer::{I16x3, Accelerometer, Tracker};

#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
    /// Invalid input data.
    WrongChip(u8),
    WriteToReadOnly,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DataRate {
    Hz_0_78 = 0x01,
    Hz_1_56 = 0x02,
    Hz_3_12 = 0x03,
    Hz_6_25 = 0x04,
    Hz_12_5 = 0x05,
    Hz_25   = 0x06,
    Hz_50   = 0x07,
    Hz_100  = 0x08,
    Hz_200  = 0x09,
    Hz_400  = 0x0A,
    Hz_800  = 0x0B,
    Hz_1600 = 0x0C,
}

impl DataRate {
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Range {
    G2 = 0x00,
    G4 = 0x01,
    G8 = 0x02,
    G16 = 0x03,
}

impl Range {
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Bandwidth {
    OSR4_AVG1   = 0x00,
    OSR2_AVG2   = 0x01,
    NORMAL_AVG4 = 0x02,
    CIC_AVG8    = 0x03,
    RES_AVG16   = 0x04,
    RES_AVG32   = 0x05,
    RES_AVG64   = 0x06,
    RES_AVG128  = 0x07,
}

impl Bandwidth {
    pub fn bits(self) -> u8 {
        self as u8
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum PerfMode {
    CIC_AVG    = 0x00,
    CONTINUOUS = 0x01,
}

impl PerfMode {
    pub fn bits(self) -> u8 {
        self as u8
    }
}

pub struct BMA421<I2C> {
    i2c: I2C,
}

pub const I2C_ADDR: u8 = 0x18;
pub const DEVICE_ID: u8 = 0x11;

impl<I2C, E> BMA421 <I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>
{
    pub fn new(i2c: I2C, delay: &mut DelayMs<u8>) -> Result<Self, Error<E>> {
        // TODO occasionally getting i2c transmit errors in here, needs debugging
        let mut bma421 = BMA421 { i2c };
        let id = bma421.read_register(Register::CHIPID)?;
        if id != DEVICE_ID {
            return Err(Error::WrongChip(id))
        } else {
            bma421.soft_reset()?;
            delay.delay_ms(200);
            bma421.set_accel_enable(true)?;
            delay.delay_ms(100);
          
            bma421.set_datarate(DataRate::Hz_100)?;
            bma421.set_range(Range::G2)?;
            bma421.set_bandwidth(Bandwidth::NORMAL_AVG4)?;
            bma421.set_perf_mode(PerfMode::CONTINUOUS)?;
            delay.delay_ms(100);

            Ok(bma421)
        }
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

    pub fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<E>> {
        let mut config_data = self.read_register(Register::ACCEL_CONFIG)?;
        config_data &= !0x0f;
        config_data |= datarate.bits();
        self.write_register(Register::ACCEL_CONFIG, config_data)?;
        Ok(())
    }

    pub fn set_bandwidth(&mut self, bandwidth: Bandwidth) -> Result<(), Error<E>> {
        let mut config_data = self.read_register(Register::ACCEL_CONFIG)?;
        config_data |=  bandwidth.bits() << 4 ;
        self.write_register(Register::ACCEL_CONFIG, config_data)?;
        Ok(())
    }

    pub fn set_perf_mode(&mut self, perf_mode: PerfMode) -> Result<(), Error<E>> {
        let mut config_data = self.read_register(Register::ACCEL_CONFIG)?;
        config_data |= perf_mode.bits() << 7;
        self.write_register(Register::ACCEL_CONFIG, config_data)?;
        Ok(())
    }

    pub fn set_range(&mut self, range: Range) -> Result<(), Error<E>> {
        let mut config_data = self.read_register(Register::ACCEL_CONFIG)?;
        config_data |= range.bits() & 0x03;
        self.write_register(Register::ACCEL_RANGE, config_data)?;
        Ok(())
    }

    pub fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut data = [0];
        self.i2c
            .write_read(I2C_ADDR, &[register.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }

    fn read_accel_bytes(&mut self) -> Result<[u8;6], Error<E>> {
        let mut data = [0u8;6];
        self.i2c
            .write_read(I2C_ADDR, &[Register::ACC_DATA_8.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data))
    }

    pub fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>>
    {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }
        self.i2c.write(I2C_ADDR, &[register.addr(), value]).map_err(Error::I2C)
    }

    //pub fn try_into_tracker(mut self) -> Result<Tracker<Self, I16x3>, Error<E>> 
    //where 
        //E: Debug
    //{
        //self.set_range(Range::G8)?;
        //Ok(Tracker::new(self, 3700))
    //}
}

impl<I2C, E> Accelerometer<I16x3> for BMA421<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    /// Get acceleration reading from the accelerometer
    fn acceleration(&mut self) -> Result<I16x3, Error<E>> {
       let accel_bytes = self.read_accel_bytes()?;
       let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
       let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
       let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());
       Ok(I16x3::new(x, y, z))
    }
}
