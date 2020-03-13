#![allow(non_camel_case_types)]

use core::convert::TryFrom;
use accelerometer::error::{Error, ErrorKind};

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

impl TryFrom<u8> for DataRate
{
    type Error = Error<ErrorKind>;
    fn try_from(value: u8) -> Result<Self, Error<ErrorKind>> {
        match value {
            0x01 => Ok(DataRate::Hz_0_78),
            0x02 => Ok(DataRate::Hz_1_56),
            0x03 => Ok(DataRate::Hz_3_12),
            0x04 => Ok(DataRate::Hz_6_25),
            0x05 => Ok(DataRate::Hz_12_5),
            0x06 => Ok(DataRate::Hz_25),
            0x07 => Ok(DataRate::Hz_50),
            0x08 => Ok(DataRate::Hz_100),
            0x09 => Ok(DataRate::Hz_200),
            0x0A => Ok(DataRate::Hz_400),
            0x0B => Ok(DataRate::Hz_800),
            0x0C => Ok(DataRate::Hz_1600),
            _ => Err(Error::new(ErrorKind::Param)),
        }
    }
}

impl Into<f32> for DataRate {
    fn into(self) -> f32 {
        match self {
            DataRate::Hz_0_78 => 0.78,
            DataRate::Hz_1_56 => 1.56,
            DataRate::Hz_3_12 => 3.12,
            DataRate::Hz_6_25 => 6.25,
            DataRate::Hz_12_5 => 12.5,
            DataRate::Hz_25   => 25.0,
            DataRate::Hz_50   => 50.0,
            DataRate::Hz_100  => 100.0,
            DataRate::Hz_200  => 200.0,
            DataRate::Hz_400  => 400.0,
            DataRate::Hz_800  => 800.0,
            DataRate::Hz_1600  => 1600.0,
        }
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

impl TryFrom<u8> for Range
{
    type Error = Error<ErrorKind>;
    fn try_from(value: u8) -> Result<Self, Error<ErrorKind>> {
        match value {
            0x00 => Ok(Range::G2),
            0x01 => Ok(Range::G4),
            0x02 => Ok(Range::G8),
            0x03 => Ok(Range::G16),
            _ => Err(Error::new(ErrorKind::Param)),
        }
    }
}

impl Into<f32> for Range {
    fn into(self) -> f32 {
        match self {
            Range::G2 => 2.0,
            Range::G4 => 4.0,
            Range::G8 => 8.0,
            Range::G16 => 16.0,
        }
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

impl TryFrom<u8> for Bandwidth
{
    type Error = Error<ErrorKind>;
    fn try_from(value: u8) -> Result<Self, Error<ErrorKind>> {
        match value {
            0x00 => Ok(Bandwidth::OSR4_AVG1),
            0x01 => Ok(Bandwidth::OSR2_AVG2),
            0x02 => Ok(Bandwidth::NORMAL_AVG4),
            0x03 => Ok(Bandwidth::CIC_AVG8),
            0x04 => Ok(Bandwidth::RES_AVG16),
            0x05 => Ok(Bandwidth::RES_AVG32),
            0x06 => Ok(Bandwidth::RES_AVG64),
            0x07 => Ok(Bandwidth::RES_AVG128),
            _ => Err(Error::new(ErrorKind::Param)),
        }
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

impl TryFrom<u8> for PerfMode
{
    type Error = Error<ErrorKind>;
    fn try_from(value: u8) -> Result<Self, Error<ErrorKind>> {
        match value {
            0x00 => Ok(PerfMode::CIC_AVG),
            0x01 => Ok(PerfMode::CONTINUOUS),
            _ => Err(Error::new(ErrorKind::Param)),
        }
    }
}


