#![allow(non_camel_case_types)]

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    CHIPID              = 0x00,
    ERR_REG             = 0x02,
    STATUS              = 0x03,
    ACC_DATA_0          = 0x0A,
    ACC_DATA_8          = 0x12,
    SENSORTIME_0        = 0x18,
    INT_STAT_0          = 0x1C,
    INT_STAT_1          = 0x1D,
    STEP_CNT_OUT_0      = 0x1E,
    HIGH_G_OUT          = 0x1F,
    TEMPERATURE         = 0x22,
    FIFO_LENGTH_0       = 0x24,
    FIFO_DATA           = 0x26,
    ACTIVITY_OUT        = 0x27,
    ORIENTATION_OUT     = 0x28,
    INTERNAL_STAT       = 0x2A,
    ACCEL_CONFIG        = 0x40,
    ACCEL_RANGE         = 0x41,
    AUX_CONFIG          = 0x44,
    FIFO_DOWN           = 0x45,
    FIFO_WTM_0          = 0x46,
    FIFO_CONFIG_0       = 0x48,
    FIFO_CONFIG_1       = 0x49,
    AUX_DEV_ID          = 0x4B,
    AUX_IF_CONF         = 0x4C,
    AUX_RD              = 0x4D,
    AUX_WR              = 0x4E,
    AUX_DATA            = 0x4F,
    INT1_IO_CTRL        = 0x53,
    INT2_IO_CTRL        = 0x54,
    INTR_LATCH          = 0x55,
    INT_MAP_1           = 0x56,
    INT_MAP_2           = 0x57,
    INT_MAP_DATA        = 0x58,
    INIT_CTRL           = 0x59,
    FEATURE_CONFIG      = 0x5E,
    IF_CONFIG           = 0x6B,
    ACC_SELF_TEST       = 0x6D,
    NV_CONFIG           = 0x70,
    OFFSET_0            = 0x71,
    OFFSET_1            = 0x72,
    OFFSET_2            = 0x73,
    POWER_CONF          = 0x7C,
    POWER_CTRL          = 0x7D,
    CMD                 = 0x7E,
} 

impl Register {
    pub fn addr(self) -> u8 {
        self as u8
    }

    pub fn read_only(self) -> bool {
        match self {
            // TODO
            _ => false,
        }
    }
}
