use crate::{Error, LPS28DFW};
use embedded_hal::i2c::I2c;

pub struct Register;
impl Register {
    pub const INTERRUPT_CFG: u8 = 0x0B;
    pub const THRESHOLD_PRESSURE_LOW: u8 = 0x0C;
    pub const THRESHOLD_PRESSURE_HIGH: u8 = 0x0D;
    pub const INTERFACE_CONTROL: u8 = 0x0E;
    pub const WHO_AM_I: u8 = 0x0F;
    pub const CONTROL_1: u8 = 0x10;
    pub const CONTROL_2: u8 = 0x11;
    pub const CONTROL_3: u8 = 0x12;
    pub const CONTROL_4: u8 = 0x13;
    pub const FIFO_CONTROL: u8 = 0x14;
    pub const FIFO_WATERMARK: u8 = 0x15;
    pub const REFERENCE_PRESSURE_LOW: u8 = 0x16;
    pub const REFERENCE_PRESSURE_HIGH: u8 = 0x17;
    pub const I3C_INTERFACE_CONTROL: u8 = 0x19;
    pub const PRESSURE_OFFSET_LOW: u8 = 0x1A;
    pub const PRESSURE_OFFSET_HIGH: u8 = 0x1B;
    pub const INTERRUPT_SOURCE: u8 = 0x24;
    pub const FIFO_STATUS1: u8 = 0x25;
    pub const FIFO_STATUS2: u8 = 0x26;
    pub const STATUS: u8 = 0x27;
    pub const PRESSURE_OUT_XL: u8 = 0x28;
    pub const PRESSURE_OUT_L: u8 = 0x29;
    pub const PRESSURE_OUT_H: u8 = 0x2A;
    pub const TEMPERATURE_OUT_L: u8 = 0x2B;
    pub const TEMPERATURE_OUT_H: u8 = 0x2C;
    pub const FIFO_DATA_OUT_PRESSURE_XL: u8 = 0x78;
    pub const FIFO_DATA_OUT_PRESSURE_L: u8 = 0x79;
    pub const FIFO_DATA_OUT_PRESSURE_H: u8 = 0x7A;
}

pub const ADDR: u8 = 0b110_1000;

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: I2c<Error = E>,
{
    pub(crate) fn register_bit_flag_high(
        &mut self,
        address: u8,
        bitmask: u8,
    ) -> Result<bool, Error<E>> {
        let data = self.read_register(address)?;
        Ok((data & bitmask) != 0)
    }

    pub(crate) fn set_register_bit_flag(
        &mut self,
        address: u8,
        bitmask: u8,
    ) -> Result<(), Error<E>> {
        let data = self.read_register(address)?;
        if (data & bitmask) == 0 {
            self.write_register(address, data | bitmask)
        } else {
            Ok(())
        }
    }

    pub(crate) fn clear_register_bit_flag(
        &mut self,
        address: u8,
        bitmask: u8,
    ) -> Result<(), Error<E>> {
        let data = self.read_register(address)?;
        if (data & bitmask) != 0 {
            self.write_register(address, data & !bitmask)
        } else {
            Ok(())
        }
    }

    pub(crate) fn write_register(&mut self, register: u8, data: u8) -> Result<(), Error<E>> {
        let payload: [u8; 2] = [register, data];
        self.i2c.write(ADDR, &payload).map_err(Error::I2C)
    }

    pub(crate) fn read_register(&mut self, register: u8) -> Result<u8, Error<E>> {
        let mut data = [0];
        self.i2c
            .write_read(ADDR, &[register], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }
}
