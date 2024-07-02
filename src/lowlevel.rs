#![deny(unsafe_code)]
#![deny(missing_docs)]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

use crate::{Error, Registers, LPS28DFW};
use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// write data to a register, potentially overriding current values
    pub fn write_register(&mut self, register: Registers, data: u8) -> Result<(), Error<E>> {
        let payload: [u8; 2] = [register as u8, data];
        let address = self.address as u8;
        self.i2c.write(address, &payload).map_err(Error::I2C)
    }

    /// read register values
    pub fn read_register(&mut self, register: Registers) -> Result<u8, Error<E>> {
        let mut data = [0];
        let address = self.address as u8;
        self.i2c
            .write_read(address, &[register as u8], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }

    /// read multiple registers from a starting address on
    pub fn read_registers(&mut self, register: Registers, data: &mut [u8]) -> Result<(), Error<E>> {
        let address = self.address as u8;
        self.i2c
            .write_read(address, &[register as u8], &mut data[..])
            .map_err(Error::I2C)
    }

    /// set a bits in a register
    pub fn set_bits(&mut self, register: Registers, bits: u8) -> Result<(), Error<E>> {
        let reg: u8 = self.read_register(register)?;
        if (reg & bits) == 0 {
            let reg_set = reg | bits;
            self.write_register(register, reg_set)
        } else {
            Ok(())
        }
    }

    /// clear bits in a register
    pub fn clear_bits(&mut self, register: Registers, bits: u8) -> Result<(), Error<E>> {
        let reg: u8 = self.read_register(register)?;
        if (reg & bits) != 0 {
            let reg_clear = reg & !bits;
            self.write_register(register, reg_clear)
        } else {
            Ok(())
        }
    }
}
