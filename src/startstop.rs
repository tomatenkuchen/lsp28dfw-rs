#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

extern crate embedded_hal;
extern crate uom;

use crate::{Averaging, Error, OutputDataRate, Registers, LPS28DFW};
use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// start conversion of stop it
    pub fn start(&mut self, data_rate: OutputDataRate, average: Averaging) -> Result<(), Error<E>> {
        let reg: u8 = average as u8 | (data_rate as u8) << 3;
        self.write_register(Registers::Control1, reg)
    }

    /// stop conversion of stop it
    pub fn stop(&mut self) -> Result<(), Error<E>> {
        self.write_register(Registers::Control1, 0u8)
    }

    /// enable or disable pressure conversion on demand by writing 0 to Control1
    pub fn enable_oneshot(&mut self, enable: bool) -> Result<(), Error<E>> {
        if enable {
            self.set_bits(Registers::Control2, 0x1)
        } else {
            self.clear_bits(Registers::Control2, 0x1)
        }
    }

    /// enable or disable pressure conversion on demand by writing 0 to Control1
    pub fn oneshot(&mut self) -> Result<(), Error<E>> {
        self.write_register(Registers::Control1, 0)
    }
}
