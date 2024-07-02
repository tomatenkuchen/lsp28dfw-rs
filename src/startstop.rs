#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

extern crate embedded_hal;
extern crate uom;

use crate::{Error, Registers, LPS28DFW};
use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

/// declare conversion speed of sensor
#[derive(Copy, Clone)]
pub enum OutputDataRate {
    /// disable sensor, or use one shot mode for one single value
    Stop,
    /// 1 value per second
    Hz1,
    /// 4 values per second
    Hz4,
    /// 10 values per second
    Hz10,
    /// 25 values per second
    Hz25,
    /// 50 values per second
    Hz50,
    /// 75 values per second
    Hz75,
    /// 100 values per second
    Hz100,
    /// 200 values per second
    Hz200,
}

/// defines how many measurements should be averaged over before publishing in result register
#[derive(Copy, Clone)]
pub enum Averaging {
    /// averages 4 measurements to output
    Over4,
    /// averages 8 measurements to output
    Over8,
    /// averages 16 measurements to output
    Over16,
    /// averages 32 measurements to output
    Over32,
    /// averages 64 measurements to output
    Over64,
    /// averages 128 measurements to output
    Over128,
    /// averages 512 measurements to output
    Over512 = 7,
}

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
