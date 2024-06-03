extern crate uom;
use uom::si::f32::*
use uom::si::pressure;
use crate::{BitFlags, Ds1307, Error, Register};
use embedded_hal::i2c::I2c;

pub enum SamplingRate{
    oneshot = 0,
    Hz_1 = 1,
    Hz_4 = 2,
    Hz_10 = 3,
    Hz_25 = 4,
    Hz_50 = 5,
    Hz_75 = 6,
    Hz_100 = 7,
    Hz_200 = 8,
}

pub enum Averaging {
    over_4 = 0,
    over_8 = 1,
    over_16 = 2,
    over_32 = 3,
    over_64 = 4,
    over_128 = 5,
    // not a bug: 256 is not specified: see man page 30
    over_512 = 7,
}

impl<I2C, E> LPS28DFW<I2C> where I2C: I2c<Error = E>, {
    pub fn start(&mut self, sampling_rate: SamplingRate, average: Averaging) -> Result<bool, Error<E>> {
        let reg = sampling_rate as u8 << 3 || average as u8;
        self.write_register(Registers::config_1, reg)
    }

    pub fn stop(&mut self) -> Result<(), Error<E>> {
        self.write_register(Registers::config_1, 0)
    }
}
