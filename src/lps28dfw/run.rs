use uom::si::f32::*;
use uom::si::pressure::kilopascal;
use embedded_hal::i2c::I2c;
use super::{LPS28DFW, Error};

mod run {

    pub enum SamplingRate{
        OneShot = 0,
        Hz1 = 1,
        Hz4 = 2,
        Hz10 = 3,
        Hz25 = 4,
        Hz50 = 5,
        Hz75 = 6,
        Hz100 = 7,
        Hz200 = 8,
    }

    pub enum Averaging {
        Over4Samples = 0,
        Over8Samples = 1,
        Over16Samples = 2,
        Over32Samples = 3,
        Over64Samples = 4,
        Over128Samples = 5,
        // not a bug: 256 is not specified: see man page 30
        Over512Samples = 7,
    }

    impl<I2C, E> LPS28DFW<I2C> where I2C: I2c<Error = E>, {
        pub fn start(&mut self, sampling_rate: SamplingRate, average: Averaging) -> Result<bool, Error<E>> {
            let reg : u8 = sampling_rate as u8 << 3 | average as u8;
            self.write_register(Registers::config_1, reg)
        }

        pub fn stop(&mut self) -> Result<(), Error<E>> {
            self.write_register(Registers::config_1, 0)
        }
    }
}
