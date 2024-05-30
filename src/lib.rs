#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

use embedded_hal::i2c::I2c;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
    /// Invalid input data.
    InvalidInputData,
}

#[derive(Debug, Default)]
pub struct LPS28DFW<I2C> {
    /// The concrete I²C device implementation.
    i2c: I2C,
}

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Create a new instance.
    pub fn new(i2c: I2C) -> Self {
        LPS28DFW { i2c }
    }

    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}
