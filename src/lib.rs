#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

extern crate uom;
mod register_access;

use embedded_hal::i2c::I2c;
use uom::si::f32::*
use uom::si::pressure;

pub mod LPS28DFW {

    /// All possible errors in this crate
    #[derive(Debug)]
    pub enum Error<E> {
        /// I²C bus error
        I2C(E),
        /// Invalid input data.
        InvalidInputData,
    }

    pub enum I2CAddress {
        low = 0x5C,
        high = 0x5D,
    }

    #[derive(Debug, Default)]
    pub struct LPS28DFW<I2C> {
        /// The concrete I²C device implementation.
        i2c: I2C,
        /// i2c address, depending on
        address: I2CAddress,
        /// measuring range for pressure
        measuring_range_p : pressure,
    }

    impl<I2C, E> LPS28DFW<I2C>
    where
        I2C: I2c<Error = E>,
    {
        /// Create a new instance.
        pub fn new(i2c: I2C, address: I2CAddress) -> Self {
            LPS28DFW {
                i2c: i2c,
                address: address,
            }
        }

        /// Destroy driver instance, return I²C bus instance.
        pub fn destroy(self) -> I2C {
            self.i2c
        }
    }
}
