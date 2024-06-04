#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

extern crate embedded_hal;
extern crate uom;

mod register_access;

pub mod lps28dfw {

    /// All possible errors in this crate
    #[derive(Debug)]
    pub enum Error<E> {
        /// I²C bus error
        I2C(E),
        /// Invalid input data.
        InvalidInputData,
    }

    pub enum I2CAddress {
        Low = 0x5C,
        High = 0x5D,
    }

    #[derive(Debug, Default)]
    pub struct LPS28DFW<I2C> {
        /// The concrete I²C device implementation.
        i2c: I2C,
        /// i2c address, depending on
        address: I2CAddress,
        /// measuring range for pressure
        measuring_range_p: uom::si::pressure::kilopascal,
    }

    impl<I2C, E> LPS28DFW<I2C>
    where
        I2C: embedded_hal::i2c::I2c<Error = E>,
    {
        /// Create a new instance.
        pub fn new(_i2c: I2C, _address: I2CAddress, range: uom::si::pressure::kilopascal) -> Self {
            LPS28DFW {
                i2c: _i2c,
                address: _address,
                measuring_range_p: range,
            }
        }

        /// Destroy driver instance, return I²C bus instance.
        pub fn destroy(self) -> I2C {
            self.i2c
        }
    }
}
