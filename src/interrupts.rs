use crate::register_access;
use uom::si::f32::*;
use uom::si::pressure;
use crate::{lps28dfw, Error, register_access};
use embedded_hal::i2c::I2c;

mod Interrupts {

    pub enum InterruptPressureLevel {
        pressure_low,
        pressure_high,
    }

    impl<I2C, E> LPS28DFW<I2C> where I2C: I2c<Error = E>, {
        pub fn enable_interrupt(&mut self, type : InterruptPressureLevel, threshold : pressure) -> Result<bool, Error<E>> {
            let bits = match type {
                pressure_low => 0x1u8
                pressure_high => 0x2u8
            }
            self.set_bits(register_access::Registers::interrupt_cfg.address, bits)
            todo!("write limit to register")
        }

        pub fn disable_interrupt(&mut self, type: InterruptPressureLevel) -> Result<bool, Error<E>> {
            let bits = match type {
                pressure_low => 0x1u8
                pressure_high => 0x2u8
            }
            self.clear_bits(register_access::Registers::interrupt_cfg.address, bits)
        }
    }

}

