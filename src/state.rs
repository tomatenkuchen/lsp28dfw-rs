#![deny(unsafe_code)]
#![deny(missing_docs)]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

use crate::{Error, Registers, LPS28DFW};
use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

/// Device status struct
pub struct Status {
    is_boot_running: bool,
    is_interrupt_active: bool,
    is_low_pressure_event: bool,
    is_high_pressure_event: bool,
    unread_data_in_fifo: u8,
    is_fifo_watermark_reached: bool,
    is_fifo_overrun: bool,
    is_fifo_full: bool,
    is_temperature_data_overrun: bool,
    is_pressure_data_overrun: bool,
    is_temperature_data_available: bool,
    is_pressure_data_available: bool,
}

/// Low pass filter strength
#[derive(Debug, Default, Copy, Clone)]
pub enum LowPassStrength {
    /// ODR/4
    #[default]
    Low = 0,
    /// ODR/9
    High = 1,
}

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// tara for pressure output: all measurements from now on are relative to current pressure
    /// level
    pub fn autozero_set(&mut self) -> Result<(), Error<E>> {
        self.set_bits(Registers::InterruptCfg, 0x20)
    }

    /// switch back from autozero to normal mode
    pub fn autozero_reset(&mut self) -> Result<(), Error<E>> {
        self.set_bits(Registers::InterruptCfg, 0x10)
    }

    /// interrupt generation is now issued based on the current pressure
    pub fn auto_reference_pressure_set(&mut self) -> Result<(), Error<E>> {
        self.set_bits(Registers::InterruptCfg, 0x80)
    }

    /// switch back from autozero to normal mode
    pub fn auto_reference_pressure_reset(&mut self) -> Result<(), Error<E>> {
        self.set_bits(Registers::InterruptCfg, 0x40)
    }

    /// sends an identify request to the sensor and expects a certain number as an answer
    pub fn identify(&mut self) -> Result<(), Error<E>> {
        const IDENTIFIER: u8 = 0xB4;
        let idn = self.read_register(Registers::WhoAmI)?;
        if idn != IDENTIFIER {
            Err(Error::DeviceIdentityFailure)
        } else {
            Ok(())
        }
    }

    /// reuse internal configuration from boot rom file
    pub fn boot_rom_content(&mut self) -> Result<(), Error<E>> {
        self.set_bits(Registers::Control2, 0x80)
    }

    /// enable low pass filter on results
    pub fn low_pass_filter(
        &mut self,
        enable: bool,
        strength: LowPassStrength,
    ) -> Result<(), Error<E>> {
        if enable {
            self.set_bits(Registers::Control2, 0x10)?;
        } else {
            self.clear_bits(Registers::Control2, 0x10)?;
        }

        match strength {
            LowPassStrength::Low => self.set_bits(Registers::Control2, 0x20),
            LowPassStrength::High => self.set_bits(Registers::Control2, 0x20),
        }
    }

    /// shadow u16 and u24 bit registers until all bytes are written
    pub fn block_data_update(&mut self, enable: bool) -> Result<(), Error<E>> {
        if enable {
            self.set_bits(Registers::Control2, 0x8)
        } else {
            self.clear_bits(Registers::Control2, 0x8)
        }
    }

    /// resets all registers to startup
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.set_bits(Registers::Control2, 0x4)
    }

    /// i²c or i³c pin configuration
    pub fn ixc_interface_config(
        &mut self,
        enable_sda_pin_pullup: bool,
        enable_i3c_data_ready_pin: bool,
    ) -> Result<(), Error<E>> {
        let reg: u8 = (enable_sda_pin_pullup as u8) << 4 | (enable_i3c_data_ready_pin as u8) << 5;
        self.write_register(Registers::InterfaceControl, reg)
    }

    /// takes documentation and packages it nicely into a struct
    pub fn get_status(&mut self) -> Result<Status, Error<E>> {
        let mut data: [u8; 4] = [0; 4];
        self.read_registers(Registers::InterruptSource, &mut data)?;

        let status = Status {
            is_boot_running: (data[0] & 0x80) > 0,
            is_interrupt_active: (data[0] & 0x04) > 0,
            is_low_pressure_event: (data[0] & 0x02) > 0,
            is_high_pressure_event: (data[0] & 0x01) > 0,
            unread_data_in_fifo: data[1],
            is_fifo_watermark_reached: data[2] & 0x80 > 0,
            is_fifo_overrun: (data[2] & 0x40) > 0,
            is_fifo_full: (data[2] & 0x20) > 0,
            is_temperature_data_overrun: (data[3] & 0x20) > 0,
            is_pressure_data_overrun: (data[3] & 0x10) > 0,
            is_temperature_data_available: (data[3] & 0x02) > 0,
            is_pressure_data_available: (data[3] & 0x01) > 0,
        };

        Ok(status)
    }
}
