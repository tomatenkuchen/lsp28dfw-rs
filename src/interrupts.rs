#![deny(unsafe_code)]
#![deny(missing_docs)]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

use crate::{Error, InterruptPressureLevel, Range, Registers, LPS28DFW};
use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// enable an interrupt on pressure passing a threshold
    ///
    /// * `interrupt_pressure_level` - define what edge the interrupt is supposed to be signaled
    /// * `threshold` - tell the sensor at what pressure level the interrupt should be issued
    /// * `make_interrupt_latched` - define if interrupt signal on interrupt pin should be a
    /// latched signal
    pub fn enable_interrupt(
        &mut self,
        interrupt_pressure_level: InterruptPressureLevel,
        threshold: uom::si::f32::Pressure,
        make_interrupt_latched: bool,
        interrupt_pin_active_low: bool,
        interrupt_pin_open_drain: bool,
    ) -> Result<(), Error<E>> {
        // set interrupt level comparison and output behaviour
        let interrupt_config: u8 =
            (interrupt_pressure_level as u8) | ((make_interrupt_latched as u8) << 2);

        let threshold_hPa = threshold.get::<uom::si::pressure::hectopascal>();

        //check threshold input
        match self.measuring_range_p {
            Range::Range1260hPa => {
                if threshold_hPa >= 1260_f32 {
                    return Err(Error::InvalidInputData);
                }
            }
            Range::Range4060hPa => {
                if threshold_hPa >= 4060_f32 {
                    return Err(Error::InvalidInputData);
                }
            }
        }

        if threshold_hPa <= 0_f32 {
            return Err(Error::InvalidInputData);
        }

        // calculate register value for threshold from input and range
        let reg_thresh: u16 = match self.measuring_range_p {
            Range::Range1260hPa => threshold.value as u16 * 16,
            Range::Range4060hPa => threshold.value as u16 * 8,
        };

        let reg_pin: u8 =
            (interrupt_pin_active_low as u8) << 3 | (interrupt_pin_open_drain as u8) << 1;

        self.set_bits(Registers::InterruptCfg, interrupt_config)?;

        self.set_bits(Registers::Control3, reg_pin)?;

        self.write_register(Registers::ThresholdPressureLow, reg_thresh as u8)?;

        self.write_register(Registers::ThresholdPressureLow, (reg_thresh >> 8) as u8)
    }

    /// disables the selected threshold crossing interrupt
    ///
    /// * `interrupt_pressure_level` - select which interrupt you want to disable
    pub fn disable_interrupt(
        &mut self,
        interrupt_pressure_level: InterruptPressureLevel,
    ) -> Result<(), Error<E>> {
        self.clear_bits(Registers::InterruptCfg, interrupt_pressure_level as u8)
    }

    /// configure the pinout behaviour of the interrupt/dataready pin
    pub fn interrupt_pin_control(
        &mut self,
        disable_pulldown: bool,
        active_low: bool,
        open_drain: bool,
    ) -> Result<(), Error<E>> {
        // config pulldown
        let if_ctrl_reg: u8 = (disable_pulldown as u8) << 2;
        self.set_bits(Registers::InterfaceControl, if_ctrl_reg)?;

        // config signal gates
        let ctrl3_reg: u8 = (active_low as u8) << 3 | (open_drain as u8) << 1;
        self.write_register(Registers::InterfaceControl, ctrl3_reg)
    }

    /// config which signals get to create
    pub fn interrupt_pin_signal_gate(
        &mut self,
        enable_data_ready_signal: bool,
        data_ready_pulsed: bool,
        enable_interrupt_signal: bool,
        fifo_full_signal: bool,
        fifo_at_watermark_signal: bool,
        fifo_overrun_signal: bool,
    ) -> Result<(), Error<E>> {
        let reg: u8 = (enable_data_ready_signal as u8) << 5
            | (data_ready_pulsed as u8) << 6
            | (enable_interrupt_signal as u8) << 4
            | (fifo_full_signal as u8) << 2
            | (fifo_at_watermark_signal as u8) << 1
            | (fifo_overrun_signal as u8);
        self.write_register(Registers::Control4, reg)
    }
}
