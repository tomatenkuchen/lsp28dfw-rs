#![deny(unsafe_code)]
#![deny(missing_docs)]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

use crate::{Error, Range, Registers, LPS28DFW};
use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// read pressure registers
    pub fn read_pressure(&mut self) -> Result<Pressure, Error<E>> {
        let mut data: [u8; 3] = [0, 0, 0];
        self.read_registers(Registers::PressureOutXtraLow, &mut data)?;
        let p = self.calc_pressure_from_regs(&data);
        Ok(p)
    }

    /// read temperature registers
    pub fn read_temperature(&mut self) -> Result<TemperatureInterval, Error<E>> {
        let mut data: [u8; 2] = [0,0];
        self.read_registers(Registers::TemperatureOutL, &mut data)?;

        let t_raw = ((data[1] as u16) << 8 | data[0] as u16) as i16 as f32;
        // scale temperature: manpage 40, section 9.24
        let t = t_raw / 100_f32;

        Ok(TemperatureInterval::new::<degree_celsius>(t))
    }

    /// calc pressure from 3 u8-data values from registers
    pub fn calc_pressure_from_regs(&mut self, data: &[u8]) -> Pressure {
        let p_reg: u32 = (data[1] as u32) << 8 | (data[2] as u32) << 16 | data[0] as u32;

        let range = match self.measuring_range_p {
            Range::Range1260hPa => 1260_f32,
            Range::Range4060hPa => 4060_f32,
        };

        let p: f32 = p_reg as f32 * range / (2f32.powf(24.));

        Pressure::new::<hectopascal>(p)
    }
}
