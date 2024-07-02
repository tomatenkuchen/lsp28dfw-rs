//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

use crate::{LPS28DFW, Registers, Error, Range}
#![deny(unsafe_code)]
#![deny(missing_docs)]

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
        let t_high = self.read_register(Registers::TemperatureOutH)? as u16;
        let t_low = self.read_register(Registers::TemperatureOutL)? as u16;

        let t_raw = (t_high << 8 | t_low) as i16 as f32;
        // scale temperature: manpage 40, section 9.24
        let t = t_raw / 100_f32;

        Ok(TemperatureInterval::new::<degree_celsius>(t))
    }
 
    /// calc pressure from 3 u8-data values from registers
    fn calc_pressure_from_regs(&mut self, data: &[u8]) -> Pressure {
        let p_reg: u32 = (data[1] as u32) << 8 | (data[2] as u32) << 16 | data[0] as u32;

        let range = match self.measuring_range_p {
            Range::Range1260hPa => 2048_f32,
            Range::Range4060hPa => 4096_f32,
        };

        let p: f32 = p_reg as f32 / range;

        Pressure::new::<hectopascal>(p)
    }
}