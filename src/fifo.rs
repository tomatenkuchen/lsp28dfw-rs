#![deny(unsafe_code)]
#![deny(missing_docs)]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

use crate::{Error, Registers, LPS28DFW};
use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

/// Fifo Modes, as described in manpage 13, paragraph 5.1-6
#[derive(Debug, Default, Copy, Clone)]
pub enum FifoMode {
    /// no use of fifo, or reset from other fifo mode (mandatory), manpage p 5.1
    #[default]
    Bypass = 0,
    /// basic fifo buffering. needs to be reset when full by switching to bypass mode
    Fifo,
    /// fifo behaves like a ring buffer
    ContinuousDynStream,
    /// starts writing to fifo only after interrupt signal
    BypassToFifo = 5,
    /// starts writing to fifo only after interrupt signal. behaves like ringbuffer.
    BypassToContinuous,
    /// fifo behaves like ringbuffer, until interrupt switches behaviour to formal fifo mode
    ContinuousDynStreamToFifo,
}

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// fifo control
    pub fn fifo_config(
        &mut self,
        mode: FifoMode,
        stop_on_watermark: bool,
        watermark: u8,
    ) -> Result<(), Error<E>> {
        // check if watermark is smaller than 64
        if watermark > 128 {
            return Err(Error::InvalidInputData);
        }

        // set control register
        let reg_ctrl: u8 = (mode as u8) | (stop_on_watermark as u8) << 3;
        self.write_register(Registers::FifoWatermark, watermark)?;

        self.write_register(Registers::FifoControl, reg_ctrl)
    }

    /// flush fifo
    pub fn flush_fifo(&mut self) -> Result<&[Pressure], Error<E>> {
        // check how many bytes we need to read
        let number_of_unread_data = self.read_register(Registers::FifoStatus1)? as usize;
        let number_of_registers_to_read = number_of_unread_data * 3;

        // read fifo
        let mut data: [u8; 384] = [0; 384];
        let dataslice = &mut data[0..number_of_registers_to_read];
        self.read_registers(Registers::FifoDataOutPressureXtraLow, dataslice)?;

        // cut data to chunks of 3 bytes. Then assemble them to a pressure data point.
        let mut datachunks = dataslice.chunks(3).map(|c| self.calc_pressure_from_regs(c));

        // create a buffer for pressure data
        let mut buf = [Pressure::new::<hectopascal>(0f32); 128];
        let pressure_slice = &mut buf[0..number_of_unread_data];

        // populate buffer with data from closure
        pressure_slice
            .iter_mut()
            .for_each(|&mut ref mut x| *x = datachunks.next().unwrap());

        // copy data to member
        self.fifo_buffer.copy_from_slice(pressure_slice);
        Ok(&self.fifo_buffer[0..number_of_unread_data])
    }
}
