use crate::{Error, LPS28DFW};
use embedded_hal::i2c::I2c;

struct Register {
    address:u8,
    default_value:u8
}

pub enum Registers {
    interrupt_cfg(Register) = {address:0x0b, default_value:0x00},
    threshold_pressure_low(Register) = {address:0x0c, default_value:0x00},
    threshold_pressure_high(Register) = {address:0x0d, default_value:0x00},
    interface_control(Register) = {address:0x0e, default_value:0x00},
    who_am_i(Register) = {address:0x0f,default_value:0xb4},
    control_1(Register) = {address:0x10, default_value:0x00},
    control_2(Register) = {address:0x11, default_value:0x00},
    control_3(Register) = {address:0x12, default_value:0x01},
    control_4(Register) = {address:0x13, default_value:0x00},
    fifo_control(Register) = {address:0x14, default_value:0x00},
    fifo_watermark(Register) = {address:0x15, default_value:0x00},
    reference_pressure_low(Register) = {address:0x16, default_value:0x00},
    reference_pressure_high(Register) = {address:0x17, default_value:0x00},
    i3c_interface_control(Register) = {address:0x19, default_value:0x80},
    pressure_offset_low(Register) = {address:0x1a, default_value:0x00},
    pressure_offset_high(Register) = {address:0x1b, default_value:0x00},
    interrupt_source(Register) = {address:0x24, default_value:0x00},
    fifo_status1(Register) = {address:0x25, default_value:0x00},
    fifo_status2(Register) = {address:0x26, default_value:0x00},
    status(Register) = {address:0x27, default_value:0x00},
    pressure_out_xl(Register) = {address:0x28, default_value:0x00},
    pressure_out_l(Register) = {address:0x29, default_value:0x00},
    pressure_out_h(Register) = {address:0x2a, default_value:0x00},
    temperature_out_l(Register) = {address:0x2b, default_value:0x00},
    temperature_out_h(Register) = {address:0x2c, default_value:0x00},
    fifo_data_out_pressure_xl(Register) = {address:0x78, default_value:0x00},
    fifo_data_out_pressure_l(Register) = {address:0x79, default_value:0x00},
    fifo_data_out_pressure_h(Register) = {address:0x7a, default_value:0x00},
}

impl<I2C, E> LPS28DFW<I2C> where I2C: I2c<Error = E>, {
    fn write_register(&mut self, register: Registers, data: u8) -> Result<(), Error<E>> {
        let payload: [u8; 2] = [register as u8, data];
        self.i2c.write(self.address, &payload).map_err(Error::I2C)
    }

    fn read_register(&mut self, register: Registers) -> Result<u8, Error<E>> {
        let mut data = [0];
        self.i2c
            .write_read(ADDR, &[register], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }

    fn set_bits(&mut self, register: Registers, bits:u8) -> Result<(), Error<E>> {
        let reg:u8 = read_register(register).unwrap();
        reg |= bits; 
        write_register(register, reg)
    }

    fn clear_bits(&mut self, register: Registers, bits:u8) -> Result<(), Error<E>> {
        let reg:u8 = read_register(register).unwrap();
        reg &= ~bits;
        self.write_register(register, reg)
    }
}
