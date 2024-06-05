use super::{Error, LPS28DFW};
use embedded_hal::i2c::I2c;

mod register_access {
    #[repr(u8)]
    pub enum Registers {
        InterruptCfg(u8) = 0x0b,
        ThresholdPressureLow(u8) = 0x0c,
        ThresholdPressureHigh(u8) = 0x0d,
        TnterfaceControl(u8) = 0x0e,
        WhoAmI(u8) = 0x0f,
        Control1(u8) = 0x10,
        Control2(u8) = 0x11,
        Control3(u8) = 0x12,
        Control4(u8) = 0x13,
        FifoControl(u8) = 0x14,
        FifoWatermark(u8) = 0x15,
        ReferencePressureLow(u8) = 0x16,
        ReferencePressureHigh(u8) = 0x17,
        I3cInterfaceControl(u8) = 0x19,
        PressureOffsetLow(u8) = 0x1a,
        PressureOffsetHigh(u8) = 0x1b,
        InterruptSource(u8) = 0x24,
        FifoStatus1(u8) = 0x25,
        FifoStatus2(u8) = 0x26,
        Status(u8) = 0x27,
        PressureOutXtraLow(u8) = 0x28,
        PressureOutL(u8) = 0x29,
        PressureOutH(u8) = 0x2a,
        TemperatureoutL(u8) = 0x2b,
        TemperatureoutH(u8) = 0x2c,
        FifoDataOutPressureXtraLow(u8) = 0x78,
        FifoDataOutPressureLow(u8) = 0x79,
        FifoDataOutPressureHigh(u8) = 0x7a,
    }

    impl<I2C, E> LPS28DFW<I2C>
    where
        I2C: I2c<Error = E>,
    {
        fn write_register(&mut self, register: Registers, data: u8) -> Result<(), Error<E>> {
            let payload: [u8; 2] = [register as u8, data];
            self.i2c.write(self.address, &payload).map_err(Error::I2C)
        }

        fn read_register(&mut self, register: Registers) -> Result<u8, Error<E>> {
            let mut data = [0];
            self.i2c
                .write_read(self.address, &[register], &mut data)
                .map_err(Error::I2C)
                .and(Ok(data[0]))
        }

        fn set_bits(&mut self, register: Registers, bits: u8) -> Result<(), Error<E>> {
            let reg: u8 = read_register(register).unwrap();
            reg |= bits;
            write_register(register, reg)
        }

        fn clear_bits(&mut self, register: Registers, bits: u8) -> Result<(), Error<E>> {
            let reg: u8 = read_register(register).unwrap();
            reg &= !bits;
            self.write_register(register, reg)
        }
    }
}
