#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

extern crate embedded_hal;
extern crate uom;

use uom::si::pressure::Pressure;
use uom::si::pressure::kilopascal;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
    /// Invalid input data.
    InvalidInputData,
    /// device not available
    DeviceIdentityFailure,
}

#[derive(Debug, Default, Copy, Clone)]
pub enum I2CAddress {
    #[default]
    Low = 0x5C,
    High = 0x5D,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum Registers {
    InterruptCfg = 0x0b,
    ThresholdPressureLow = 0x0c,
    ThresholdPressureHigh = 0x0d,
    InterfaceControl = 0x0e,
    WhoAmI = 0x0f,
    Control1 = 0x10,
    Control2 = 0x11,
    Control3 = 0x12,
    Control4 = 0x13,
    FifoControl = 0x14,
    FifoWatermark = 0x15,
    ReferencePressureLow = 0x16,
    ReferencePressureHigh = 0x17,
    I3cInterfaceControl = 0x19,
    PressureOffsetLow = 0x1a,
    PressureOffsetHigh = 0x1b,
    InterruptSource = 0x24,
    FifoStatus1 = 0x25,
    FifoStatus2 = 0x26,
    Status = 0x27,
    PressureOutXtraLow = 0x28,
    PressureOutL = 0x29,
    PressureOutH = 0x2a,
    TemperatureoutL = 0x2b,
    TemperatureoutH = 0x2c,
    FifoDataOutPressureXtraLow = 0x78,
    FifoDataOutPressureLow = 0x79,
    FifoDataOutPressureHigh = 0x7a,
}

#[derive(Debug)]
#[derive(Copy, Clone)]
pub enum Range{
    Range126kPa = 0,
    Range406kPa = 1,
}

#[derive(Debug)]
pub struct LPS28DFW<I2C> {
    /// The concrete I²C device implementation.
    i2c: I2C,
    /// i2c address, depending on
    address: I2CAddress,
    /// measuring range for pressure
    measuring_range_p: Range,
}

#[derive(Copy, Clone)]
pub enum InterruptPressureLevel {
    PressureLow = 1,
    PressureHigh = 2,
}

impl<I2C, E> LPS28DFW<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// Create a new instance.
    pub fn new(_i2c: I2C, _address: I2CAddress, range: Range) -> Self {
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

    pub fn enable_interrupt(
        &mut self,
        interrupt_pressure_level: InterruptPressureLevel,
        threshold: Pressure,
        make_interrupt_latched: bool,
    ) -> Result<(), Error<E>> {
        // set interrupt level comparison and output behaviour
        let interrupt_config: u8 = (interrupt_pressure_level as u8) | ((make_interrupt_latched as u8) << 2);
        self.set_bits(Registers::InterruptCfg, interrupt_config)?;

        // calculate register value for threshold from input and range
        let reg_thresh = match self.measuring_range_p {
            Range::Range126kPa => {
                let reg_thresh_unlimited = threshold * 16;
                let reg_thresh_limited = if reg_thresh_unlimited > Pressure::new<kilopascal>(126) {Pressure::new<kilopascal>(126)} else {reg_thresh_unlimited};
                reg_thresh_limited
            }
            Range::Range406kPa => {
                let reg_thresh_unlimited = threshold * 8;
                let reg_thresh_limited = if reg_thresh_unlimited > Pressure::new<kilopascal>(406) {Pressure::new<kilopascal>(406)} else {reg_thresh_unlimited};
                reg_thresh_limited
            }
        };
        self.write_register(Registers::ThresholdPressureLow, reg_thresh as u8)?;
        self.write_register(Registers::ThresholdPressureLow, (reg_thresh as u16 >> 8) as u8)
    }

    pub fn disable_interrupt(
        &mut self,
        interrupt_pressure_level: InterruptPressureLevel,
    ) -> Result<(), Error<E>> {
        self.clear_bits(Registers::InterruptCfg, interrupt_pressure_level as u8)
    }

    pub fn identify(self) -> Result<(), Error<E>> {
        const IDENTIFIER: u8 = 0xB4;
        let idn = self.read_register(Registers::WhoAmI)?;
        if idn != IDENTIFIER {
            Error::DeviceIdentityFailure
        }else{
            Ok(())
        }
    }

    pub fn interface_control(mut self,
    disable_data_ready_pin_pulldown: bool,
    enable_sda_pin_pullup: bool,
        enable_i3c_data_ready_pin: bool) -> Result<(), Error<E>>{
        let reg: u8 = (disable_data_ready_pin_pulldown as u8) << 2 | (enable_sda_pin_pullup as u8) << 4 | (enable_i3c_data_ready_pin as u8) << 5;
        self.write_register(Registers::InterfaceControl, reg)
    }

    fn write_register(&mut self, register: Registers, data: u8) -> Result<(), Error<E>> {
        let payload: [u8; 2] = [register as u8, data];
        let address = self.address as u8;
        self.i2c.write(address, &payload).map_err(Error::I2C)
    }

    fn read_register(&mut self, register: Registers) -> Result<u8, Error<E>> {
        let mut data = [0];
        let address = self.address as u8;
        self.i2c
            .write_read(address, &[register as u8], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }

    fn set_bits(&mut self, register: Registers, bits: u8) -> Result<(), Error<E>> {
        let reg: u8 = self.read_register(register)?;
        if (reg & bits) == 0 {
            let reg_set = reg | bits;
            self.write_register(register, reg_set)
        } else {
            Ok(())
        }
    }

    fn clear_bits(&mut self, register: Registers, bits: u8) -> Result<(), Error<E>> {
        let reg: u8 = self.read_register(register)?;
        if (reg & bits) != 0 {
            let reg_clear = reg & !bits;
            self.write_register(register, reg_clear)
        } else {
            Ok(())
        }
    }
}
