#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

extern crate embedded_hal;
extern crate uom;

use uom::si::f32::*;

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

/// measurement range for sensor
#[derive(Debug, Copy, Clone)]
pub enum Range {
    /// minimum range of 1260 hPa
    Range1260hPa = 0,
    /// max range of 4060 hPa
    Range4060hPa = 1,
}

/// configuration struct of sensor
#[derive(Debug)]
pub struct LPS28DFW<I2C> {
    /// The concrete I²C device implementation.
    i2c: I2C,
    /// i2c address, depending on
    address: I2CAddress,
    /// measuring range for pressure
    measuring_range_p: Range,
}

/// pressure interrupt edge selection
#[derive(Copy, Clone)]
pub enum InterruptPressureLevel {
    /// issue interrupt on pressure dropping under threshold
    PressureLow = 1,
    /// issue interrupt on pressure rising above threshold
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
    ) -> Result<(), Error<E>> {
        // set interrupt level comparison and output behaviour
        let interrupt_config: u8 =
            (interrupt_pressure_level as u8) | ((make_interrupt_latched as u8) << 2);

        let threshold_hPa = threshold.get::<uom::si::pressure::hectopascal>();

        //check threshold input
        match self.measuring_range_p {
            Range::Range1260hPa => {
                if threshold_hPa >= 1260_f32 {
                    Err(Error::InvalidInputData)
                }
            }
            Range::Range4060hPa => {
                if threshold_hPa >= 4060_f32 {
                    Err(Error::InvalidInputData)
                }
            }
            _ => {
                if threshold_hPa <= 0_f32 {
                    Err(Error::InvalidInputData)
                }
            }
        }

        // calculate register value for threshold from input and range
        let reg_thresh: u16 = match self.measuring_range_p {
            Range::Range1260hPa => threshold.value as u16 * 16,
            Range::Range4060hPa => threshold.value as u16 * 8,
        };

        self.set_bits(Registers::InterruptCfg, interrupt_config)?;

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

    /// sends an identify request to the sensor and expects a certain number as an answer
    pub fn identify(mut self) -> Result<(), Error<E>> {
        const IDENTIFIER: u8 = 0xB4;
        let idn = self.read_register(Registers::WhoAmI)?;
        if idn != IDENTIFIER {
            Err(Error::DeviceIdentityFailure)
        } else {
            Ok(())
        }
    }

    /// configure the pinout behaviour of some signaling pins
    pub fn interface_control(
        mut self,
        disable_data_ready_pin_pulldown: bool,
        enable_sda_pin_pullup: bool,
        enable_i3c_data_ready_pin: bool,
    ) -> Result<(), Error<E>> {
        let reg: u8 = (disable_data_ready_pin_pulldown as u8) << 2
            | (enable_sda_pin_pullup as u8) << 4
            | (enable_i3c_data_ready_pin as u8) << 5;
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
