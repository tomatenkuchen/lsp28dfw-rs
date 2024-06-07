#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

extern crate embedded_hal;
extern crate uom;

use uom::si::{f32::*, pressure::hectopascal};

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

/// the sensor supports multiple address via a address pin.
/// if the address pin is connected to ground, look for the
/// sensor on Low, otherwise on High
#[derive(Debug, Default, Copy, Clone)]
pub enum I2CAddress {
    /// low address on i2c bus
    #[default]
    Low = 0x5C,
    /// high address
    High = 0x5D,
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

/// all registers with their register address
#[repr(u8)]
#[derive(Copy, Clone)]
enum Registers {
    /// interrupt and status related configurations
    InterruptCfg = 0x0b,
    /// interrupt threshold lower bits
    ThresholdPressureLow = 0x0c,
    /// interrupt threshold higher bits
    ThresholdPressureHigh = 0x0d,
    /// pinout control on some pins
    InterfaceControl = 0x0e,
    /// identification number
    WhoAmI = 0x0f,
    /// control on data aquisition speed an averaging
    Control1 = 0x10,
    /// aquisition control
    Control2 = 0x11,
    /// interface and interrupt controls
    Control3 = 0x12,
    /// interrupt, fifo and data ready signals
    Control4 = 0x13,
    /// fifo controls
    FifoControl = 0x14,
    /// fifo watermark
    FifoWatermark = 0x15,
    /// reference pressure lower bits
    ReferencePressureLow = 0x16,
    /// reference pressure higher bits
    ReferencePressureHigh = 0x17,
    /// special i3c interface
    I3cInterfaceControl = 0x19,
    /// pressure offset register lower bits
    PressureOffsetLow = 0x1a,
    /// pressure offset register higher bits
    PressureOffsetHigh = 0x1b,
    /// interrupt status register
    InterruptSource = 0x24,
    /// fifo status
    FifoStatus1 = 0x25,
    /// fifo status
    FifoStatus2 = 0x26,
    /// sensor status
    Status = 0x27,
    /// pressure value lower bits
    PressureOutXtraLow = 0x28,
    /// pressure sensor low bits
    PressureOutL = 0x29,
    /// ressure sensor high bits
    PressureOutH = 0x2a,
    /// temperature sensor low bits
    TemperatureoutL = 0x2b,
    /// temperature sensor high bits
    TemperatureoutH = 0x2c,
    /// fifo control
    FifoDataOutPressureXtraLow = 0x78,
    /// fifo control
    FifoDataOutPressureLow = 0x79,
    /// fifo control
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
    pub fn identify(mut self) -> Result<(), Error<E>> {
        const IDENTIFIER: u8 = 0xB4;
        let idn = self.read_register(Registers::WhoAmI)?;
        if idn != IDENTIFIER {
            Err(Error::DeviceIdentityFailure)
        } else {
            Ok(())
        }
    }

    /// reuse internal configuration from boot rom file
    pub fn boot_rom_content(mut self) -> Result<(), Error<E>> {
        self.set_bits(Registers::Control2, 0x80)
    }

    /// enable low pass filter on results
    pub fn low_pass_filter(
        mut self,
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
    pub fn block_data_update(mut self, enable: bool) -> Result<(), Error<E>> {
        if enable {
            self.set_bits(Registers::Control2, 0x8)
        } else {
            self.clear_bits(Registers::Control2, 0x8)
        }
    }

    /// enable or disable pressure conversion on demand by writing 0 to Control1
    pub fn oneshot(mut self, enable: bool) -> Result<(), Error<E>> {
        if enable {
            self.set_bits(Registers::Control2, 0x1)
        } else {
            self.clear_bits(Registers::Control2, 0x1)
        }
    }

    /// resets all registers to startup
    pub fn reset(mut self) -> Result<(), Error<E>> {
        self.set_bits(Registers::Control2, 0x4)
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

    /// read pressure registers
    pub fn read_pressure(mut self) -> Result<Pressure, Error<E>> {
        let p_low = self.read_register(Registers::PressureOutL)? as u32;
        let p_high = self.read_register(Registers::PressureOutH)? as u32;
        let p_xlow = self.read_register(Registers::PressureOutXtraLow)? as u32;

        let p_reg: u32 = p_low << 8 | p_high << 16 | p_xlow;
        let range = match self.measuring_range_p {
            Range::Range1260hPa => 2048_f32,
            Range::Range4060hPa => 4096_f32,
        };
        Pressure::new::<hectopascal>(p_reg as f32 / range)
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
