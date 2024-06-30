#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

extern crate embedded_hal;
extern crate uom;

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
    TemperatureOutL = 0x2b,
    /// temperature sensor high bits
    TemperatureOutH = 0x2c,
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

/// pressure interrupt edge selection
#[derive(Copy, Clone)]
pub enum InterruptPressureLevel {
    /// issue interrupt on pressure dropping under threshold
    PressureLow = 1,
    /// issue interrupt on pressure rising above threshold
    PressureHigh = 2,
}

/// declare conversion speed of sensor
#[derive(Copy, Clone)]
pub enum OutputDataRate {
    /// disable sensor, or use one shot mode for one single value
    Stop,
    /// one value per second
    Hz1,
    /// one value per second
    Hz4,
    /// one value per second
    Hz10,
    /// one value per second
    Hz25,
    /// one value per second
    Hz50,
    /// one value per second
    Hz75,
    /// one value per second
    Hz100,
    /// one value per second
    Hz200,
}

/// defines how many measurements should be averaged over before publishing in result register
#[derive(Copy, Clone)]
pub enum Averaging {
    /// averages 4 measurements to output
    Over4,
    /// averages 8 measurements to output
    Over8,
    /// averages 16 measurements to output
    Over16,
    /// averages 32 measurements to output
    Over32,
    /// averages 64 measurements to output
    Over64,
    /// averages 128 measurements to output
    Over128,
    /// averages 512 measurements to output
    Over512 = 7,
}

/// configuration struct of sensor
#[derive(Debug, Copy, Clone)]
pub struct LPS28DFW<I2C> {
    /// The concrete I²C device implementation.
    i2c: I2C,
    /// i2c address, depending on
    address: I2CAddress,
    /// measuring range for pressure
    measuring_range_p: Range,
    /// measureing buffer for fifo flushing
    fifo_buffer: [Pressure; 128],
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
            fifo_buffer: [Pressure::new::<hectopascal>(0_f32); 128],
        }
    }

    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// start conversion of stop it
    pub fn start(&mut self, data_rate: OutputDataRate, average: Averaging) -> Result<(), Error<E>> {
        let reg: u8 = average as u8 | (data_rate as u8) << 3;
        self.write_register(Registers::Control1, reg)
    }

    /// stop conversion of stop it
    pub fn stop(&mut self) -> Result<(), Error<E>> {
        self.write_register(Registers::Control1, 0u8)
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

    /// enable or disable pressure conversion on demand by writing 0 to Control1
    pub fn oneshot(&mut self, enable: bool) -> Result<(), Error<E>> {
        if enable {
            self.set_bits(Registers::Control2, 0x1)
        } else {
            self.clear_bits(Registers::Control2, 0x1)
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

    /// flush fifo
    pub fn flush_fifo(&mut self) -> Result<&[Pressure], Error<E>> {
        // check how many bytes we need to read
        let number_of_unread_data = self.read_register(Registers::FifoStatus1)? as usize;
        let number_of_registers_to_read = (number_of_unread_data * 3) as usize;

        // read fifo
        let mut data: [u8; 384] = [0; 384];
        let dataslice = &mut data[0..number_of_registers_to_read];
        self.read_registers(Registers::FifoDataOutPressureXtraLow, dataslice)?;

        // have a reference to fifo buffer before closure catches my self
        let buf = &mut self.fifo_buffer;

        // cut data to chunks of 3 bytes. Then assemble them to a pressure data point.
        let mut datachunks = dataslice.chunks(3).map(|c| self.calc_pressure_from_regs(c));

        let mut pressure_slice = &mut buf[0..number_of_unread_data];

        pressure_slice
            .iter_mut()
            .for_each(|&mut ref mut x| *x = datachunks.next().unwrap());

        Ok(&self.fifo_buffer[0..number_of_unread_data])
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

    fn read_registers(&mut self, register: Registers, data: &mut [u8]) -> Result<(), Error<E>> {
        let address = self.address as u8;
        self.i2c
            .write_read(address, &[register as u8], &mut data[..])
            .map_err(Error::I2C)
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
