#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

//! rust driver for ST LPS28DFW pressure sensor over i2c bus
//! generalized by embedded_hal abstraction level to run on all
//! platforms supported by embedded_hal

extern crate embedded_hal;
extern crate uom;

use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

mod fifo;
mod interrupts;
mod lowlevel;
mod readdevice;
mod startstop;
mod state;

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

/// Low pass filter strength
#[derive(Debug, Copy, Clone)]
pub enum LowPassStrength {
    ///ODR/4
    Low = 0,
    /// ODR/9
    High = 1,
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

/// declare conversion speed of sensor
#[derive(Copy, Clone)]
pub enum OutputDataRate {
    /// disable sensor, or use one shot mode for one single value
    Stop,
    /// 1 value per second
    Hz1,
    /// 4 values per second
    Hz4,
    /// 10 values per second
    Hz10,
    /// 25 values per second
    Hz25,
    /// 50 values per second
    Hz50,
    /// 75 values per second
    Hz75,
    /// 100 values per second
    Hz100,
    /// 200 values per second
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

/// pressure interrupt edge selection
#[derive(Copy, Clone)]
pub enum InterruptPressureLevel {
    /// issue interrupt on pressure dropping under threshold
    PressureLow = 1,
    /// issue interrupt on pressure rising above threshold
    PressureHigh = 2,
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

/// measurement range for sensor
#[derive(Debug, Copy, Clone)]
pub enum Range {
    /// minimum range of 1260 hPa
    Range1260hPa = 0,
    /// max range of 4060 hPa
    Range4060hPa = 1,
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

/// Device status
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
}
