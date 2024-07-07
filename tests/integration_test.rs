#[macro_use]
extern crate approx;
extern crate embedded_hal_mock;
extern crate lps28dfw;
extern crate uom;

use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};
use lps28dfw::{I2CAddress, Range, LPS28DFW};
use uom::si::pressure::hectopascal;

#[test]
fn test_read_pressure() {
    let expectations = [I2cTrans::write_read(
        0x5C,
        vec![0x28],
        vec![0xFF, 0xFF, 0xFF],
    )];
    let i2c = I2cMock::new(&expectations);
    let mut i2c_clone = i2c.clone();

    let mut sens_p = LPS28DFW::new(i2c, I2CAddress::Low, Range::Range1260hPa);
    let p_meas = sens_p.read_pressure();

    relative_eq!(
        p_meas.unwrap().get::<hectopascal>(),
        1260f32,
        max_relative = 0.1
    );

    i2c_clone.done();
}

#[test]
fn test_read_temperature() {
    let expectations = [I2cTrans::write_read(0x5C, vec![0x2b], vec![0xFF, 0xFF])];
    let i2c = I2cMock::new(&expectations);
    let mut i2c_clone = i2c.clone();

    let mut sens_p = LPS28DFW::new(i2c, I2CAddress::Low, Range::Range1260hPa);
    let t_meas = sens_p.read_temperature();

    relative_eq!(
        t_meas
            .unwrap()
            .get::<uom::si::temperature_interval::degree_celsius>(),
        655.35f32,
        max_relative = 0.1
    );

    i2c_clone.done();
}

#[test]
fn test_identity() {
    let expectations = [I2cTrans::write_read(0x5C, vec![0x0f], vec![0xB4])];
    let i2c = I2cMock::new(&expectations);
    let mut i2c_clone = i2c.clone();

    let mut sens_p = LPS28DFW::new(i2c, I2CAddress::Low, Range::Range1260hPa);
    sens_p.identify().unwrap();

    i2c_clone.done();
}

#[test]
#[should_panic]
fn test_identity_panics() {
    let expectations = [I2cTrans::write_read(0x5C, vec![0x0f], vec![0xB5])];
    let i2c = I2cMock::new(&expectations);
    let mut i2c_clone = i2c.clone();

    let mut sens_p = LPS28DFW::new(i2c, I2CAddress::Low, Range::Range1260hPa);
    sens_p.identify().unwrap();

    i2c_clone.done();
}
