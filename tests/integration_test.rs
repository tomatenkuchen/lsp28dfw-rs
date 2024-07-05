extern crate embedded_hal_mock;
extern crate lps28dfw;
extern crate uom;

use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};
use lps28dfw::{I2CAddress, Range, LPS28DFW};
use uom::si::pressure::hectopascal;

#[test]
fn test_embedded_hal_mock_test() {
    let expectations = [
        I2cTrans::write(0x5C, vec![0x28]),
        I2cTrans::read(0x5C, vec![0xFF, 0xFF, 0xFF]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut sens_p = LPS28DFW::new(i2c, I2CAddress::Low, Range::Range1260hPa);
    let p_meas = sens_p.read_pressure();

    assert_eq!(p_meas.unwrap().get::<hectopascal>(), 1260f32);
}
