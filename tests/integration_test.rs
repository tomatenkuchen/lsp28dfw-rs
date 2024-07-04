use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};
use lps28dfw::{I2CAddress, Range, LPS28DFW};
use uom::si::f32::Pressure;

#[test]
fn test_embedded_hal_mock_test() {
    let expectations = [
        I2cTrans::write(0x5C, vec![0x28]),
        I2cTrans::read(0x5C, vec![0xFF, 0xFF, 0xFF]),
    ];
    let mut i2c = I2cMock::new(&expectations);

    let sens_p = LPS28DFW::new(i2c, I2CAddress::low, Range::Range1060hPa);
    let p_meas = sens_p.read_pressure();

    assert_eq!(p_meas, Pressure::new::<hectopascal>(1060f32));

    i2c.done();
}
