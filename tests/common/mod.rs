use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};
use lps28dfw::LPS28DFW;

#[test]
fn embedded_hal_mock_test() -> () {
    let expectations = [
        I2cTrans::write(0x5C, vec![1, 2]),
        I2cTrans::read(0x5C, vec![3, 4]),
    ];
    let mut i2c = I2cMock::new(&expectations);

    i2c.write(0x5C, &vec![1, 2]).unwrap();

    let mut buf = vec![0; 2];
    i2c.read(0x5C, &mut buf).unwrap();
    assert_eq!(buf, vec![3, 4]);

    i2c.done();
}
