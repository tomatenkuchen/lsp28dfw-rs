extern crate linux_embedded_hal;
extern crate lps28dfw;
extern crate uom;

use linux_embedded_hal::I2cdev;
use lps28dfw::{Averaging, I2CAddress, OutputDataRate, Range, LPS28DFW};
use uom::si::{f32::*, pressure::hectopascal, temperature_interval::degree_celsius};

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sens_p = LPS28DFW::new(dev, I2CAddress::Low, Range::Range4060hPa);
    sens_p.start(OutputDataRate::Hz1, Averaging::Over4).unwrap();
    let p = sens_p
        .read_pressure()
        .unwrap()
        .get::<uom::si::f32::hectopascal>();
    let t = sens_p
        .read_temperature()
        .unwrap()
        .get::<uom::si::f32::hectopascal>();
    println!("pressure in kPa: {p}\ntemperature in °C: {t}");
}
