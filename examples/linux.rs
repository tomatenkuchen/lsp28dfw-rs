extern crate linux_embedded_hal;
extern crate lps28dfw;

use linux_embedded_hal::I2cdev;
use lps28dfw::{Address, Averaging, Range, SamplingRate, LPS28DFW};

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sens_p = LPS28DFW::new(dev, Address::low, Range::Range4060hPa);
    sens_p.start(SamplingRate::Hz_1, Averaging::over_4).unwrap();
    let p = sens_p.get_pressure().unwrap();
    let t = sens_p.get_temperature().unwrap();
    println!("pressure in kPa: {p}\ntemperature in °C: {t}");
}
