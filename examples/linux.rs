use linux_embedded_hal::I2cdev;
use LPS28DFW;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut sens_p = LPS28DFW::LPS28DFW::new(dev, LPS28DFW::Address::low);
    sens_p
        .start(LPS28DFW::SamplingRate::Hz_1, LPS28DFW::Averaging::over_4)
        .unwrap();
    let p = sens_p.get_pressure().unwrap();
    let t = sens_p.get_temperature().unwrap();
    println!("pressure in kPa: {p}\ntemperature in °C: {t}");
}
