# Rust DS1307 Real-Time Clock Driver

# hardware driver for ST LPS28DFW pressure sensor

This is a platform agnostic Rust driver for the LPS28DFW pressure sensor from ST,
based on the [`embedded-hal`] traits.

[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal

This driver allows you to:

- read pressure and temperature values from sensor
- enable pressure value interrupts
- set sampling rates

## The device

LPS28DFW is a high precision MEMS pressure and temperature sensor for multiple fluids. Pressure measuring range is variable between 1024 and 4096 kPa.

Datasheet: [LPS28DFW](https://www.st.com/resource/en/datasheet/lps28dfw.pdf)

## Usage

To use this driver, import this crate and an `embedded_hal` implementation,
then instantiate the device.

```rust
use ds1307::{DateTimeAccess, Ds1307, NaiveDate};
use linux_embedded_hal::I2cdev;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut pressure_sensor = LPS28DFW::new(dev);
    let mut pressure_sensor_cfg = LPS28DFW::get_default_config();
    pressure_sensor.write_config(pressure_sensor_cfg).unwrap();
    let pressure = pressure_sensor.get_pressure().unwrap();
    println!("{pressure} kPa");
}
```

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile on stable Rust 1.62 and up. It *might*
compile with older versions but that may change in any new patch release.

## Support

For questions, issues, feature requests, and other changes, please file an
[issue in the github project](https://github.com/tomatenkuchen/lps28dfw)

## License

Licensed under

- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in MIT Licence text.
