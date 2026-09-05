# ina229

[![Build](https://img.shields.io/github/actions/workflow/status/justinlatimer/ina229/ci.yml?branch=master)](https://github.com/justinlatimer/ina229/actions/workflows/ci.yml)
[![Crates.io](https://img.shields.io/crates/v/ina229)](https://crates.io/crates/ina229)
[![docs.rs](https://img.shields.io/docsrs/ina229)](https://docs.rs/ina229)

Rust `embedded-hal` driver for the Texas Instruments INA229.

## Blocking and async SPI

The default API uses `embedded_hal::spi::SpiDevice` and provides blocking
methods:

```rust,ignore
let mut monitor = ina229::INA229::new(spi);
let voltage = monitor.bus_voltage_microvolts()?;
```

Enable the `async` feature to additionally provide `INA229Async`, which uses
`embedded_hal_async::spi::SpiDevice`:

```toml
[dependencies]
ina229 = { version = "0.1", features = ["async"] }
```

```rust,ignore
let mut monitor = ina229::INA229Async::new(spi);
let voltage = monitor.bus_voltage_microvolts().await?;
```

The blocking `INA229` remains available when the feature is enabled, so blocking
and async instances can be used together in the same build.

## License

ina229 is distributed under the terms of both the MIT license and the
Apache License (Version 2.0).

See [LICENSE-APACHE](LICENSE-APACHE) and [LICENSE-MIT](LICENSE-MIT) for details.
