//! Platform agnostic Rust driver for Sensirion SGP40 device with
//! gas, temperature and humidity sensors based on
//! the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! ## Sensirion SGP40
//!
//! Sensirion SGP40 is a low-power accurate gas sensor for air quality application.
//! The sensor has different sampling rates to optimize power-consumption per application
//! bases as well as ability save and set the baseline for faster start-up accuracy.
//! The sensor uses I²C interface and measures TVOC (*Total Volatile Organic Compounds*)
//!
//! Datasheet: https://www.sensirion.com/file/datasheet_sgp40
//!
//! ## Usage
//!
//! ### Instantiating
//!
//! Import this crate and an `embedded_hal` implementation, then instantiate
//! the device:
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//!
//! use hal::{Delay, I2cdev};
//! use sgp40::Sgp40;
//!
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sgp = Sgp40::new(dev, 0x59, Delay);
//! ```
//! ### Doing Measurements
//!
//! The device is doing measurements independently of the driver and calls to the device
//! will just fetch the latest information making the usage easy.
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//! use hal::{Delay, I2cdev};
//!
//! use std::time::Duration;
//! use std::thread;
//!
//! use sgp40::Sgp40;
//!
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//!
//! let mut sensor = Sgp40::new(dev, 0x59, Delay);
//!
//! // Discard the first 45 samples as the algorithm is just warming up.
//! for _ in 1..45 {
//!     sensor.measure_voc_index().unwrap();
//! }
//!
//! loop {
//!     if let Ok(result) = sensor.measure_voc_index() {
//!         println!("VOC index: {}", result);
//!     }
//!     else {
//!         println!("Failed I2C reading");
//!     }
//!
//!     thread::sleep(Duration::new(1_u64, 0));
//! }
//! ```
//! ### VOC Index calculation
//! VOC index calculation is not no-std proof right now so if this is a problem for you, then
//! you want to turn the feature off by turning "the defaults off". Work for no-std index calculation
//! will start soon.
#![cfg_attr(not(test), no_std)]
#![allow(non_snake_case)]
#![allow(dead_code)]

#[cfg(feature = "sync")]
mod synchronous;
#[cfg(feature = "sync")]
use embedded_hal::i2c::I2c;
#[cfg(feature = "sync")]
pub use synchronous::*;

#[cfg(feature = "voc_index")]
mod vocalg;

#[cfg(feature = "async")]
mod asynchronous;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;
#[cfg(feature = "async")]
pub use asynchronous::*;

use sensirion_i2c::i2c;

#[cfg(feature = "voc_index")]
use crate::vocalg::VocAlgorithm;

/// Sgp40 errors
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2c(E),
    /// CRC checksum validation failed
    Crc,
    /// Self test failed
    SelfTest,
}


impl<E, I> From<i2c::Error<I>> for Error<E>
where
    I: I2c<Error = E>,
{
    fn from(err: i2c::Error<I>) -> Self {
        match err {
            i2c::Error::Crc => Error::Crc,
            i2c::Error::I2cWrite(e) => Error::I2c(e),
            i2c::Error::I2cRead(e) => Error::I2c(e),
        }
    }
}
#[derive(Debug, Copy, Clone)]
enum Command {
    /// Measures raw signal
    MeasurementRaw,
    /// Gets chips serial number
    Serial,
    /// Stops the measurement
    HeaterOff,
    /// Build-in self-test. This should be normally needed by any application
    MeasureTest,
    /// Get chipset featureset
    //FeatureSet,
    /// This is I²C wide command resetting all devices connected to the same bus
    SoftReset,
}

impl Command {
    /// Command and the requested delay in ms
    fn as_tuple(self) -> (u16, u32) {
        match self {
            Command::MeasurementRaw => (0x260f, 30),
            Command::Serial => (0x3682, 1),
            Command::HeaterOff => (0x3615, 1),
            Command::MeasureTest => (0x280e, 250),
            //Command::FeatureSet => (0x202f, 1),
            Command::SoftReset => (0x0006, 1),
        }
    }
}
