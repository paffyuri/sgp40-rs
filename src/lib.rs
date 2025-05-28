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
            Command::SoftReset => (0x0006, 1),
        }
    }
}
