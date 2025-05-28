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



// Testing is focused on checking the primitive transactions. It is assumed that during
// the real sensor testing, the basic flows in the command structure has been caught.
#[cfg(test)]
mod tests {
    use embedded_hal_mock as mock_hal;

    use super::*;

    #[cfg(feature = "sync")]
    use mock_hal::eh1::{
        delay::NoopDelay,
        i2c::{Mock as I2cMock, Transaction},
    };

    const SGP40_ADDR: u8 = 0x59;

    /// Tests that the commands without parameters work
    #[test]
    #[cfg(feature = "sync")]
    fn test_basic_command() {
        let (cmd, _) = Command::MeasurementRaw.as_tuple();
        let expectations = [
            Transaction::write(
                SGP40_ADDR,
                [
                    cmd.to_be_bytes().to_vec(),
                    [0x7f, 0xfb, 0x4b, 0x66, 0x8a, 0x2f].to_vec(),
                ]
                .concat(),
            ),
            Transaction::read(SGP40_ADDR, vec![0x12, 0x34, 0x37]),
        ];
        let mut mock = I2cMock::new(&expectations);
        let mut sensor = Sgp40::new(mock.clone(), SGP40_ADDR, NoopDelay);
        let result = sensor.measure_raw().unwrap();
        assert_eq!(result, 0x1234);
        mock.done();
    }

    /// Test the `serial` function
    #[test]
    #[cfg(feature = "sync")]
    fn serial() {
        let (cmd, _) = Command::Serial.as_tuple();
        let expectations = [
            Transaction::write(0x58, cmd.to_be_bytes().to_vec()),
            Transaction::read(0x58, vec![0xde, 0xad, 0x98, 0xbe, 0xef, 0x92, 0xde, 0xad, 0x98]),
        ];
        let mut mock = I2cMock::new(&expectations);
        let mut sensor = Sgp40::new(mock.clone(), 0x58, NoopDelay);
        let serial = sensor.serial().unwrap();
        assert_eq!(serial, 0x00deadbeefdead);
        mock.done();
    }

    #[test]
    #[cfg(feature = "sync")]
    fn test_crc_error() {
        let (cmd, _) = Command::MeasureTest.as_tuple();
        let expectations = [
            Transaction::write(SGP40_ADDR, cmd.to_be_bytes().to_vec()),
            Transaction::read(SGP40_ADDR, vec![0xD4, 0x00, 0x00]),
        ];
        let mut mock = I2cMock::new(&expectations);
        let mut sensor = Sgp40::new(mock.clone(), SGP40_ADDR, NoopDelay);

        match sensor.self_test() {
            Err(Error::Crc) => {}
            Err(_) => panic!("Unexpected error in CRC test"),
            Ok(_) => panic!("Unexpected success in CRC test"),
        }
        mock.done();
    }
}