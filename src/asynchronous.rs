use embedded_hal_async as hal_async;
use hal_async::delay::DelayNs;
use hal_async::i2c::I2c;

use sensirion_i2c::crc8;

use crate::{Command, Error};
#[cfg(feature = "voc_index")]
use crate::vocalg::VocAlgorithm;

/// Sgp40Async driver instance
///
/// Create the driver instance with valid I²C address (0x59) and then it is just
/// rock'n'roll. This driver doesn't require special starting but once can start to
/// make measurements right away. However, the initial values after start-up will
/// unstable so you will want to throw away some of them.
pub struct Sgp40Async<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
    temperature_offset: i16,
    #[cfg(feature = "voc_index")]
    voc: VocAlgorithm,
}

impl<I2C, D, E> Sgp40Async<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    /// Creates Sgp40Async driver
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        Sgp40Async {
            i2c,
            address,
            delay,
            temperature_offset: 0,
            #[cfg(feature = "voc_index")]
            voc: VocAlgorithm::new(),
        }
    }

    /// Command for reading values from the sensor
    async fn delayed_read_cmd(&mut self, cmd: Command, data: &mut [u8]) -> Result<(), Error<E>> {
        self.write_command(cmd).await?;
        self.read_words_with_crc(data).await?;
        Ok(())
    }

    /// Helper function to read words with CRC (async implementation)
    async fn read_words_with_crc(&mut self, data: &mut [u8]) -> Result<(), Error<E>> {
        let mut buffer = [0u8; 3];
        let words = data.len() / 2;
        
        for i in 0..words {
            self.i2c.read(self.address, &mut buffer).await.map_err(Error::I2c)?;
            
            // Verify CRC
            let expected_crc = crc8::calculate(&buffer[0..2]);
            if buffer[2] != expected_crc {
                return Err(Error::Crc);
            }
            
            data[i*2] = buffer[0];
            data[i*2+1] = buffer[1];
        }
        
        Ok(())
    }

    /// Writes commands with arguments
    async fn write_command_with_args(&mut self, cmd: Command, data: &[u8]) -> Result<(), Error<E>> {
        const MAX_TX_BUFFER: usize = 14; //cmd (2 bytes) + max args (12 bytes)

        let mut transfer_buffer = [0; MAX_TX_BUFFER];

        let size = data.len();

        // 2 for command, size of transferred bytes and CRC per each two bytes.
        assert!(size < 2 + size + size / 2);
        let (command, delay) = cmd.as_tuple();

        transfer_buffer[0..2].copy_from_slice(&command.to_be_bytes());

        let mut i = 2;
        for chunk in data.chunks(2) {
            let end = i + 2;
            transfer_buffer[i..end].copy_from_slice(chunk);
            transfer_buffer[end] = crc8::calculate(chunk);
            i += 3;
        }

        self.i2c
            .write(self.address, &transfer_buffer[0..i])
            .await
            .map_err(Error::I2c)?;
        self.delay.delay_ms(delay).await;

        Ok(())
    }

    /// Writes commands without additional arguments.
    async fn write_command(&mut self, cmd: Command) -> Result<(), Error<E>> {
        let (command, delay) = cmd.as_tuple();
        
        // Manual implementation of write_command_u16 since we need async
        let bytes = command.to_be_bytes();
        self.i2c.write(self.address, &bytes).await.map_err(Error::I2c)?;
        self.delay.delay_ms(delay).await;
        Ok(())
    }

    /// Sensor self-test.
    ///
    /// Performs sensor self-test. This is intended for production line and testing and verification only and
    /// shouldn't be needed for normal use.
    pub async fn self_test(&mut self) -> Result<&mut Self, Error<E>> {
        const MEASURE_TEST_OK: u16 = 0xd400;
        let mut data = [0; 3];

        self.delayed_read_cmd(Command::MeasureTest, &mut data).await?;

        let result = u16::from_be_bytes([data[0], data[1]]);

        if result != MEASURE_TEST_OK {
            Err(Error::SelfTest)
        } else {
            Ok(self)
        }
    }

    /// Turn sensor heater off and places it in idle-mode.
    ///
    /// Stops running the measurements, places heater into idle by turning the heaters off.
    #[inline]
    pub async fn turn_heater_off(&mut self) -> Result<&Self, Error<E>> {
        self.write_command(Command::HeaterOff).await?;
        Ok(self)
    }

    /// Resets the sensor.
    ///
    /// Executes a reset on the device. The caller must wait 100ms before starting to use the device again.
    #[inline]
    pub async fn reset(&mut self) -> Result<&Self, Error<E>> {
        self.write_command(Command::SoftReset).await?;
        Ok(self)
    }

    /// Reads the voc index from the sensor.
    ///
    /// Reads VOC index. Driver is using Sensirion proprietary algortihm and it takes minimum
    /// 45 reads to start working. These reads should be made with 1Hz interval to keep the
    /// algoritm working.
    #[cfg(feature = "voc_index")]
    #[inline]
    pub async fn measure_voc_index(&mut self) -> Result<u16, Error<E>> {
        let raw = self.measure_raw_with_rht(50000, 25000).await?;

        Ok(self.voc.process(raw as i32) as u16)
    }

    /// Reads the voc index from the sensor with humidity and temperature compensation.
    ///
    /// Reads VOC index with humidity and temperature compensation. Both values us milli-notation where
    /// 25°C is equivalent of 25000 and 50% humidity equals 50000.
    ///
    /// Driver is using Sensirion proprietary algortihm and it takes minimum
    /// 45 reads to start working. These reads should be made with 1Hz interval to keep the
    /// algoritm working.
    #[cfg(feature = "voc_index")]
    #[inline]
    pub async fn measure_voc_index_with_rht(&mut self, humidity: u16, temperature: i16) -> Result<u16, Error<E>> {
        let raw = self.measure_raw_with_rht(humidity, temperature).await?;

        Ok(self.voc.process(raw as i32) as u16)
    }

    /// Reads the raw signal from the sensor.
    ///
    /// Raw signal without temperature and humidity compensation. This is not
    /// VOC index but needs to be processed through different algorithm for that.
    #[inline]
    pub async fn measure_raw(&mut self) -> Result<u16, Error<E>> {
        self.measure_raw_with_rht(50000, 25000).await
    }

    /// Reads the raw signal from the sensor.
    ///
    /// Raw signal with temperature and humidity compensation. This is not
    /// VOC index but needs to be processed through different algorithm for that.
    pub async fn measure_raw_with_rht(&mut self, humidity: u16, temperature: i16) -> Result<u16, Error<E>> {
        let mut data = [0; 3];

        let (hum_ticks, temp_ticks) = self.convert_rht(humidity as u32, temperature as i32);

        let mut params = [0u8; 4];
        params[0..2].copy_from_slice(&hum_ticks.to_be_bytes());
        params[2..4].copy_from_slice(&temp_ticks.to_be_bytes());

        self.write_command_with_args(Command::MeasurementRaw, &params).await?;
        self.read_words_with_crc(&mut data).await?;

        Ok(u16::from_be_bytes([data[0], data[1]]))
    }

    // Returns tick converted values
    fn convert_rht(&self, humidity: u32, temperature: i32) -> (u16, u16) {
        let mut temperature = temperature;
        let mut humidity = humidity;
        if humidity > 100000 {
            humidity = 100000;
        }

        temperature += self.temperature_offset as i32;

        temperature = temperature.clamp(-45000, 129760);

        /* humidity_sensor_format = humidity / 100000 * 65535;
         * 65535 / 100000 = 0.65535 -> 0.65535 * 2^5 = 20.9712 / 2^10 ~= 671
         */
        let humidity_sensor_format = ((humidity * 671) >> 10) as u16;

        /* temperature_sensor_format[1] = (temperature + 45000) / 175000 * 65535;
         * 65535 / 175000 ~= 0.375 -> 0.375 * 2^3 = 2.996 ~= 3
         */
        let temperature_sensor_format = (((temperature + 45000) * 3) >> 3) as u16;

        (humidity_sensor_format, temperature_sensor_format)
    }

    /// Sets the temperature offset.
    ///
    /// This command sets the temperature offset used for the compensation of subsequent RHT measurements.
    /// The parameter provides the temperature offset (in °C) with a scaling factor of 200, e.g., an output of +400 corresponds to +2.00 °C.
    #[inline]
    pub async fn set_temperature_offset(&mut self, offset: i16) -> Result<&mut Self, Error<E>> {
        self.temperature_offset += offset;
        Ok(self)
    }

    /// Gets the temperature offset
    ///
    /// Gets the temperature compensation offset issues to the device.
    pub async fn get_temperature_offset(&mut self) -> Result<i16, Error<E>> {
        Ok(self.temperature_offset)
    }

    /// Acquires the sensor serial number.
    ///
    /// Sensor serial number is only 48-bits long so the remaining 16-bits are zeros.
    pub async fn serial(&mut self) -> Result<u64, Error<E>> {
        let mut serial = [0; 9];

        self.delayed_read_cmd(Command::Serial, &mut serial).await?;

        let serial = (u64::from(serial[0]) << 40)
            | (u64::from(serial[1]) << 32)
            | (u64::from(serial[3]) << 24)
            | (u64::from(serial[4]) << 16)
            | (u64::from(serial[6]) << 8)
            | u64::from(serial[7]);
        Ok(serial)
    }
}