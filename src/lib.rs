//! A platform agnostic Rust driver for TMP451, based on the
//! [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.

#![no_std]
#![macro_use]
pub(crate) mod fmt;

mod error;
pub use error::{Error, Result};

#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("You should probably choose at least one of `sync` and `async` features.");

#[cfg(feature = "sync")]
use embedded_hal::i2c::ErrorType;
#[cfg(feature = "sync")]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::ErrorType as AsyncErrorType;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

/// TMP451 sensor's I2C address.
pub const DEFAULT_ADDRESS: u8 = 0b1001100; // This is I2C address 0x4C

const TMP451_PRODUCT_ID: u8 = 0x55;

/// ADC Conversion Rates.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum ConversionRate {
    Rate1_16Hz = 0,
    Rate1_8Hz = 1,
    Rate1_4Hz = 2,
    Rate1_2Hz = 3,
    Rate1Hz = 4,
    Rate2Hz = 5,
    Rate4Hz = 6,
    Rate8Hz = 7,
    Rate16Hz = 8,
    Rate32Hz = 9,
}

/// Registers of the TMP451 sensor.
#[cfg(any(feature = "async", feature = "sync"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Register {
    LocalTempMSB = 0x00,
    RemoteTempMSB = 0x01,
    Status = 0x02,
    Configuration = 0x03,
    ConversionRate = 0x04,
    // LocalTempHighLimitMSB = 0x05, //TODO: impl other functionalities
    // LocalTempLowLimitMSB = 0x06,
    // RemoteTempHighLimitMSB = 0x07,
    // RemoteTempLowLimitMSB = 0x08,
    // OneShotStart = 0x0F,
    RemoteTempLSB = 0x10,
    // RemoteTempOffsetMSB = 0x11,
    // RemoteTempOffsetLSB = 0x12,
    // RemoteTempHighLimitLSB = 0x13,
    // RemoteTempLowLimitLSB = 0x14,
    LocalTempLSB = 0x15,
    // RemoteTempThermLimitMSB = 0x19,
    // LocalTempThermLimitMSB = 0x20,
    // ThermHysteresisMSB = 0x21,
    // ConsecutiveAlert = 0x22,
    // NfactorCorrection = 0x23,
    // DigitalFilterControl = 0x24,
    ProductID = 0xFE,
}

#[cfg(any(feature = "async", feature = "sync"))]
impl From<Register> for u8 {
    fn from(r: Register) -> u8 {
        r as u8
    }
}

/// Device Satuts.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Status {
    pub busy: bool,
    pub local_temp_high_limit: bool,
    pub local_temp_low_limit: bool,
    pub remote_temp_high_limit: bool,
    pub remote_temp_low_limit: bool,
    pub open: bool,
    pub remote_therm_limit: bool,
    pub local_therm_limit: bool,
}

impl From<u8> for Status {
    fn from(s: u8) -> Self {
        Self {
            busy: s & 0x80 == 0x80,
            local_temp_high_limit: s & 0x40 == 0x40,
            local_temp_low_limit: s & 0x20 == 0x20,
            remote_temp_high_limit: s & 0x10 == 0x10,
            remote_temp_low_limit: s & 0x08 == 0x08,
            open: s & 0x04 == 0x04,
            remote_therm_limit: s & 0x02 == 0x02,
            local_therm_limit: s & 0x01 == 0x01,
        }
    }
}

// Type State Range
#[derive(Debug, Default)]
pub struct RangeStandard;
#[derive(Debug, Default)]
pub struct RangeExtended;

/// An TMP451 sensor on the I2C bus `I`.
///
/// The address of the sensor will be `DEFAULT_ADDRESS` from this package,
/// unless there is some kind of special address translating hardware in use.
#[maybe_async_cfg::maybe(
    sync(feature = "sync", self = "TMP451"),
    async(feature = "async", keep_self)
)]
pub struct AsyncTMP451<I, R> {
    i2c: I,
    address: u8,
    range: core::marker::PhantomData<R>,
}

#[maybe_async_cfg::maybe(
    sync(
        feature = "sync",
        self = "TMP451",
        idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
    ),
    async(feature = "async", keep_self)
)]
impl<I: AsyncI2c + AsyncErrorType, R> AsyncTMP451<I, R> {
    /// Check the TMP451 its Product ID.
    async fn check_id(&mut self) -> Result<&mut Self, I::Error> {
        trace!("check_id");
        match self.read_reg(Register::ProductID).await? {
            TMP451_PRODUCT_ID => Ok(self),
            _ => Err(Error::InvalidID),
        }
    }

    /// Get the curent status.
    pub async fn status(&mut self) -> Result<Status, I::Error> {
        trace!("status");
        Ok(self.read_reg(Register::Status).await?.into())
    }

    /// Get the current conversion rate in Hertz.
    pub async fn conversion_rate(&mut self) -> Result<ConversionRate, I::Error> {
        trace!("conversion_rate");
        let rate: ConversionRate = match self.read_reg(Register::ConversionRate).await? {
            0 => ConversionRate::Rate1_16Hz,
            1 => ConversionRate::Rate1_8Hz,
            2 => ConversionRate::Rate1_4Hz,
            3 => ConversionRate::Rate1_2Hz,
            4 => ConversionRate::Rate1Hz,
            5 => ConversionRate::Rate2Hz,
            6 => ConversionRate::Rate4Hz,
            7 => ConversionRate::Rate8Hz,
            8 => ConversionRate::Rate16Hz,
            9 => ConversionRate::Rate32Hz,
            _ => return Err(Error::InvalidValue),
        };
        debug!("conversion_rate={:?}", rate);
        Ok(rate)
    }

    /// Set the conversion rate.
    pub async fn set_conversion_rate(
        &mut self,
        rate: ConversionRate,
    ) -> Result<&mut Self, I::Error> {
        debug!("set_conversion_rate={:?}", rate);
        self.write_reg(Register::ConversionRate, rate as u8).await?;
        Ok(self)
    }
    /// read_reg read a register value.
    async fn read_reg<REG: Into<u8>>(&mut self, reg: REG) -> Result<u8, I::Error> {
        trace!("read_reg");
        let mut buf = [0x00];
        let reg = reg.into();
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .await
            .map_err(Error::I2c)?;
        debug!("R @0x{:x}={:x}", reg, buf[0]);
        Ok(buf[0])
    }

    /// write_reg blindly write a single register with a fixed value.
    async fn write_reg<REG: Into<u8>>(&mut self, reg: REG, value: u8) -> Result<(), I::Error> {
        trace!("write_reg");
        let reg = reg.into();
        debug!("W @0x{:x}={:x}", reg, value);
        self.i2c
            .write(self.address, &[reg, value])
            .await
            .map_err(Error::I2c)
    }

    /// update_reg first read the register value, apply a set mask, then a clear mask, and write the new value
    /// only if different from the initial value.
    async fn update_reg<REG: Into<u8> + Clone>(
        &mut self,
        reg: REG,
        mask_set: u8,
        mask_clear: u8,
    ) -> Result<(), I::Error> {
        trace!("update_reg");
        let current = self.read_reg(reg.clone()).await?;
        let updated = current | mask_set & !mask_clear;
        if current != updated {
            self.write_reg(reg, updated).await?;
        }
        Ok(())
    }

    /// Return the underlying I2C device
    pub fn release(self) -> I {
        self.i2c
    }

    /// Destroys this driver and releases the I2C bus `I`.
    pub fn destroy(self) -> Self {
        self
    }
}

#[maybe_async_cfg::maybe(
    sync(
        feature = "sync",
        self = "TMP451",
        idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
    ),
    async(feature = "async", keep_self)
)]
impl<I: AsyncI2c + AsyncErrorType> AsyncTMP451<I, RangeStandard> {
    /// Initializes the TMP451 driver.
    ///
    /// This consumes the I2C bus `I`. The address will almost always
    /// be `DEFAULT_ADDRESS` from this crate.
    pub async fn with_address(i2c: I, address: u8) -> Result<Self, I::Error> {
        let mut tmp451 = AsyncTMP451 {
            i2c,
            address,
            range: core::marker::PhantomData::<RangeStandard>,
        };
        trace!("new");
        tmp451.check_id().await?;
        Ok(tmp451)
    }
    pub async fn new(i2c: I) -> Result<Self, I::Error> {
        AsyncTMP451::with_address(i2c, DEFAULT_ADDRESS).await
    }

    pub async fn set_extended_range(mut self) -> Result<AsyncTMP451<I, RangeExtended>, I::Error> {
        trace!("set_extended_range");
        self.update_reg(Register::Configuration, 0b0000_0100, 0)
            .await?;
        Ok(AsyncTMP451 {
            i2c: self.i2c,
            address: self.address,
            range: core::marker::PhantomData::<RangeExtended>,
        })
    }

    fn precise(&self, msb: u8, lsb: u8) -> f32 {
        msb as f32 + (lsb >> 4) as f32 * 0.0625
    }

    /// Read the current local temperature value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn local_temp(&mut self) -> Result<u8, I::Error> {
        trace!("local_temp");
        self.read_reg(Register::LocalTempMSB).await
    }

    /// Read the current local temperature value in degree Celsius.
    ///
    /// Range is 0~127°C with 0.0625°C resolution
    pub async fn precise_local_temp(&mut self) -> Result<f32, I::Error> {
        trace!("local_temp");
        let msb = self.read_reg(Register::LocalTempMSB).await?;
        let lsb = self.read_reg(Register::LocalTempLSB).await?;
        Ok(self.precise(msb, lsb))
    }

    /// Read the current remote temperature value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn remote_temp(&mut self) -> Result<u8, I::Error> {
        trace!("remote_temp");
        self.read_reg(Register::RemoteTempMSB).await
    }

    /// Read the current remote temperature value in degree Celsius.
    ///
    /// Range is 0~127°C with 0.0625°C resolution
    pub async fn precise_remote_temp(&mut self) -> Result<f32, I::Error> {
        trace!("remote_temp");
        let msb = self.read_reg(Register::RemoteTempMSB).await?;
        let lsb = self.read_reg(Register::RemoteTempLSB).await?;
        Ok(self.precise(msb, lsb))
    }
}

#[maybe_async_cfg::maybe(
    sync(
        feature = "sync",
        self = "TMP451",
        idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
    ),
    async(feature = "async", keep_self)
)]
impl<I: AsyncI2c + AsyncErrorType> AsyncTMP451<I, RangeExtended> {
    pub async fn set_standard_range(mut self) -> Result<TMP451<I, RangeStandard>, I::Error> {
        trace!("set_extended_range");
        self.update_reg(Register::Configuration, 0, 0b0000_0100)
            .await?;
        Ok(TMP451 {
            i2c: self.i2c,
            address: self.address,
            range: core::marker::PhantomData::<RangeStandard>,
        })
    }

    fn precise(&self, msb: u8, lsb: u8) -> f32 {
        msb as f32 - 64.0 + (lsb >> 4) as f32 * 0.0625
    }

    /// Read the current local temperature value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn local_temp(&mut self) -> Result<i16, I::Error> {
        trace!("local_temp");
        Ok((self.read_reg(Register::LocalTempMSB).await? as i16) - 64)
    }

    /// Read the current local temperature value in degree Celsius.
    ///
    /// Range is -64~191°C with 0.0625°C resolution
    pub async fn precise_local_temp(&mut self) -> Result<f32, I::Error> {
        trace!("local_temp");
        let msb = self.read_reg(Register::LocalTempMSB).await?;
        let lsb = self.read_reg(Register::LocalTempLSB).await?;
        Ok(self.precise(msb, lsb))
    }

    /// Read the current remote temperature value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn remote_temp(&mut self) -> Result<i16, I::Error> {
        trace!("remote_temp");
        Ok((self.read_reg(Register::RemoteTempMSB).await? as i16) - 64)
    }

    /// Read the current remote temperature value in degree Celsius.
    ///
    /// Range is -64~191°C with 0.0625°C resolution
    pub async fn precise_remote_temp(&mut self) -> Result<f32, I::Error> {
        trace!("remote_temp");
        let msb = self.read_reg(Register::RemoteTempMSB).await?;
        let lsb = self.read_reg(Register::RemoteTempLSB).await?;
        Ok(self.precise(msb, lsb))
    }
}

#[cfg(test)]
mod test {
    // extern crate alloc;
    extern crate std;

    use super::*;
    use embedded_hal_mock::{common::Generic, eh1::i2c};
    use std::vec;

    #[test]
    fn new_tmp451() {
        let expectations = [i2c::Transaction::write_read(
            DEFAULT_ADDRESS,
            vec![Register::ProductID as u8],
            vec![TMP451_PRODUCT_ID],
        )];
        let mock = i2c::Mock::new(&expectations);
        let tmp451: TMP451<Generic<embedded_hal_mock::eh1::i2c::Transaction>, RangeStandard> =
            TMP451::new(mock).unwrap();

        let mut mock = tmp451.release();
        mock.done();
    }
}
