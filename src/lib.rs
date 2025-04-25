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
use embedded_hal::delay::DelayNs;
#[cfg(feature = "sync")]
use embedded_hal::i2c::ErrorType;
#[cfg(feature = "sync")]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
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
    ConfigurationRead = 0x03,
    ConversionRateRead = 0x04,
    LocalTempHighLimitMSBRead = 0x05,
    LocalTempLowLimitMSBRead = 0x06,
    RemoteTempHighLimitMSBRead = 0x07,
    RemoteTempLowLimitMSBRead = 0x08,
    ConfigurationWrite = 0x09,
    ConversionRateWrite = 0x0A,
    LocalTempHighLimitMSBWrite = 0x0B,
    LocalTempLowLimitMSBWrite = 0x0C,
    RemoteTempHighLimitMSBWrite = 0x0D,
    RemoteTempLowLimitMSBWrite = 0x0E,
    OneShotStart = 0x0F,
    RemoteTempLSB = 0x10,
    RemoteTempOffsetMSB = 0x11,
    RemoteTempOffsetLSB = 0x12,
    RemoteTempHighLimitLSB = 0x13,
    RemoteTempLowLimitLSB = 0x14,
    LocalTempLSB = 0x15,
    RemoteTempThermLimitMSB = 0x19,
    LocalTempThermLimitMSB = 0x20,
    ThermHysteresisMSB = 0x21,
    ConsecutiveAlert = 0x22,
    NfactorCorrection = 0x23,
    DigitalFilterControl = 0x24,
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

/// Device Configuration.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Config {
    pub alert_mask: bool,
    pub sd_mode: bool,
    pub therm2_mode: bool,
    pub ext_range: bool,
}

impl From<u8> for Config {
    fn from(s: u8) -> Self {
        Self {
            alert_mask: s & 0x80 == 0x80,
            sd_mode: s & 0x40 == 0x40,
            therm2_mode: s & 0x20 == 0x20,
            ext_range: s & 0x04 == 0x04,
        }
    }
}

/// Pin 6 Mode.
#[derive(Debug, Clone, Copy)]
pub enum Pin6Mode {
    /// Pin 6 is configured as ALERT output.
    Alert,
    /// Pin 6 is configured as second THERM output.
    Therm2,
}

/// Number of consecutive out-of-limit measurements before ALERT pin is activated.
#[derive(Debug, Clone, Copy)]
pub enum ConsecutiveAlert {
    /// One.
    One,
    /// Two.
    Two,
    /// Three.
    Three,
    /// Four.
    Four,
}

/// Digital filtering control mode.
#[derive(Debug, Clone, Copy)]
pub enum DigitalFilter {
    /// Averaging is off.
    Off,
    /// Average of 4 measurements.
    Average4,
    /// Average of 8 measurements.
    Average8,
}

// Type State Range
#[derive(Debug, Default)]
pub struct RangeStandard;
#[derive(Debug, Default)]
pub struct RangeExtended;

// Type State Conversion Mode
#[derive(Debug, Default)]
pub struct Shutdown;
#[derive(Debug, Default)]
pub struct Continuous;

/// An TMP451 sensor on the I2C bus `I`.
///
/// The address of the sensor will be `DEFAULT_ADDRESS` from this package,
/// unless there is some kind of special address translating hardware in use.
#[maybe_async_cfg::maybe(
    sync(feature = "sync", self = "TMP451"),
    async(feature = "async", keep_self)
)]
pub struct AsyncTMP451<I, R, C> {
    i2c: I,
    address: u8,
    range: core::marker::PhantomData<R>,
    cm: core::marker::PhantomData<C>,
    cr: ConversionRate,
}

#[maybe_async_cfg::maybe(
    sync(
        feature = "sync",
        self = "TMP451",
        idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
    ),
    async(feature = "async", keep_self)
)]
impl<I: AsyncI2c + AsyncErrorType, R, C> AsyncTMP451<I, R, C> {
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

    /// Get the curent configuration.
    pub async fn config(&mut self) -> Result<Config, I::Error> {
        trace!("config");
        Ok(self.read_reg(Register::ConfigurationRead).await?.into())
    }

    /// Get the current conversion rate in Hertz.
    pub async fn conversion_rate(&mut self) -> Result<ConversionRate, I::Error> {
        trace!("conversion_rate");
        let rate: ConversionRate = match self.read_reg(Register::ConversionRateRead).await? {
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
        self.write_reg(Register::ConversionRateWrite, rate as u8)
            .await?;
        self.cr = rate;
        Ok(self)
    }

    /// Enable ALERT output.
    pub async fn enable_alert(&mut self) -> Result<(), I::Error> {
        trace!("enable_alert");
        self.update_reg(
            Register::ConfigurationRead,
            Register::ConfigurationWrite,
            0,
            0b1000_0000,
        )
        .await
    }

    /// Disable ALERT output.
    pub async fn disable_alert(&mut self) -> Result<(), I::Error> {
        trace!("disable_alert");
        self.update_reg(
            Register::ConfigurationRead,
            Register::ConfigurationWrite,
            0b1000_0000,
            0,
        )
        .await
    }

    /// Enable SMBus time-out.
    pub async fn enable_smb_timeout(&mut self) -> Result<(), I::Error> {
        trace!("enable_smb_timeout");
        self.update_reg(
            Register::ConsecutiveAlert,
            Register::ConsecutiveAlert,
            0b1000_0000,
            0,
        )
        .await
    }

    /// Disable SMBus time-out.
    pub async fn disable_smb_timeout(&mut self) -> Result<(), I::Error> {
        trace!("disable_smb_timeout");
        self.update_reg(
            Register::ConsecutiveAlert,
            Register::ConsecutiveAlert,
            0,
            0b1000_0000,
        )
        .await
    }

    /// Returns true if SMBus time-out is enabled.
    pub async fn smb_timeout_enabled(&mut self) -> Result<bool, I::Error> {
        trace!("smb_timeout_enabled");
        let reg = self.read_reg(Register::ConsecutiveAlert).await?;
        Ok(reg & 0x80 == 0x80)
    }

    /// Set Pin 6 as either ALERT or THERM2 output.
    pub async fn set_pin6_mode(&mut self, mode: Pin6Mode) -> Result<(), I::Error> {
        trace!("set_pin6_mode={:?}", mode);
        match mode {
            Pin6Mode::Alert => {
                self.update_reg(
                    Register::ConfigurationRead,
                    Register::ConfigurationWrite,
                    0,
                    0b0010_0000,
                )
                .await
            }
            Pin6Mode::Therm2 => {
                self.update_reg(
                    Register::ConfigurationRead,
                    Register::ConfigurationWrite,
                    0b0010_0000,
                    0,
                )
                .await
            }
        }
    }

    /// Set the number of consecutive out-of-limit measurements before an ALERT is triggered.
    pub async fn set_consecutive_alert(
        &mut self,
        consecutive: ConsecutiveAlert,
    ) -> Result<(), I::Error> {
        trace!("set_consecutive_alert={:?}", consecutive);
        let (setmask, clearmask) = match consecutive {
            ConsecutiveAlert::One => (0b0000_0000, 0b0000_1110),
            ConsecutiveAlert::Two => (0b0000_0010, 0b0000_1100),
            ConsecutiveAlert::Three => (0b0000_0110, 0b0000_1000),
            ConsecutiveAlert::Four => (0b0000_1110, 0b0000_0000),
        };

        self.update_reg(
            Register::ConsecutiveAlert,
            Register::ConsecutiveAlert,
            setmask,
            clearmask,
        )
        .await
    }

    /// Get the current number of consecutive out-of-limit measurements before an ALERT is triggered.
    pub async fn consecutive_alert(&mut self) -> Result<ConsecutiveAlert, I::Error> {
        trace!("consecutive_alert");
        let reg = self.read_reg(Register::ConsecutiveAlert).await?;
        let conal = (reg & 0b0000_1110) >> 1;
        match conal {
            0 => Ok(ConsecutiveAlert::One),
            1 => Ok(ConsecutiveAlert::Two),
            3 => Ok(ConsecutiveAlert::Three),
            7 => Ok(ConsecutiveAlert::Four),
            _ => Err(Error::InvalidValue),
        }
    }

    /// Set the THERM hysteresis value.
    pub async fn set_hysteresis(&mut self, hysteresis: u8) -> Result<(), I::Error> {
        trace!("set_hysteresis={:?}", hysteresis);
        self.write_reg(Register::ThermHysteresisMSB, hysteresis)
            .await
    }

    /// Get the THERM hysteresis value.
    pub async fn hysteresis(&mut self) -> Result<u8, I::Error> {
        trace!("hysteresis");
        self.read_reg(Register::ThermHysteresisMSB).await
    }

    /// Set the η-factor correction value.
    pub async fn set_n_factor(&mut self, neff: f32) -> Result<(), I::Error> {
        trace!("set_n_factor={:?}", neff);

        let neff = neff.clamp(0.950198, 1.073837);
        let nadjust = (((1.008 * 2088.0) / neff) - 2088.0) as i8;

        // Reinterpret the bits of the resulting i8 as a u8 in a safe way
        let nadjust = nadjust.to_le_bytes()[0];

        self.write_reg(Register::NfactorCorrection, nadjust).await
    }

    /// Get the η-factor correction value.
    pub async fn n_factor(&mut self) -> Result<f32, I::Error> {
        trace!("n_factor");

        let nadjust = self.read_reg(Register::NfactorCorrection).await?;

        // Reinterpret the bits of the raw u8 as an i8 in a safe way
        let nadjust = i8::from_le_bytes([nadjust]) as f32;

        let neff = (1.008 * 2088.0) / (2088.0 + nadjust);

        Ok(neff)
    }

    /// Set digital filter control mode.
    pub async fn set_digital_filter(&mut self, filter: DigitalFilter) -> Result<(), I::Error> {
        trace!("set_digital_filter={:?}", filter);

        let reg = match filter {
            DigitalFilter::Off => 0b0000_0000,
            DigitalFilter::Average4 => 0b0000_0001,
            DigitalFilter::Average8 => 0b0000_0010,
        };

        self.write_reg(Register::DigitalFilterControl, reg).await
    }

    /// Get the digital filter control mode.
    pub async fn digital_filter(&mut self) -> Result<DigitalFilter, I::Error> {
        trace!("digital_filter");

        let df = self.read_reg(Register::DigitalFilterControl).await? & 0b0000_0011;
        match df {
            0 => Ok(DigitalFilter::Off),
            1 => Ok(DigitalFilter::Average4),
            2 => Ok(DigitalFilter::Average8),
            _ => Err(Error::InvalidValue),
        }
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
        read_reg: REG,
        write_reg: REG,
        mask_set: u8,
        mask_clear: u8,
    ) -> Result<(), I::Error> {
        trace!("update_reg");
        let current = self.read_reg(read_reg.clone()).await?;
        let updated = (current | mask_set) & !mask_clear;
        if current != updated {
            self.write_reg(write_reg, updated).await?;
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
impl<I: AsyncI2c + AsyncErrorType, C> AsyncTMP451<I, RangeStandard, C> {
    pub async fn set_extended_range(
        mut self,
    ) -> Result<AsyncTMP451<I, RangeExtended, C>, I::Error> {
        trace!("set_extended_range");

        self.update_reg(
            Register::ConfigurationRead,
            Register::ConfigurationWrite,
            0b0000_0100,
            0,
        )
        .await?;
        Ok(AsyncTMP451 {
            i2c: self.i2c,
            address: self.address,
            range: core::marker::PhantomData::<RangeExtended>,
            cm: core::marker::PhantomData::<C>,
            cr: self.cr,
        })
    }

    fn precise(&self, msb: u8, lsb: u8) -> f32 {
        msb as f32 + (lsb >> 4) as f32 * 0.0625
    }

    fn precise_to_bytes(&self, val: f32) -> (u8, u8) {
        let msb = val as u8;
        let lsb = (((val - msb as f32) / 0.0625) as u8) << 4;
        (msb, lsb)
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

    /// Set the local temperature high limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn set_local_temp_high_limit(&mut self, limit: u8) -> Result<(), I::Error> {
        trace!("set_local_temp_high_limit={:?}", limit);
        let limit = limit.clamp(0, 127);
        self.write_reg(Register::LocalTempHighLimitMSBWrite, limit)
            .await
    }

    /// Get the local temperature high limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn local_temp_high_limit(&mut self) -> Result<u8, I::Error> {
        trace!("local_temp_high_limit");
        self.read_reg(Register::LocalTempHighLimitMSBRead).await
    }

    /// Set the local temperature low limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn set_local_temp_low_limit(&mut self, limit: u8) -> Result<(), I::Error> {
        trace!("set_local_temp_low_limit={:?}", limit);
        let limit = limit.clamp(0, 127);
        self.write_reg(Register::LocalTempLowLimitMSBWrite, limit)
            .await
    }

    /// Get the local temperature low limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn local_temp_low_limit(&mut self) -> Result<u8, I::Error> {
        trace!("local_temp_low_limit");
        self.read_reg(Register::LocalTempLowLimitMSBRead).await
    }

    /// Set the remote temperature high limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn set_remote_temp_high_limit(&mut self, limit: u8) -> Result<(), I::Error> {
        trace!("set_remote_temp_high_limit={:?}", limit);
        let limit = limit.clamp(0, 127);
        self.write_reg(Register::RemoteTempHighLimitMSBWrite, limit)
            .await
    }

    /// Get the remote temperature high limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn remote_temp_high_limit(&mut self) -> Result<u8, I::Error> {
        trace!("remote_temp_high_limit");
        self.read_reg(Register::RemoteTempHighLimitMSBRead).await
    }

    /// Set the remote temperature low limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn set_remote_temp_low_limit(&mut self, limit: u8) -> Result<(), I::Error> {
        trace!("set_remote_temp_low_limit={:?}", limit);
        let limit = limit.clamp(0, 127);
        self.write_reg(Register::RemoteTempLowLimitMSBWrite, limit)
            .await
    }

    /// Get the remote temperature low limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn remote_temp_low_limit(&mut self) -> Result<u8, I::Error> {
        trace!("remote_temp_low_limit");
        self.read_reg(Register::RemoteTempLowLimitMSBRead).await
    }

    /// Set the remote temperature high limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 0.0625°C resolution
    pub async fn set_precise_remote_temp_high_limit(&mut self, limit: f32) -> Result<(), I::Error> {
        trace!("set_precise_remote_temp_high_limit={:?}", limit);
        let limit = limit.clamp(0.0, 127.0);
        let (msb, lsb) = self.precise_to_bytes(limit);
        self.write_reg(Register::RemoteTempHighLimitMSBWrite, msb)
            .await?;
        self.write_reg(Register::RemoteTempHighLimitLSB, lsb).await
    }

    /// Get the remote temperature high limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 0.0625°C resolution
    pub async fn precise_remote_temp_high_limit(&mut self) -> Result<f32, I::Error> {
        trace!("precise_remote_temp_high_limit");
        let msb = self.read_reg(Register::RemoteTempHighLimitMSBRead).await?;
        let lsb = self.read_reg(Register::RemoteTempHighLimitLSB).await?;
        Ok(self.precise(msb, lsb))
    }

    /// Set the remote temperature low limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 0.0625°C resolution
    pub async fn set_precise_remote_temp_low_limit(&mut self, limit: f32) -> Result<(), I::Error> {
        trace!("set_precise_remote_temp_low_limit={:?}", limit);
        let limit = limit.clamp(0.0, 127.0);
        let (msb, lsb) = self.precise_to_bytes(limit);
        self.write_reg(Register::RemoteTempLowLimitMSBWrite, msb)
            .await?;
        self.write_reg(Register::RemoteTempLowLimitLSB, lsb).await
    }

    /// Get the remote temperature low limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 0.0625°C resolution
    pub async fn precise_remote_temp_low_limit(&mut self) -> Result<f32, I::Error> {
        trace!("precise_remote_temp_low_limit");
        let msb = self.read_reg(Register::RemoteTempLowLimitMSBRead).await?;
        let lsb = self.read_reg(Register::RemoteTempLowLimitLSB).await?;
        Ok(self.precise(msb, lsb))
    }

    /// Set the offset applied to remote temperature readings in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn set_remote_temp_offset(&mut self, offset: u8) -> Result<(), I::Error> {
        trace!("set_remote_temp_offset={:?}", offset);
        let offset = offset.clamp(0, 127);
        self.write_reg(Register::RemoteTempOffsetMSB, offset).await
    }

    /// Get the offset applied to remote temperature readings in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn remote_temp_offset(&mut self) -> Result<u8, I::Error> {
        trace!("remote_temp_offset");
        self.read_reg(Register::RemoteTempOffsetMSB).await
    }

    /// Set the offset applied to remote temperature readings in degree Celsius.
    ///
    /// Range is 0~127°C with 0.0625°C resolution
    pub async fn set_precise_remote_temp_offset(&mut self, offset: f32) -> Result<(), I::Error> {
        trace!("set_precise_remote_temp_offset={:?}", offset);
        let offset = offset.clamp(0.0, 127.0);
        let (msb, lsb) = self.precise_to_bytes(offset);
        self.write_reg(Register::RemoteTempOffsetMSB, msb).await?;
        self.write_reg(Register::RemoteTempOffsetLSB, lsb).await
    }

    /// Get the offset applied to remote temperature readings in degree Celsius.
    ///
    /// Range is 0~127°C with 0.0625°C resolution
    pub async fn precise_remote_temp_offset(&mut self) -> Result<f32, I::Error> {
        trace!("precise_remote_temp_offset");
        let msb = self.read_reg(Register::RemoteTempOffsetMSB).await?;
        let lsb = self.read_reg(Register::RemoteTempOffsetLSB).await?;
        Ok(self.precise(msb, lsb))
    }

    /// Set the local temperature THERM limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn set_local_temp_therm_limit(&mut self, limit: u8) -> Result<(), I::Error> {
        trace!("set_local_temp_therm_limit={:?}", limit);
        let limit = limit.clamp(0, 127);
        self.write_reg(Register::LocalTempThermLimitMSB, limit)
            .await
    }

    /// Get the local temperature THERM limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn local_temp_therm_limit(&mut self) -> Result<u8, I::Error> {
        trace!("local_temp_therm_limit");
        self.read_reg(Register::LocalTempThermLimitMSB).await
    }

    /// Set the remote temperature THERM limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn set_remote_temp_therm_limit(&mut self, limit: u8) -> Result<(), I::Error> {
        trace!("set_remote_temp_therm_limit={:?}", limit);
        let limit = limit.clamp(0, 127);
        self.write_reg(Register::RemoteTempThermLimitMSB, limit)
            .await
    }

    /// Get the remote temperature THERM limit value in degree Celsius.
    ///
    /// Range is 0~127°C with 1°C resolution
    pub async fn remote_temp_therm_limit(&mut self) -> Result<u8, I::Error> {
        trace!("remote_temp_therm_limit");
        self.read_reg(Register::RemoteTempThermLimitMSB).await
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
impl<I: AsyncI2c + AsyncErrorType, C> AsyncTMP451<I, RangeExtended, C> {
    pub async fn set_standard_range(
        mut self,
    ) -> Result<AsyncTMP451<I, RangeStandard, C>, I::Error> {
        trace!("set_extended_range");
        self.update_reg(
            Register::ConfigurationRead,
            Register::ConfigurationWrite,
            0,
            0b0000_0100,
        )
        .await?;

        Ok(AsyncTMP451 {
            i2c: self.i2c,
            address: self.address,
            range: core::marker::PhantomData::<RangeStandard>,
            cm: core::marker::PhantomData::<C>,
            cr: self.cr,
        })
    }

    fn precise(&self, msb: u8, lsb: u8) -> f32 {
        msb as f32 - 64.0 + (lsb >> 4) as f32 * 0.0625
    }

    fn precise_to_bytes(&self, val: f32) -> (u8, u8) {
        let val = val + 64.0;
        let msb = val as u8;
        let lsb = (((val - msb as f32) / 0.0625) as u8) << 4;
        (msb, lsb)
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

    /// Set the local temperature high limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn set_local_temp_high_limit(&mut self, limit: i16) -> Result<(), I::Error> {
        trace!("set_local_temp_high_limit={:?}", limit);
        let limit = (limit.clamp(-64, 191) + 64) as u8;
        self.write_reg(Register::LocalTempHighLimitMSBWrite, limit)
            .await
    }

    /// Get the local temperature high limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn local_temp_high_limit(&mut self) -> Result<i16, I::Error> {
        trace!("local_temp_high_limit");
        Ok(self.read_reg(Register::LocalTempHighLimitMSBRead).await? as i16 - 64)
    }

    /// Set the local temperature low limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn set_local_temp_low_limit(&mut self, limit: i16) -> Result<(), I::Error> {
        trace!("set_local_temp_low_limit={:?}", limit);
        let limit = (limit.clamp(-64, 191) + 64) as u8;
        self.write_reg(Register::LocalTempLowLimitMSBWrite, limit)
            .await
    }

    /// Get the local temperature low limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn local_temp_low_limit(&mut self) -> Result<i16, I::Error> {
        trace!("local_temp_low_limit");
        Ok(self.read_reg(Register::LocalTempLowLimitMSBRead).await? as i16 - 64)
    }

    /// Set the remote temperature high limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn set_remote_temp_high_limit(&mut self, limit: i16) -> Result<(), I::Error> {
        trace!("set_remote_temp_high_limit={:?}", limit);
        let limit = (limit.clamp(-64, 191) + 64) as u8;
        self.write_reg(Register::RemoteTempHighLimitMSBWrite, limit)
            .await
    }

    /// Get the remote temperature high limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn remote_temp_high_limit(&mut self) -> Result<i16, I::Error> {
        trace!("remote_temp_high_limit");
        Ok(self.read_reg(Register::RemoteTempHighLimitMSBRead).await? as i16 - 64)
    }

    /// Set the remote temperature low limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn set_remote_temp_low_limit(&mut self, limit: i16) -> Result<(), I::Error> {
        trace!("set_remote_temp_low_limit={:?}", limit);
        let limit = (limit.clamp(-64, 191) + 64) as u8;
        self.write_reg(Register::RemoteTempLowLimitMSBWrite, limit)
            .await
    }

    /// Get the remote temperature low limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn remote_temp_low_limit(&mut self) -> Result<i16, I::Error> {
        trace!("remote_temp_low_limit");
        Ok(self.read_reg(Register::RemoteTempLowLimitMSBRead).await? as i16 - 64)
    }

    /// Set the remote temperature high limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 0.0625°C resolution
    pub async fn set_precise_remote_temp_high_limit(&mut self, limit: f32) -> Result<(), I::Error> {
        trace!("set_precise_remote_temp_high_limit={:?}", limit);
        let limit = limit.clamp(-64.0, 191.0);
        let (msb, lsb) = self.precise_to_bytes(limit);
        self.write_reg(Register::RemoteTempHighLimitMSBWrite, msb)
            .await?;
        self.write_reg(Register::RemoteTempHighLimitLSB, lsb).await
    }

    /// Get the remote temperature high limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 0.0625°C resolution
    pub async fn precise_remote_temp_high_limit(&mut self) -> Result<f32, I::Error> {
        trace!("precise_remote_temp_high_limit");
        let msb = self.read_reg(Register::RemoteTempHighLimitMSBRead).await?;
        let lsb = self.read_reg(Register::RemoteTempHighLimitLSB).await?;
        Ok(self.precise(msb, lsb))
    }

    /// Set the remote temperature low limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 0.0625°C resolution
    pub async fn set_precise_remote_temp_low_limit(&mut self, limit: f32) -> Result<(), I::Error> {
        trace!("set_precise_remote_temp_low_limit={:?}", limit);
        let limit = limit.clamp(-64.0, 191.0);
        let (msb, lsb) = self.precise_to_bytes(limit);
        self.write_reg(Register::RemoteTempLowLimitMSBWrite, msb)
            .await?;
        self.write_reg(Register::RemoteTempLowLimitLSB, lsb).await
    }

    /// Get the remote temperature low limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 0.0625°C resolution
    pub async fn precise_remote_temp_low_limit(&mut self) -> Result<f32, I::Error> {
        trace!("precise_remote_temp_low_limit");
        let msb = self.read_reg(Register::RemoteTempLowLimitMSBRead).await?;
        let lsb = self.read_reg(Register::RemoteTempLowLimitLSB).await?;
        Ok(self.precise(msb, lsb))
    }

    /// Set the offset applied to remote temperature readings in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn set_remote_temp_offset(&mut self, offset: i16) -> Result<(), I::Error> {
        trace!("set_remote_temp_offset={:?}", offset);
        let offset = (offset.clamp(-64, 191) + 64) as u8;
        self.write_reg(Register::RemoteTempOffsetMSB, offset).await
    }

    /// Get the offset applied to remote temperature readings in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn remote_temp_offset(&mut self) -> Result<i16, I::Error> {
        trace!("remote_temp_offset");
        Ok(self.read_reg(Register::RemoteTempOffsetMSB).await? as i16 - 64)
    }

    /// Set the offset applied to remote temperature readings in degree Celsius.
    ///
    /// Range is -64~191°C with 0.0625°C resolution
    pub async fn set_precise_remote_temp_offset(&mut self, offset: f32) -> Result<(), I::Error> {
        trace!("set_precise_remote_temp_offset={:?}", offset);
        let offset = offset.clamp(-64.0, 191.0);
        let (msb, lsb) = self.precise_to_bytes(offset);
        self.write_reg(Register::RemoteTempOffsetMSB, msb).await?;
        self.write_reg(Register::RemoteTempOffsetLSB, lsb).await
    }

    /// Get the offset applied to remote temperature readings in degree Celsius.
    ///
    /// Range is -64~191°C with 0.0625°C resolution
    pub async fn precise_remote_temp_offset(&mut self) -> Result<f32, I::Error> {
        trace!("precise_remote_temp_offset");
        let msb = self.read_reg(Register::RemoteTempOffsetMSB).await?;
        let lsb = self.read_reg(Register::RemoteTempOffsetLSB).await?;
        Ok(self.precise(msb, lsb))
    }

    /// Set the local temperature THERM limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn set_local_temp_therm_limit(&mut self, limit: i16) -> Result<(), I::Error> {
        trace!("set_local_temp_therm_limit={:?}", limit);
        let limit = (limit.clamp(-64, 191) + 64) as u8;
        self.write_reg(Register::LocalTempThermLimitMSB, limit)
            .await
    }

    /// Get the local temperature THERM limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn local_temp_therm_limit(&mut self) -> Result<i16, I::Error> {
        trace!("local_temp_therm_limit");
        Ok(self.read_reg(Register::LocalTempThermLimitMSB).await? as i16 - 64)
    }

    /// Set the remote temperature THERM limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn set_remote_temp_therm_limit(&mut self, limit: i16) -> Result<(), I::Error> {
        trace!("set_remote_temp_therm_limit={:?}", limit);
        let limit = (limit.clamp(-64, 191) + 64) as u8;
        self.write_reg(Register::RemoteTempThermLimitMSB, limit)
            .await
    }

    /// Get the remote temperature THERM limit value in degree Celsius.
    ///
    /// Range is -64~191°C with 1°C resolution
    pub async fn remote_temp_therm_limit(&mut self) -> Result<i16, I::Error> {
        trace!("remote_temp_therm_limit");
        Ok(self.read_reg(Register::RemoteTempThermLimitMSB).await? as i16 - 64)
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
impl<I: AsyncI2c + AsyncErrorType, R> AsyncTMP451<I, R, Continuous> {
    /// Puts the device into shutdown mode.
    ///
    /// Conversion will no longer occur automatically.
    pub async fn set_shutdown(mut self) -> Result<AsyncTMP451<I, R, Shutdown>, I::Error> {
        trace!("set_shutdown");
        self.update_reg(
            Register::ConfigurationRead,
            Register::ConfigurationWrite,
            0b0100_0000,
            0,
        )
        .await?;

        Ok(AsyncTMP451 {
            i2c: self.i2c,
            address: self.address,
            range: core::marker::PhantomData::<R>,
            cm: core::marker::PhantomData::<Shutdown>,
            cr: self.cr,
        })
    }
}

#[maybe_async_cfg::maybe(
    sync(
        feature = "sync",
        self = "TMP451",
        idents(
            AsyncI2c(sync = "I2c"),
            AsyncDelayNs(sync = "DelayNs"),
            AsyncErrorType(sync = "ErrorType")
        )
    ),
    async(feature = "async", keep_self)
)]
impl<I: AsyncI2c + AsyncErrorType, R> AsyncTMP451<I, R, Shutdown> {
    /// Puts the device into continuous conversion mode.
    pub async fn set_continuous(mut self) -> Result<AsyncTMP451<I, R, Continuous>, I::Error> {
        trace!("set_continuous");
        self.update_reg(
            Register::ConfigurationRead,
            Register::ConfigurationWrite,
            0,
            0b0100_0000,
        )
        .await?;

        Ok(AsyncTMP451 {
            i2c: self.i2c,
            address: self.address,
            range: core::marker::PhantomData::<R>,
            cm: core::marker::PhantomData::<Continuous>,
            cr: self.cr,
        })
    }

    /// Begins a one-shot conversion, and waits for the conversion to complete.
    pub async fn wait_one_shot<DELAY: AsyncDelayNs>(
        &mut self,
        mut delay: DELAY,
    ) -> Result<(), I::Error> {
        // Can write any arbitrary value to begin the conversion
        self.write_reg(Register::OneShotStart, 0).await?;

        // Wait for conversion to finish based on current conversion rate
        let ns = match self.cr {
            ConversionRate::Rate1_16Hz => 16_000_000,
            ConversionRate::Rate1_8Hz => 8_000_000,
            ConversionRate::Rate1_4Hz => 4_000_000,
            ConversionRate::Rate1_2Hz => 2_000_000,
            ConversionRate::Rate1Hz => 1_000_000,
            ConversionRate::Rate2Hz => 500_000,
            ConversionRate::Rate4Hz => 250_000,
            ConversionRate::Rate8Hz => 125_000,
            ConversionRate::Rate16Hz => 62_500,
            ConversionRate::Rate32Hz => 31_250,
        };
        delay.delay_ns(ns).await;

        Ok(())
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
impl<I: AsyncI2c + AsyncErrorType> AsyncTMP451<I, RangeStandard, Continuous> {
    /// Initializes the TMP451 driver.
    ///
    /// This consumes the I2C bus `I`. The address will almost always
    /// be `DEFAULT_ADDRESS` from this crate.
    pub async fn with_address(i2c: I, address: u8) -> Result<Self, I::Error> {
        let mut tmp451 = AsyncTMP451 {
            i2c,
            address,
            range: core::marker::PhantomData::<RangeStandard>,
            cm: core::marker::PhantomData::<Continuous>,
            cr: ConversionRate::Rate16Hz,
        };
        trace!("new");
        tmp451.check_id().await?;
        Ok(tmp451)
    }

    pub async fn new(i2c: I) -> Result<Self, I::Error> {
        AsyncTMP451::with_address(i2c, DEFAULT_ADDRESS).await
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
        let tmp451: TMP451<
            Generic<embedded_hal_mock::eh1::i2c::Transaction>,
            RangeStandard,
            Continuous,
        > = TMP451::new(mock).unwrap();

        let mut mock = tmp451.release();
        mock.done();
    }
}
