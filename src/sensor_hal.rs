#[cfg(feature = "embedded-sensors-hal")]
use embedded_sensors_hal::{sensor, temperature};
#[cfg(feature = "embedded-sensors-hal-async")]
use embedded_sensors_hal_async::{sensor, temperature};

use super::*;

#[cfg(all(
    feature = "embedded-sensors-hal",
    feature = "embedded-sensors-hal-async"
))]
compile_error!(
    "Only one of `embedded-sensors-hal` or `embedded-sensors-hal-async` must be enabled at a time."
);

#[cfg(any(
    feature = "embedded-sensors-hal",
    feature = "embedded-sensors-hal-async"
))]
impl<E: core::fmt::Debug> sensor::Error for Error<E> {
    fn kind(&self) -> sensor::ErrorKind {
        match *self {
            Error::I2c(_) => sensor::ErrorKind::Peripheral,
            Error::InvalidValue => sensor::ErrorKind::InvalidInput,
            _ => sensor::ErrorKind::Other,
        }
    }
}

#[maybe_async_cfg::maybe(
    sync(
        feature = "embedded-sensors-hal",
        self = "TMP451",
        idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
    ),
    async(feature = "embedded-sensors-hal-async", keep_self)
)]
impl<E: core::fmt::Debug, I: AsyncI2c<Error = E> + AsyncErrorType, R, C> sensor::ErrorType
    for AsyncTMP451<I, R, C>
{
    type Error = Error<E>;
}

macro_rules! impl_embedded_sensors_hal {
    ($range:ty, $itype:ty) => {
        #[maybe_async_cfg::maybe(
            sync(
                feature = "embedded-sensors-hal",
                self = "TMP451",
                idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
            ),
            async(feature = "embedded-sensors-hal-async", keep_self)
        )]
        impl<E: core::fmt::Debug, I: AsyncI2c<Error = E> + AsyncErrorType, C>
            temperature::TemperatureSensor for AsyncTMP451<I, $range, C>
        {
            async fn temperature(&mut self) -> Result<temperature::DegreesCelsius, I::Error> {
                #[cfg(feature = "embedded-sensors-use-remote")]
                {
                    self.precise_remote_temp().await
                }
                #[cfg(not(feature = "embedded-sensors-use-remote"))]
                {
                    self.precise_local_temp().await
                }
            }
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        impl<E: core::fmt::Debug, I: AsyncI2c<Error = E> + AsyncErrorType, C>
            temperature::TemperatureThresholdSet for AsyncTMP451<I, $range, C>
        {
            async fn set_temperature_threshold_low(
                &mut self,
                threshold: temperature::DegreesCelsius,
            ) -> Result<(), I::Error> {
                // Low threshold doesn't exist for THERM limits, so return error
                #[cfg(feature = "embedded-sensors-use-therm")]
                {
                    let _ = threshold; // Silence clippy warning
                    Err(Error::InvalidValue)
                }
                #[cfg(all(
                    not(feature = "embedded-sensors-use-therm"),
                    feature = "embedded-sensors-use-remote"
                ))]
                {
                    self.set_precise_remote_temp_low_limit(threshold).await
                }
                #[cfg(all(
                    not(feature = "embedded-sensors-use-therm"),
                    not(feature = "embedded-sensors-use-remote")
                ))]
                {
                    self.set_local_temp_low_limit(threshold as $itype).await
                }
            }

            async fn set_temperature_threshold_high(
                &mut self,
                threshold: temperature::DegreesCelsius,
            ) -> Result<(), I::Error> {
                #[cfg(all(
                    not(feature = "embedded-sensors-use-remote"),
                    not(feature = "embedded-sensors-use-therm")
                ))]
                {
                    self.set_local_temp_high_limit(threshold as $itype).await
                }
                #[cfg(all(
                    not(feature = "embedded-sensors-use-remote"),
                    feature = "embedded-sensors-use-therm"
                ))]
                {
                    self.set_local_temp_therm_limit(threshold as $itype).await
                }
                #[cfg(all(
                    feature = "embedded-sensors-use-remote",
                    not(feature = "embedded-sensors-use-therm")
                ))]
                {
                    self.set_precise_remote_temp_high_limit(threshold).await
                }
                #[cfg(all(
                    feature = "embedded-sensors-use-remote",
                    feature = "embedded-sensors-use-therm"
                ))]
                {
                    self.set_remote_temp_therm_limit(threshold as $itype).await
                }
            }
        }

        #[cfg(all(
            feature = "embedded-sensors-hal-async",
            feature = "embedded-sensors-use-therm"
        ))]
        impl<E: core::fmt::Debug, I: AsyncI2c<Error = E> + AsyncErrorType, C>
            temperature::TemperatureHysteresis for AsyncTMP451<I, $range, C>
        {
            async fn set_temperature_threshold_hysteresis(
                &mut self,
                hysteresis: temperature::DegreesCelsius,
            ) -> Result<(), I::Error> {
                self.set_hysteresis(hysteresis as u8).await
            }
        }
    };
}

impl_embedded_sensors_hal!(RangeStandard, u8);
impl_embedded_sensors_hal!(RangeExtended, i16);
