use embedded_hal::blocking::delay::DelayMs;
use heapless::Vec;

/// embedded_radio traits, to provide implementations of various radio drivers compatible with embedded_hal.
/// This mirrors the design of the std::sync::mpsc APIs, and can create mock implementations to support unit testing.
/// This trait does not cover configuration, because of lack of knowledge on the designer's part. PRs welcome if
/// a device agnostic way can be found to do that.
pub trait EmbeddedRadio {
    type Error;

    /// Attempts to send a value on this channel. Unsuccessful sends can result from hardware errors.
    fn transmit_payload(&mut self, payload: &[u8]) -> Result<(), Self::Error>;
    /// Blocks until the payload has been sent from the transmitter. Unsuccessful sends can result from hardware errors.
    fn transmit_payload_busy(&mut self, payload: &[u8]) -> Result<(), Self::Error>;
    /// Will return a boolean value of whether or not the radio is still transmitting.
    fn transmitting(&mut self) -> Result<bool, Self::Error>;

    /// Attempts to read a value on this channel. Unsuccessful reads result from a packet not being present.
    /// Successful reads would be one where up to 255 bytes of data are received.
    fn read_packet(&mut self) -> Result<Option<Vec<u8, 255>>, Self::Error>;
    /// Attempts to read a value on this channel. Unsuccessful reads can result from a hardware failure or the specified timeout passing.
    /// Successful reads would be ones where up to 255 bytes of data are received.
    fn read_packet_timeout<DELAY: DelayMs<u8>>(
        &mut self,
        timeout_ms: i32,
        delay: &mut DELAY,
    ) -> Result<Option<Vec<u8, 255>>, Self::Error>;
}
