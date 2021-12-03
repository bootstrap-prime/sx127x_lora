use bit_field::BitField;
use heapless::Vec;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::Mode;

mod register;
use register::AsAddr;
use register::{FskDataModulationShaping, FskRampUpRamDown};
use register::{IRQMask, PaConfig, Register};

/// Provides the necessary SPI mode configuration for the radio
/// Note that this may vary by device. Modules other than the RFM95
/// module requires may require `embedded_hal::spi::MODE_3` instead.
pub const MODE: Mode = embedded_hal::spi::MODE_0;

use crate::radio_traits::EmbeddedRadio;

/// Provides high-level access to Semtech SX1276/77/78/79 based boards connected to a Raspberry Pi
pub struct LoRa<SPI, CS, RESET> {
    spi: SPI,
    cs: CS,
    reset: RESET,
    frequency: i64,
    pub explicit_header: bool,
    pub mode: RadioMode,
}

#[derive(Debug)]
pub enum Error<SPI, CS, RESET> {
    Uninformative,
    VersionMismatch(u8),
    CS(CS),
    Reset(RESET),
    Spi(SPI),
    Transmitting,
}

use Error::*;

#[cfg(not(feature = "version_0x09"))]
const VERSION_CHECK: u8 = 0x12;

#[cfg(feature = "version_0x09")]
const VERSION_CHECK: u8 = 0x09;

/// Implement embedded_radio traits
impl<SPI, CS, RESET, E> EmbeddedRadio for LoRa<SPI, CS, RESET>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
    RESET: OutputPin,
{
    type Error = Error<E, CS::Error, RESET::Error>;

    /// Blocking version of transmit_payload().
    fn transmit_payload_busy(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        self.transmit_payload(payload)?;
        while self.transmitting()? {}
        Ok(())
    }

    /// Transmits up to 255 bytes of data. Takes a u8 slice of up to 255 elements. Returns () on success.
    fn transmit_payload(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        if self.transmitting()? {
            Err(Transmitting)
        } else {
            self.set_mode(RadioMode::Stdby)?;

            self.write_register(Register::IrqFlags, 0)?;
            self.write_register(Register::FifoAddrPtr, 0)?;
            self.write_register(Register::PayloadLength, 0)?;
            for &byte in payload.iter().take(255) {
                self.write_register(Register::Fifo, byte)?;
            }
            self.write_register(Register::PayloadLength, payload.len().min(255) as u8)?;
            self.set_mode(RadioMode::Tx)?;
            Ok(())
        }
    }

    /// Returns Some Vec with a capacity of 255 bytes, if a packet has arrived. If no packet has arrived, None
    /// is returned. Errors result from hardware faults.
    fn read_packet(&mut self) -> Result<Option<Vec<u8, 255>>, Self::Error> {
        self.set_mode(RadioMode::RxContinuous)?;
        if let Some(packet_size) = self.check_irq()? {
            // IRQ already cleared
            let mut buffer = Vec::new();

            let fifo_addr = self.read_register(Register::FifoRxCurrentAddr)?;
            self.write_register(Register::FifoAddrPtr, fifo_addr)?;

            for _ in 0..packet_size {
                let byte = self.read_register(Register::Fifo)?;
                // memory safety guaranteed here, packet size cannot be more than 255
                buffer.push(byte).ok();
            }
            self.write_register(Register::FifoAddrPtr, 0)?;

            Ok(Some(buffer))
        } else {
            Ok(None)
        }
    }

    /// Polls read_packet() for timeout (in milliseconds). Same return type.
    fn read_packet_timeout<DELAY: DelayMs<u8>>(
        &mut self,
        timeout_ms: i32,
        delay: &mut DELAY,
    ) -> Result<Option<Vec<u8, 255>>, Self::Error> {
        let mut count = 0;

        let packet = loop {
            let packet = self.read_packet()?;

            if packet.is_some() {
                break packet;
            }

            if count >= timeout_ms {
                break None;
            }

            count += 1;
            delay.delay_ms(1);
        };

        Ok(packet)
    }

    /// Returns true if the radio is currently transmitting a packet.
    fn transmitting(&mut self) -> Result<bool, Self::Error> {
        if (self.read_register(Register::OpMode)? & RadioMode::Tx.addr()) == RadioMode::Tx.addr() {
            Ok(true)
        } else {
            if (self.read_register(Register::IrqFlags)? & IRQMask::TxDone.addr()) == 1 {
                self.write_register(Register::IrqFlags, IRQMask::TxDone.addr())?;
            }
            Ok(false)
        }
    }
}

impl<SPI, CS, RESET, E> LoRa<SPI, CS, RESET>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
    RESET: OutputPin,
{
    /// Builds and returns a new instance of the radio. Only one instance of the radio should exist at a time.
    /// This also preforms a hardware reset of the module and then puts it in standby.
    pub fn new<DELAY: DelayMs<u8>>(
        spi: SPI,
        cs: CS,
        reset: RESET,
        frequency: i64,
        delay: &mut DELAY,
    ) -> Result<Self, Error<E, CS::Error, RESET::Error>> {
        let mut sx127x = LoRa {
            spi,
            cs,
            reset,
            frequency,
            explicit_header: true,
            mode: RadioMode::Sleep,
        };
        sx127x.reset.set_low().map_err(Reset)?;
        delay.delay_ms(10);
        sx127x.reset.set_high().map_err(Reset)?;
        delay.delay_ms(10);
        let version = sx127x.read_register(Register::Version)?;
        if version == VERSION_CHECK {
            sx127x.set_mode(RadioMode::Sleep)?;
            sx127x.set_frequency(frequency)?;
            sx127x.write_register(Register::FifoTxBaseAddr, 0)?;
            sx127x.write_register(Register::FifoRxBaseAddr, 0)?;
            let lna = sx127x.read_register(Register::Lna)?;
            sx127x.write_register(Register::Lna, lna | 0x03)?;
            sx127x.write_register(Register::ModemConfig3, 0x04)?;
            sx127x.set_mode(RadioMode::Stdby)?;
            sx127x.cs.set_high().map_err(CS)?;
            Ok(sx127x)
        } else {
            Err(Error::VersionMismatch(version))
        }
    }

    /// Return ownership of lora driver component elements.
    pub fn decompose(self) -> (SPI, CS, RESET) {
        (self.spi, self.cs, self.reset)
    }

    /// Check the radio's IRQ registers for a new packet, and only return it's size if one has arrived.
    fn check_irq(&mut self) -> Result<Option<usize>, Error<E, CS::Error, RESET::Error>> {
        let packet_ready: bool = self.read_register(Register::IrqFlags)?.get_bit(6);

        if packet_ready {
            self.clear_irq()?;
            Ok(Some(self.read_register(Register::RxNbBytes)? as usize))
        } else {
            Ok(None)
        }
    }

    /// Clears the radio's IRQ registers.
    fn clear_irq(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let irq_flags = self.read_register(Register::IrqFlags)?;
        self.write_register(Register::IrqFlags, irq_flags)?;

        Ok(())
    }

    /// Blocks the current thread, returning the size of a packet if one is received or an error is the
    /// task timed out. The timeout can be supplied with None to make it poll indefinitely or
    /// with `Some(timeout_in_milliseconds)`
    fn poll_irq<DELAY: DelayMs<u8>>(
        &mut self,
        timeout_ms: Option<i32>,
        delay: &mut DELAY,
    ) -> Result<usize, Error<E, CS::Error, RESET::Error>> {
        self.set_mode(RadioMode::RxContinuous)?;
        match timeout_ms {
            Some(value) => {
                let mut count = 0;
                let packet_ready = loop {
                    let packet_ready = self.read_register(Register::IrqFlags)?.get_bit(6);
                    if count >= value || packet_ready {
                        break packet_ready;
                    }
                    count += 1;
                    delay.delay_ms(1);
                };
                if packet_ready {
                    self.clear_irq()?;
                    Ok(self.read_register(Register::RxNbBytes)? as usize)
                } else {
                    Err(Uninformative)
                }
            }
            None => {
                while !self.read_register(Register::IrqFlags)?.get_bit(6) {
                    delay.delay_ms(100);
                }
                self.clear_irq()?;
                Ok(self.read_register(Register::RxNbBytes)? as usize)
            }
        }
    }

    pub fn set_dio0_tx_done(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        self.write_register(Register::DioMapping1, 0b01_00_00_00)?;

        Ok(())
    }

    /// Sets the transmit power and pin. Levels can range from 0-14 when the output
    /// pin = 0(RFO), and form 0-20 when output pin = 1(PaBoost). Power is in dB.
    /// Default value is `17`.
    pub fn set_tx_power(
        &mut self,
        level: i32,
        output_pin: u8,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        if PaConfig::PaOutputRfoPin.addr() == output_pin {
            // RFO
            let level = level.clamp(0, 14);

            self.write_register(Register::PaConfig, (0x70 | level) as u8)?;
        } else {
            // PA BOOST
            let mut level = level.clamp(2, 20);

            if level > 17 {
                // subtract 3 from level, so 18 - 20 maps to 15 - 17
                level -= 3;

                // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
                self.write_register(Register::PaDac, 0x87)?;
                self.set_ocp(140)?;
            } else {
                //Default value PA_HF/LF or +17dBm
                self.write_register(Register::PaDac, 0x84)?;
                self.set_ocp(100)?;
            }
            level -= 2;
            self.write_register(Register::PaConfig, PaConfig::PaBoost.addr() | level as u8)?;
        }

        Ok(())
    }

    /// Sets the over current protection on the radio(mA).
    pub fn set_ocp(&mut self, ma: u8) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let mut ocp_trim: u8 = 27;

        if ma <= 120 {
            ocp_trim = (ma - 45) / 5;
        } else if ma <= 240 {
            ocp_trim = (ma + 30) / 10;
        }
        self.write_register(Register::Ocp, 0x20 | (0x1F & ocp_trim))?;

        Ok(())
    }

    /// Sets the state of the radio. Default mode after initiation is `Standby`.
    pub fn set_mode(&mut self, mode: RadioMode) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        if mode != self.mode {
            if self.explicit_header {
                self.set_explicit_header_mode()?;
            } else {
                self.set_implicit_header_mode()?;
            }
            self.write_register(
                Register::OpMode,
                RadioMode::LongRangeMode.addr() | mode.addr(),
            )?;

            self.mode = mode;
        }
        Ok(())
    }

    /// Sets the frequency of the radio. Values are in megahertz.
    /// I.E. 915 MHz must be used for North America. Check regulation for your area.
    pub fn set_frequency(&mut self, freq: i64) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        self.frequency = freq;
        // calculate register values
        let base = 1;
        let frf = (freq * (base << 19)) / 32;
        // write registers
        self.write_register(Register::FrfMsb, ((frf & 0x00FF_0000) >> 16) as u8)?;
        self.write_register(Register::FrfMid, ((frf & 0x0000_FF00) >> 8) as u8)?;
        self.write_register(Register::FrfLsb, (frf & 0x0000_00FF) as u8)?;

        Ok(())
    }

    /// Sets the radio to use an explicit header. Default state is `ON`.
    fn set_explicit_header_mode(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let reg_modem_config_1 = self.read_register(Register::ModemConfig1)?;
        self.write_register(Register::ModemConfig1, reg_modem_config_1 & 0xfe)?;
        self.explicit_header = true;
        Ok(())
    }

    /// Sets the radio to use an implicit header. Default state is `OFF`.
    fn set_implicit_header_mode(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let reg_modem_config_1 = self.read_register(Register::ModemConfig1)?;
        self.write_register(Register::ModemConfig1, reg_modem_config_1 & 0x01)?;
        self.explicit_header = false;
        Ok(())
    }

    /// Sets the spreading factor of the radio. Supported values are between 6 and 12.
    /// If a spreading factor of 6 is set, implicit header mode must be used to transmit
    /// and receive packets. Default value is `7`.
    pub fn set_spreading_factor(
        &mut self,
        sf: u8,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let sf = sf.clamp(6, 12);

        if sf == 6 {
            self.write_register(Register::DetectionOptimize, 0xc5)?;
            self.write_register(Register::DetectionThreshold, 0x0c)?;
        } else {
            self.write_register(Register::DetectionOptimize, 0xc3)?;
            self.write_register(Register::DetectionThreshold, 0x0a)?;
        }
        let modem_config_2 = self.read_register(Register::ModemConfig2)?;
        self.write_register(
            Register::ModemConfig2,
            (modem_config_2 & 0x0f) | ((sf << 4) & 0xf0),
        )?;
        self.set_ldo_flag()?;
        Ok(())
    }

    /// Sets the signal bandwidth of the radio. Supported values are: `7800 Hz`, `10400 Hz`,
    /// `15600 Hz`, `20800 Hz`, `31250 Hz`,`41700 Hz` ,`62500 Hz`,`125000 Hz` and `250000 Hz`
    /// Default value is `125000 Hz`
    pub fn set_signal_bandwidth(
        &mut self,
        sbw: i64,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let bw: i64 = match sbw {
            7_800 => 0,
            10_400 => 1,
            15_600 => 2,
            20_800 => 3,
            31_250 => 4,
            41_700 => 5,
            62_500 => 6,
            125_000 => 7,
            250_000 => 8,
            _ => 9,
        };
        let modem_config_1 = self.read_register(Register::ModemConfig1)?;
        self.write_register(
            Register::ModemConfig1,
            (modem_config_1 & 0x0f) | ((bw << 4) as u8),
        )?;
        self.set_ldo_flag()?;
        Ok(())
    }

    /// Sets the coding rate of the radio with the numerator fixed at 4. Supported values
    /// are between `5` and `8`, these correspond to coding rates of `4/5` and `4/8`.
    /// Default value is `5`.
    pub fn set_coding_rate_4(
        &mut self,
        denominator: u8,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let denominator = denominator.clamp(5, 8);

        let cr = denominator - 4;
        let modem_config_1 = self.read_register(Register::ModemConfig1)?;
        self.write_register(Register::ModemConfig1, (modem_config_1 & 0xf1) | (cr << 1))?;

        Ok(())
    }

    /// Sets the preamble length of the radio. Values are between 6 and 65535.
    /// Default value is `8`.
    pub fn set_preamble_length(
        &mut self,
        length: i64,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        self.write_register(Register::PreambleMsb, (length >> 8) as u8)?;
        self.write_register(Register::PreambleLsb, length as u8)?;

        Ok(())
    }

    /// Enables are disables the radio's CRC check. Default value is `false`.
    pub fn set_crc(&mut self, value: bool) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let modem_config_2 = self.read_register(Register::ModemConfig2)?;
        if value {
            self.write_register(Register::ModemConfig2, modem_config_2 | 0x04)?;
        } else {
            self.write_register(Register::ModemConfig2, modem_config_2 & 0xfb)?;
        }

        Ok(())
    }

    /// Inverts the radio's IQ signals. Default value is `false`.
    pub fn set_invert_iq(&mut self, value: bool) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        if value {
            self.write_register(Register::Invertiq, 0x66)?;
            self.write_register(Register::Invertiq2, 0x19)?;
        } else {
            self.write_register(Register::Invertiq, 0x27)?;
            self.write_register(Register::Invertiq2, 0x1d)?;
        }
        Ok(())
    }

    /// Returns the spreading factor of the radio.
    pub fn get_spreading_factor(&mut self) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        Ok(self.read_register(Register::ModemConfig2)? >> 4)
    }

    /// Returns the signal bandwidth of the radio.
    pub fn get_signal_bandwidth(&mut self) -> Result<i64, Error<E, CS::Error, RESET::Error>> {
        let bw = self.read_register(Register::ModemConfig1)? >> 4;
        let bw = match bw {
            0 => 7_800,
            1 => 10_400,
            2 => 15_600,
            3 => 20_800,
            4 => 31_250,
            5 => 41_700,
            6 => 62_500,
            7 => 125_000,
            8 => 250_000,
            9 => 500_000,
            _ => -1,
        };
        Ok(bw)
    }

    /// Returns the RSSI of the last received packet.
    pub fn get_packet_rssi(&mut self) -> Result<i32, Error<E, CS::Error, RESET::Error>> {
        Ok(i32::from(self.read_register(Register::PktRssiValue)?) - 157)
    }

    /// Returns the signal to noise radio of the the last received packet.
    pub fn get_packet_snr(&mut self) -> Result<f64, Error<E, CS::Error, RESET::Error>> {
        Ok(f64::from(self.read_register(Register::PktSnrValue)?))
    }

    /// Returns the frequency error of the last received packet in Hz.
    pub fn get_packet_frequency_error(&mut self) -> Result<i64, Error<E, CS::Error, RESET::Error>> {
        let mut freq_error: i32;
        freq_error = i32::from(self.read_register(Register::FreqErrorMsb)? & 0x7);
        freq_error <<= 8_i64;
        freq_error += i32::from(self.read_register(Register::FreqErrorMid)?);
        freq_error <<= 8_i64;
        freq_error += i32::from(self.read_register(Register::FreqErrorLsb)?);

        let f_xtal = 32_000_000; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
        let f_error = ((f64::from(freq_error) * (1i64 << 24) as f64) / f64::from(f_xtal))
            * (self.get_signal_bandwidth()? as f64 / 500_000.0f64); // p. 37
        Ok(f_error as i64)
    }

    fn set_ldo_flag(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let sw = self.get_signal_bandwidth()?;
        // Section 4.1.1.5
        let symbol_duration = 1000 / (sw / ((1_i64) << self.get_spreading_factor()?));

        // Section 4.1.1.6
        let ldo_on = symbol_duration > 16;

        let mut config_3 = self.read_register(Register::ModemConfig3)?;
        config_3.set_bit(3, ldo_on);
        self.write_register(Register::ModemConfig3, config_3)?;

        Ok(())
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
        let reg = reg.addr();
        self.cs.set_low().map_err(CS)?;

        let mut buffer = [reg & 0x7f, 0];
        let transfer = self.spi.transfer(&mut buffer).map_err(Spi)?;
        self.cs.set_high().map_err(CS)?;
        Ok(transfer[1])
    }

    fn write_register(
        &mut self,
        reg: Register,
        byte: u8,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let reg = reg.addr();
        self.cs.set_low().map_err(CS)?;

        let buffer = [reg | 0x80, byte];
        self.spi.write(&buffer).map_err(Spi)?;
        self.cs.set_high().map_err(CS)?;
        Ok(())
    }

    /// Puts the radio in FSK mode.
    pub fn put_in_fsk_mode(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        // Put in FSK mode
        let mut op_mode: u8 = 0x0;
        op_mode
            .set_bit(7, false) // FSK mode
            .set_bits(5..6, 0x00) // FSK modulation
            .set_bit(3, false) //Low freq registers
            .set_bits(0..2, 0b011); // Mode

        self.write_register(Register::OpMode, op_mode)?;

        Ok(())
    }

    /// Sets the ramp-up time for FSK mode.
    pub fn set_fsk_pa_ramp(
        &mut self,
        modulation_shaping: FskDataModulationShaping,
        ramp: FskRampUpRamDown,
    ) -> Result<(), Error<E, CS::Error, RESET::Error>> {
        let mut pa_ramp: u8 = 0x0;
        pa_ramp
            .set_bits(5..6, modulation_shaping as u8)
            .set_bits(0..3, ramp as u8);

        self.write_register(Register::PaRamp, pa_ramp)?;

        Ok(())
    }
}

/// Modes of the radio and their corresponding register values.
#[derive(Clone, Copy, PartialEq)]
pub enum RadioMode {
    LongRangeMode = 0x80,
    Sleep = 0x00,
    Stdby = 0x01,
    Tx = 0x03,
    RxContinuous = 0x05,
    RxSingle = 0x06,
}

impl AsAddr for RadioMode {
    /// Returns the address of the mode.
    fn addr(self) -> u8 {
        self as u8
    }
}
