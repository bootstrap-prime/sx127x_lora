use crate::EmbeddedRadio;
use crossbeam::channel;
use embedded_hal::blocking::delay::DelayMs;

#[derive(Debug)]
pub enum LoraError<RX, TX> {
    Receiver(RX),
    Transmitter(TX),
}

type RadioBuffer = heapless::Vec<u8, 255>;

pub struct MockLora {
    rx: channel::Receiver<RadioBuffer>,
    tx: Vec<channel::Sender<RadioBuffer>>,
}

impl MockLora {
    pub fn new(num_radios: usize) -> Vec<MockLora> {
        let radios_antennas: Vec<(_, _)> = (0..num_radios)
            .map(|_| channel::unbounded::<RadioBuffer>())
            .enumerate()
            .collect();

        let radios_antennas_reference = radios_antennas.clone();

        let lora_modules: Vec<MockLora> = radios_antennas
            .iter()
            .map(|self_antenna| {
                let txs = radios_antennas_reference
                    .iter()
                    .filter(|e| e.0 != self_antenna.0)
                    .map(|radio_ref| radio_ref.1 .0.clone())
                    .collect();

                MockLora {
                    rx: self_antenna.1 .1.clone(),
                    tx: txs,
                }
            })
            .collect();

        lora_modules
    }
}

impl EmbeddedRadio for MockLora {
    type Error = LoraError<channel::RecvError, channel::SendError<RadioBuffer>>;

    fn transmit_payload(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        let mut buffer: RadioBuffer = heapless::Vec::new();

        for &payload_byte in payload.iter().take(255) {
            buffer.push(payload_byte).unwrap();
        }

        for tx in self.tx.iter() {
            tx.send(buffer.clone())
                .map_err(|e| Self::Error::Transmitter(e))?;
        }

        Ok(())
    }

    fn transmit_payload_busy(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        self.transmit_payload(&payload)?;
        while self.transmitting()? {}
        Ok(())
    }

    fn transmitting(&mut self) -> Result<bool, Self::Error> {
        // this shim ignores this, mpsc does not block on transmission
        Ok(false)
    }

    fn read_packet(&mut self) -> Result<Option<RadioBuffer>, Self::Error> {
        match self.rx.try_recv() {
            Ok(value) => Ok(Some(value)),
            Err(channel::TryRecvError::Empty) => Ok(None),
            Err(channel::TryRecvError::Disconnected) => {
                Err(Self::Error::Receiver(channel::RecvError))
            }
        }
    }

    fn read_packet_timeout<DELAY: DelayMs<u16>>(
        &mut self,
        timeout_ms: i32,
        delay: &mut DELAY,
    ) -> Result<Option<RadioBuffer>, Self::Error> {
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
}

// just some quick tests to confirm the mock exists
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_transmit_self() {
        fn test_lora(lora: &mut MockLora) {
            let mut payload: heapless::Vec<u8, 5> = heapless::Vec::new();
            payload.extend_from_slice(&[1, 2, 3, 4]).unwrap();

            lora.transmit_payload(&payload[..]).unwrap();

            let payload_received = lora.read_packet().unwrap();
            println!("{:?}", payload_received);

            assert!(payload_received.is_none());
        }

        let mut loras = MockLora::new(2);
        let mut lora1 = loras.pop().unwrap();
        test_lora(&mut lora1);
        let mut loras = MockLora::new(2);
        let mut lora2 = loras.pop().unwrap();
        test_lora(&mut lora2);
    }

    #[test]
    fn transmit_between_two() {
        let mut loras = MockLora::new(2);
        let mut lora_1 = loras.pop().unwrap();
        let mut lora_2 = loras.pop().unwrap();

        let mut payload: heapless::Vec<u8, 255> = heapless::Vec::new();
        payload.extend_from_slice(&[1, 2, 3, 4]).unwrap();

        lora_1.transmit_payload(&payload[..]).unwrap();

        let received_payload = lora_2.read_packet().unwrap();

        assert_eq!(received_payload, Some(payload));
    }
}
