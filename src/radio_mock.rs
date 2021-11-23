use embedded_hal::blocking::delay::DelayMs;
use heapless::Vec;
use std::sync::mpsc;
use crate::EmbeddedRadio;

#[derive(Debug)]
pub enum LoraError<RX, TX> {
    Receiver(RX),
    Transmitter(TX),
}

type RadioBuffer = Vec<u8, 255>;

pub struct MockLora {
    rx: mpsc::Receiver<RadioBuffer>,
    tx: mpsc::Sender<RadioBuffer>,
}

impl MockLora {
    pub fn new() -> (MockLora, MockLora) {
        let (tx1, rx2) = mpsc::channel();
        let (tx2, rx1) = mpsc::channel();

        (MockLora { rx: rx1, tx: tx1 }, MockLora { rx: rx2, tx: tx2 })
    }
}

impl EmbeddedRadio for MockLora {
    type Error = LoraError<mpsc::RecvError, mpsc::SendError<RadioBuffer>>;

    fn transmit_payload(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        let mut buffer: RadioBuffer = Vec::new();

        for &payload_byte in payload.iter().take(255) {
            buffer.push(payload_byte).unwrap();
        }

        self.tx
            .send(buffer)
            .map_err(|e| Self::Error::Transmitter(e))?;
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
            Err(mpsc::TryRecvError::Empty) => Ok(None),
            Err(mpsc::TryRecvError::Disconnected) => Err(Self::Error::Receiver(mpsc::RecvError)),
        }
    }

    fn read_packet_timeout<DELAY: DelayMs<u8>>(
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
            let mut payload: Vec<u8, 5> = Vec::new();
            payload.extend_from_slice(&[1, 2, 3, 4]).unwrap();

            lora.transmit_payload(&payload[..]).unwrap();

            let payload_received = lora.read_packet().unwrap();
            println!("{:?}", payload_received);

            assert!(payload_received.is_none());
        }

        let (mut lora1, _unusedlora) = MockLora::new();
        test_lora(&mut lora1);
        let (_unusedlora, mut lora2) = MockLora::new();
        test_lora(&mut lora2);
    }

    #[test]
    fn transmit_between() {
        let (mut lora_1, mut lora_2) = MockLora::new();

        let mut payload: Vec<u8, 255> = Vec::new();
        payload.extend_from_slice(&[1, 2, 3, 4]).unwrap();

        lora_1.transmit_payload(&payload[..]).unwrap();

        let received_payload = lora_2.read_packet().unwrap();

        assert_eq!(received_payload, Some(payload));
    }
}
