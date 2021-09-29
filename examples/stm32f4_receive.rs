#![no_std]
#![no_main]

extern crate cortex_m;
extern crate panic_semihosting;
extern crate stm32f4xx_hal as hal;
extern crate sx127x_lora;

use cortex_m_semihosting::*;
use stm32f4xx_hal::{
    delay::Delay,
    pac,
    spi::Spi,
    prelude::*,
    time::MegaHertz,
};

use sx127x_lora::MODE;

const FREQUENCY: i64 = 915;

#[cortex_m_rt::entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(MegaHertz(64))
        .pclk1(MegaHertz(32))
        .freeze();

    let mut gpioa = p.GPIOA.split();
    let mut gpiod = p.GPIOD.split();
    let mut gpiof = p.GPIOF.split();

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    let reset = gpiof.pf13.into_push_pull_output();
    let cs = gpiod.pd14.into_push_pull_output();

    let mut delay = Delay::new(cp.SYST, &clocks);

    let spi = Spi::new(
        p.SPI1,
        (sck, miso, mosi),
        MODE,
        MegaHertz(8),
        clocks,
    );

    let mut lora =
        sx127x_lora::LoRa::new(spi, cs, reset, FREQUENCY, &mut delay).unwrap();

    loop {
        let poll = lora.poll_irq(Some(30), &mut delay); //30 Second timeout
        match poll {
            Ok(size) => {
                hprintln!(
                    "New Packet with size {} and RSSI: {}",
                    size,
                    lora.get_packet_rssi().unwrap()
                )
                .unwrap();
                let buffer = lora.read_packet().unwrap(); // Received buffer. NOTE: 255 bytes are always returned
                hprint!("with Payload: ");
                for i in 0..size {
                    hprint!("{}", buffer[i] as char).unwrap();
                }
                hprintln!();
            }
            Err(_) => hprintln!("Timeout").unwrap(),
        }
    }
}
