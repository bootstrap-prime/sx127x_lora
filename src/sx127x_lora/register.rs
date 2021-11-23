#![allow(dead_code)]

#[derive(Clone, Copy)]
pub enum Register {
    Fifo = 0x00,
    OpMode = 0x01,
    FrfMsb = 0x06,
    FrfMid = 0x07,
    FrfLsb = 0x08,
    PaConfig = 0x09,
    PaRamp = 0x0a,
    Ocp = 0x0b,
    Lna = 0x0c,
    FifoAddrPtr = 0x0d,
    FifoTxBaseAddr = 0x0e,
    FifoRxBaseAddr = 0x0f,
    FifoRxCurrentAddr = 0x10,
    IrqFlags = 0x12,
    RxNbBytes = 0x13,
    PktSnrValue = 0x19,
    PktRssiValue = 0x1a,
    ModemConfig1 = 0x1d,
    ModemConfig2 = 0x1e,
    PreambleMsb = 0x20,
    PreambleLsb = 0x21,
    PayloadLength = 0x22,
    ModemConfig3 = 0x26,
    FreqErrorMsb = 0x28,
    FreqErrorMid = 0x29,
    FreqErrorLsb = 0x2a,
    RssiWideband = 0x2c,
    DetectionOptimize = 0x31,
    Invertiq = 0x33,
    DetectionThreshold = 0x37,
    SyncWord = 0x39,
    Invertiq2 = 0x3b,
    DioMapping1 = 0x40,
    Version = 0x42,
    PaDac = 0x4d,
}
#[derive(Clone, Copy)]
pub enum PaConfig {
    PaBoost = 0x80,
    PaOutputRfoPin = 0,
}

#[derive(Clone, Copy)]
pub enum IRQMask {
    TxDone = 0x08,
    PayloadCrcError = 0x20,
    RxDone = 0x40,
}

pub trait AsAddr {
    fn addr(self) -> u8;
}

impl AsAddr for Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

impl AsAddr for PaConfig {
    fn addr(self) -> u8 {
        self as u8
    }
}

impl AsAddr for IRQMask {
    fn addr(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy)]
pub enum FskDataModulationShaping {
    None = 1,
    GaussianBt1d0 = 2,
    GaussianBt0d5 = 10,
    GaussianBt0d3 = 11,
}

#[derive(Clone, Copy)]
pub enum FskRampUpRamDown {
    _3d4ms = 0b000,
    _2ms = 0b0001,
    _1ms = 0b0010,
    _500us = 0b0011,
    _250us = 0b0100,
    _125us = 0b0101,
    _100us = 0b0110,
    _62us = 0b0111,
    _50us = 0b1000,
    _40us = 0b1001,
    _31us = 0b1010,
    _25us = 0b1011,
    _20us = 0b1100,
    _15us = 0b1101,
    _12us = 0b1110,
    _10us = 0b1111,
}
