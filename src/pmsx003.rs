//! Plantower PMS-7003, High Precision Laser Dust Sensor Module PM1.0 PM2.5 PM10 sensors.
//!
//! https://www.pdf-archive.com/2017/04/12/plantower-pms-7003-sensor-data-sheet/plantower-pms-7003-sensor-data-sheet.pdf

use core::convert::TryInto;
use embedded_hal_02::serial;
use nb::block;

const PREFIX1: u8 = 0x42;
const PREFIX2: u8 = 0x4d;

pub struct PMSx003<S> {
    serial: S,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurements {
    pub cf1_pm1_0: u16,
    pub cf1_pm2_5: u16,
    pub cf1_pm10: u16,
    pub pm1_0: u16,
    pub pm2_5: u16,
    pub pm10: u16,
    pub db0_3_um: u16,
    pub db0_5_um: u16,
    pub db1_0_um: u16,
    pub db2_5_um: u16,
    pub db5_0_um: u16,
    pub db10_um: u16,
    pub version: u8,
    pub error_code: u8,
}

impl Measurements {
    // When the sensor is just started, all db*-um data will be zero.
    pub fn is_ready(&self) -> bool {
        self.db0_3_um != 0 || self.db0_5_um != 0 || self.db1_0_um != 0
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() != 32 {
            return None;
        }

        if &buf[..4] != &[PREFIX1, PREFIX2, 0x00, 0x1c] {
            return None;
        }

        let checksum = u16::from_be_bytes(buf[30..=31].try_into().unwrap());
        let mut sum: u16 = 0;
        for i in 0..30 {
            sum += buf[i] as u16;
        }
        if sum != checksum {
            return None;
        }

        // check ok
        Some(Measurements {
            cf1_pm1_0: u16::from_be_bytes(buf[4..6].try_into().unwrap()),
            cf1_pm2_5: u16::from_be_bytes(buf[6..8].try_into().unwrap()),
            cf1_pm10: u16::from_be_bytes(buf[8..10].try_into().unwrap()),
            pm1_0: u16::from_be_bytes(buf[10..12].try_into().unwrap()),
            pm2_5: u16::from_be_bytes(buf[12..14].try_into().unwrap()),
            pm10: u16::from_be_bytes(buf[14..16].try_into().unwrap()),
            db0_3_um: u16::from_be_bytes(buf[16..18].try_into().unwrap()),
            db0_5_um: u16::from_be_bytes(buf[18..20].try_into().unwrap()),
            db1_0_um: u16::from_be_bytes(buf[20..22].try_into().unwrap()),
            db2_5_um: u16::from_be_bytes(buf[22..24].try_into().unwrap()),
            db5_0_um: u16::from_be_bytes(buf[24..26].try_into().unwrap()),
            db10_um: u16::from_be_bytes(buf[26..28].try_into().unwrap()),
            version: buf[28],
            error_code: buf[29],
        })
    }
}

impl<S> PMSx003<S>
where
    S: serial::Read<u8> + serial::Write<u8>,
{
    pub fn new(serial: S) -> Self {
        Self { serial }
    }

    pub fn read_measurements(&mut self) -> Result<Measurements, ()> {
        // NOTE: 0x00_1c = 28, number of remaining bytes
        const PREFIX: [u8; 4] = [PREFIX1, PREFIX2, 0x00, 0x1c];

        let mut i = 0;
        let mut buf = [0u8; 26];
        let mut checksum: u16 = 0;

        loop {
            let c = self.must_read_byte();
            match i {
                0..=3 if c == PREFIX[i] => {
                    checksum += c as u16;
                    i += 1;
                }
                4..=29 => {
                    checksum += c as u16;
                    buf[i - 4] = c;
                    i += 1;
                }
                30 if c == (checksum >> 8) as u8 => {
                    i += 1;
                }
                31 if c == (checksum & 0xff) as u8 => {
                    break;
                }
                _ => {
                    i = 0;
                    checksum = 0;
                }
            }
        }

        Ok(Measurements {
            cf1_pm1_0: u16::from_be_bytes(buf[0..2].try_into().unwrap()),
            cf1_pm2_5: u16::from_be_bytes(buf[2..4].try_into().unwrap()),
            cf1_pm10: u16::from_be_bytes(buf[4..6].try_into().unwrap()),
            pm1_0: u16::from_be_bytes(buf[6..8].try_into().unwrap()),
            pm2_5: u16::from_be_bytes(buf[8..10].try_into().unwrap()),
            pm10: u16::from_be_bytes(buf[10..12].try_into().unwrap()),
            db0_3_um: u16::from_be_bytes(buf[12..14].try_into().unwrap()),
            db0_5_um: u16::from_be_bytes(buf[14..16].try_into().unwrap()),
            db1_0_um: u16::from_be_bytes(buf[16..18].try_into().unwrap()),
            db2_5_um: u16::from_be_bytes(buf[18..20].try_into().unwrap()),
            db5_0_um: u16::from_be_bytes(buf[20..22].try_into().unwrap()),
            db10_um: u16::from_be_bytes(buf[22..24].try_into().unwrap()),
            version: buf[24],
            error_code: buf[25],
        })
    }

    pub fn release(self) -> S {
        self.serial
    }

    fn must_read_byte(&mut self) -> u8 {
        loop {
            if let Ok(b) = self.read_byte() {
                return b;
            }
        }
    }

    fn read_byte(&mut self) -> Result<u8, ()> {
        block!(self.serial.read()).map_err(|_| ())
    }
}
