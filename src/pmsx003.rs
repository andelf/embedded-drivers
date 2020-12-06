//! PMSx003, High Precision Laser Dust Sensor Module PM1.0 PM2.5 PM10 sensors.
//!
//!

use core::convert::TryInto;
use embedded_hal::serial;
use nb::block;

const PREFIX1: u8 = 0x42;
const PREFIX2: u8 = 0x4d;

pub struct PMSx003<S> {
    serial: S,
    read_buf: [u8; 30],
}

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
}

impl Measurements {
    // When the sensor is just started, all db*-um data will be zero.
    pub fn is_ready(&self) -> bool {
        self.db0_3_um != 0 || self.db0_5_um != 0 || self.db1_0_um != 0
    }
}

impl<S> PMSx003<S>
where
    S: serial::Read<u8> + serial::Write<u8>,
{
    pub fn new(serial: S) -> Self {
        Self {
            serial,
            read_buf: [0u8; 30],
        }
    }

    pub fn read_measurements(&mut self) -> Result<Measurements, ()> {
        loop {
            self.read_until(PREFIX1);
            match self.read_byte() {
                Ok(b) if b == PREFIX2 => break,
                _ => continue,
            }
        }
        let mut sum: u16 = (PREFIX1 + PREFIX2) as u16;
        // remain 30 bytes
        for i in 0..30 {
            let b = self.read_byte().unwrap();
            self.read_buf[i] = b;
            if i < 28 {
                sum += b as u16;
            }
        }
        let buf = &self.read_buf[..];
        if buf[0] != 0x00 || buf[1] != 0x1c {
            return Err(()); // length error, might restart
        }
        if buf[28] != (sum >> 8) as u8 || buf[29] != (sum & 0xff) as u8 {
            return Err(()); // checksum error
        }

        Ok(Measurements {
            cf1_pm1_0: u16::from_be_bytes(buf[2..4].try_into().unwrap()),
            cf1_pm2_5: u16::from_be_bytes(buf[4..6].try_into().unwrap()),
            cf1_pm10: u16::from_be_bytes(buf[6..8].try_into().unwrap()),
            pm1_0: u16::from_be_bytes(buf[8..10].try_into().unwrap()),
            pm2_5: u16::from_be_bytes(buf[10..12].try_into().unwrap()),
            pm10: u16::from_be_bytes(buf[12..14].try_into().unwrap()),
            db0_3_um: u16::from_be_bytes(buf[14..16].try_into().unwrap()),
            db0_5_um: u16::from_be_bytes(buf[16..18].try_into().unwrap()),
            db1_0_um: u16::from_be_bytes(buf[18..20].try_into().unwrap()),
            db2_5_um: u16::from_be_bytes(buf[20..22].try_into().unwrap()),
            db5_0_um: u16::from_be_bytes(buf[22..24].try_into().unwrap()),
            db10_um: u16::from_be_bytes(buf[24..26].try_into().unwrap()),
        })
    }

    pub fn release(self) -> S {
        self.serial
    }

    fn read_until(&mut self, byte: u8) {
        loop {
            if let Ok(b) = self.serial.read() {
                if b == byte {
                    return;
                }
            }
        }
    }

    fn read_byte(&mut self) -> Result<u8, ()> {
        block!(self.serial.read()).map_err(|_| ())
    }
}
