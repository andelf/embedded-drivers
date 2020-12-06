//! LM75 temperature sensor
//!

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const LM75_I2CADDR: u8 = 0x48;

pub struct LM75<I> {
    device: I,
}

impl<I: Write + WriteRead + Read> LM75<I> {
    /// Create device driver instance.
    pub fn new(i2c: I) -> Self {
        LM75 { device: i2c }
    }

    pub fn measure(&mut self) -> Result<f32, ()> {
        let raw = self.read_u16(0x00);
        Ok((raw >> 8) as f32 + (0.5 * ((raw >> 7) & 0b1) as f32))
    }

    pub fn release(self) -> I {
        self.device
    }

    fn read_u16(&mut self, reg: u8) -> u16 {
        let mut buf = [0u8; 2];
        let _ = self.device.write_read(LM75_I2CADDR, &[reg], &mut buf[..]);
        u16::from_be_bytes(buf)
    }
}
