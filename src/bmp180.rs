//! BMP180 Digital pressure sensor.
//!

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use num_traits::Pow;

// BMP180, BMP085 address.
const BMP180_I2CADDR: u8 = 0x77;

const BMP180_CAL_AC1: u8 = 0xAA; // R   Calibration data (16 bits)
const BMP180_CAL_AC2: u8 = 0xAC; // R   Calibration data (16 bits)
const BMP180_CAL_AC3: u8 = 0xAE; // R   Calibration data (16 bits)
const BMP180_CAL_AC4: u8 = 0xB0; // R   Calibration data (16 bits)
const BMP180_CAL_AC5: u8 = 0xB2; // R   Calibration data (16 bits)
const BMP180_CAL_AC6: u8 = 0xB4; // R   Calibration data (16 bits)
const BMP180_CAL_B1: u8 = 0xB6; // R   Calibration data (16 bits)
const BMP180_CAL_B2: u8 = 0xB8; // R   Calibration data (16 bits)
const BMP180_CAL_MB: u8 = 0xBA; // R   Calibration data (16 bits)
const BMP180_CAL_MC: u8 = 0xBC; // R   Calibration data (16 bits)
const BMP180_CAL_MD: u8 = 0xBE; // R   Calibration data (16 bits)

const BMP180_CONTROL: u8 = 0xF4;
const BMP180_TEMPDATA: u8 = 0xF6;
const BMP180_PRESSUREDATA: u8 = 0xF6;

const BMP180_READTEMPCMD: u8 = 0x2E;
const BMP180_READPRESSURECMD: u8 = 0x34;

/// Hardware pressure sampling accuracy modes.
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Mode {
    UltraLowPower = 0,
    Standard,
    HighResolution,
    UltraHighResolution,
}

impl Mode {
    fn oversampling_settings(&self) -> u8 {
        *self as u8
    }

    fn wait_conversion<D: DelayMs<u8>>(&self, delay: &mut D) {
        let ms = match self {
            Mode::UltraLowPower => 5,
            Mode::Standard => 8,
            Mode::HighResolution => 14,
            Mode::UltraHighResolution => 26,
        };
        delay.delay_ms(ms);
    }
}

/// BMP180, or BMP085.
pub struct BMP180<I> {
    device: I,
    mode: Mode,
    ac1: i16,
    ac2: i16,
    ac3: i16,
    ac4: u16,
    ac5: u16,
    ac6: u16,
    b1: i16,
    b2: i16,
    mb: i16,
    mc: i16,
    md: i16,
}

impl<I: Write + WriteRead + Read> BMP180<I> {
    /// Create device driver instance.
    pub fn new(i2c: I) -> Self {
        BMP180 {
            device: i2c,
            mode: Mode::UltraHighResolution,
            ac1: 408,
            ac2: -72,
            ac3: -14383,
            ac4: 32741,
            ac5: 32757,
            ac6: 23153,
            b1: 6190,
            b2: 4,
            mb: -32767,
            mc: -8711,
            md: 2868,
        }
    }

    /// Read calibration data from the EEPROM of BMP180.
    pub fn init(&mut self) {
        self.ac1 = self.read_i16(BMP180_CAL_AC1);
        self.ac2 = self.read_i16(BMP180_CAL_AC2);
        self.ac3 = self.read_i16(BMP180_CAL_AC3);

        self.ac4 = self.read_u16(BMP180_CAL_AC4);
        self.ac5 = self.read_u16(BMP180_CAL_AC5);
        self.ac6 = self.read_u16(BMP180_CAL_AC6);

        self.b1 = self.read_i16(BMP180_CAL_B1);
        self.b2 = self.read_i16(BMP180_CAL_B2);
        self.mb = self.read_i16(BMP180_CAL_MB);
        self.mc = self.read_i16(BMP180_CAL_MC);
        self.md = self.read_i16(BMP180_CAL_MD);
    }

    /// read uncompensated temperature value
    #[inline]
    fn get_ut<D: DelayMs<u8>>(&mut self, delay: &mut D) -> i32 {
        self.write(&[BMP180_CONTROL, BMP180_READTEMPCMD]);
        delay.delay_ms(5);
        self.read_i16(BMP180_TEMPDATA) as i32
    }

    /// read uncompensated pressure value
    #[inline]
    fn get_up<D: DelayMs<u8>>(&mut self, delay: &mut D) -> i32 {
        let oss = self.mode.oversampling_settings();
        self.write(&[BMP180_CONTROL, BMP180_READPRESSURECMD + (oss << 6)]);
        self.mode.wait_conversion(delay);

        let msb = self.read_u8(BMP180_PRESSUREDATA) as i32;
        let lsb = self.read_u8(BMP180_PRESSUREDATA + 1) as i32;
        let xlsb = self.read_u8(BMP180_PRESSUREDATA + 2) as i32;

        ((msb << 16) + (lsb << 8) + xlsb) >> (8 - oss)
    }

    /// Calculate true temperature, resolution is 0.1C
    pub fn get_temperature<D: DelayMs<u8>>(&mut self, delay: &mut D) -> f32 {
        let ut = self.get_ut(delay);

        let x1 = ((ut - self.ac6 as i32) * self.ac5 as i32) >> 15;
        let x2 = ((self.mc as i32) << 11) / (x1 + self.md as i32);
        let b5 = x1 + x2;
        ((b5 + 8) >> 4) as f32 / 10.0
    }

    /// Calculate true pressure, in Pa
    pub fn get_pressure<D: DelayMs<u8>>(&mut self, delay: &mut D) -> i32 {
        let oss = self.mode.oversampling_settings();

        let ut = self.get_ut(delay);
        let up = self.get_up(delay);

        let x1 = ((ut - self.ac6 as i32) * self.ac5 as i32) >> 15;
        let x2 = ((self.mc as i32) << 11) / (x1 + self.md as i32);
        let b5 = x1 + x2;

        let b6 = b5 - 4000;
        let x1 = ((self.b2 as i32) * ((b6 * b6) >> 12)) >> 11;
        let x2 = ((self.ac2 as i32) * b6) >> 11;
        let x3 = x1 + x2;
        // NOTE: must use i64 type
        let b3: i64 = ((((self.ac1 as i64) * 4 + x3 as i64) << oss) + 2) / 4;

        let x1 = ((self.ac3 as i32) * b6) >> 13;
        let x2 = ((self.b1 as i32) * ((b6 * b6) >> 12)) >> 16;
        let x3 = ((x1 + x2) + 2) >> 2;
        let b4: u32 = (self.ac4 as u32) * ((x3 + 32768) as u32) >> 15;

        let b7 = (up as i64 - b3 as i64) * (50000 >> oss);

        let p = if b7 < 0x80000000 {
            (b7 * 2) / (b4 as i64)
        } else {
            (b7 / (b4 as i64)) * 2
        };

        let x1 = (p >> 8) * (p >> 8);
        let x1 = (x1 * 3038) >> 16;
        let x2 = (-7357 * p) >> 16;
        let p = p + ((x1 + x2 + 3791) >> 4);

        p as i32
    }

    /// Calculate absolute altitude
    pub fn calculate_altitude<D: DelayMs<u8>>(&mut self, delay: &mut D, sealevel_pa: f32) -> f32 {
        let pa = self.get_pressure(delay) as f32;
        44330.0 * (1.0 - (pa / sealevel_pa).pow(1.0 / 5.255))
    }

    /// Calculate pressure at sea level
    pub fn calculate_sealevel_pressure<D: DelayMs<u8>>(
        &mut self,
        delay: &mut D,
        altitude_m: f32,
    ) -> u32 {
        let pressure = self.get_pressure(delay) as f32;
        let p0 = pressure / (1.0 - altitude_m / 44330.0).pow(5.255);
        p0 as u32
    }

    pub fn release(self) -> I {
        self.device
    }

    fn write(&mut self, data: &[u8]) {
        let _ = self.device.write(BMP180_I2CADDR, data);
    }

    fn read_u8(&mut self, reg: u8) -> u8 {
        let mut buf = [0u8];
        let _ = self.device.write_read(BMP180_I2CADDR, &[reg], &mut buf[..]);
        buf[0]
    }

    fn read_i16(&mut self, reg: u8) -> i16 {
        let mut buf = [0u8; 2];
        // buf[0] = self.read_u8(reg);
        // buf[1] = self.read_u8(reg + 1);
        let _ = self.device.write_read(BMP180_I2CADDR, &[reg], &mut buf[..]);
        i16::from_be_bytes(buf)
    }

    fn read_u16(&mut self, reg: u8) -> u16 {
        let mut buf = [0u8; 2];
        // buf[0] = self.read_u8(reg);
        // buf[1] = self.read_u8(reg + 1);
        let _ = self.device.write_read(BMP180_I2CADDR, &[reg], &mut buf[..]);
        u16::from_be_bytes(buf)
    }
}
