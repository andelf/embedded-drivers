use embedded_hal_1::{delay::DelayUs, i2c::I2c};

/// BH1750 Ambient Light Sensor(ALS)
///
/// - Output: lux(lx)
/// - Range: 0 to 65535
/// - Adddress(7bit): 0x23 or 0x5C
pub struct BH1750<I2C> {
    i2c: I2C,
    address: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Resolution {
    /// 1lx, Measurement Time is typically 120ms, first might be 180ms
    High,
    /// 0.5lx, Measurement Time is typically 120ms.
    High2,
    /// 4lx, 16ms
    Low,
}

impl Default for Resolution {
    fn default() -> Self {
        Resolution::High
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub continuous: bool,
    pub resolution: Resolution,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            continuous: true,
            resolution: Resolution::High,
        }
    }
}

impl<I2C> BH1750<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        BH1750 { i2c, address }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    pub fn init(&mut self, config: Config, delay: &mut impl DelayUs) -> Result<(), I2C::Error> {
        // power on
        self.i2c.write(self.address, &[0x01])?;
        // defaults to Continuously H-Resolution Mode
        let mut opcode = 0x10;
        if !config.continuous {
            opcode = 0x20;
        }
        match config.resolution {
            Resolution::High => {}
            Resolution::High2 => opcode |= 0x01,
            Resolution::Low => opcode |= 0x03,
        }
        self.i2c.write(self.address, &[opcode])?;
        // delay values are from Electrical Characteristics in datasheet
        if matches!(config.resolution, Resolution::Low) {
            delay.delay_us(24_000);
        } else {
            delay.delay_us(180_000);
        }
        Ok(())
    }

    #[inline]
    pub fn read_raw(&mut self) -> Result<u16, I2C::Error> {
        let mut buf = [0u8; 2];
        self.i2c.read(self.address, &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    pub fn read_lux(&mut self) -> Result<u16, I2C::Error> {
        // Ok((self.read_raw()? as f32) / 1.2)
        // avoid using floating point arithmetic
        let raw_val = self.read_raw()?;
        Ok((((raw_val as u32) * 54516) >> 16) as u16)
    }

    // Is dark, lower than 10 lx
    pub fn is_dark(&mut self) -> Result<bool, I2C::Error> {
        const THRESHOLD: u16 = 12;
        Ok(self.read_raw()? < THRESHOLD)
    }
}
