use embedded_hal::{
    delay::DelayUs,
    digital::{InputPin, OutputPin},
    spi::{SpiBus, SpiBusRead, SpiBusWrite},
};

pub trait InOutPin {
    type Input: InputPin;
    type Output: OutputPin;
    fn set_output(&mut self);
    fn set_input(&mut self);
    fn as_input(&self) -> &Self::Input;
    fn as_output(&mut self) -> &mut Self::Output;
}

pub struct ThreeWire<CE, IO, CLK>
where
    CE: OutputPin,
    IO: InOutPin,
    CLK: OutputPin,
{
    pub ce: CE,
    pub io: IO,
    pub clk: CLK,
}

impl<CE, IO, CLK> ThreeWire<CE, IO, CLK>
where
    CE: OutputPin,
    IO: InOutPin,
    CLK: OutputPin,
{
    pub fn new(ce: CE, io: IO, clk: CLK) -> Self {
        ThreeWire { ce, io, clk }
    }

    fn read_byte<D: DelayUs>(&mut self, delay: &mut D) -> u8 {
        let mut byte = 0u8;
        for i in 0..8 {
            byte |= (self.io.as_input().is_high().unwrap() as u8) << i;
            delay.delay_us(10);
            self.clk.set_high().unwrap();
            delay.delay_us(10);
            self.clk.set_low().unwrap();
        }
        byte
    }

    fn write_byte<D: DelayUs>(&mut self, delay: &mut D, byte: u8) {
        for i in 0..8 {
            if (byte >> i) & 1 != 0 {
                self.io.as_output().set_high().unwrap();
            } else {
                self.io.as_output().set_low().unwrap();
            }

            delay.delay_us(10);
            self.clk.set_high().unwrap();
            delay.delay_us(10);
            self.clk.set_low().unwrap();
        }
    }

    pub fn read_reg<D: DelayUs>(&mut self, delay: &mut D, addr: u8) -> u8 {
        self.ce.set_high().unwrap();
        self.io.set_output();
        delay.delay_us(10_u32);
        self.write_byte(delay, addr);

        self.io.set_input();
        delay.delay_us(10_u32);
        let out = self.read_byte(delay);
        self.ce.set_low().unwrap();
        out
    }

    pub fn write_reg<D: DelayUs>(&mut self, delay: &mut D, addr: u8, val: u8) {
        self.ce.set_high().unwrap();
        self.io.set_output();
        delay.delay_us(10_u32);
        self.write_byte(delay, addr);
        delay.delay_us(10_u32);
        self.write_byte(delay, val);
        self.io.set_input();
        self.ce.set_low().unwrap();
    }

    pub fn read_hms<D: DelayUs>(&mut self, delay: &mut D) -> (u8, u8, u8) {
        let mut h = self.read_reg(delay, 0x85);
        let mut m = self.read_reg(delay, 0x83);
        let mut s = self.read_reg(delay, 0x81);

        #[cfg(feature = "defmt")]
        defmt::info!("h: {:02x}, m: {:02x}, s: {:02x}", h, m, s);

        h = ((h & 0b0001_1111) >> 4) * 10 + (h & 0b0000_1111);
        m = (m >> 4) * 10 + (m & 0x0f);
        s = (s >> 4) * 10 + (s & 0x0f);

        (h, m, s)
    }

    pub fn read_ymd<D: DelayUs>(&mut self, delay: &mut D) -> (u8, u8, u8) {
        let mut y = self.read_reg(delay, 0x8d);
        let mut m = self.read_reg(delay, 0x89);
        let mut d = self.read_reg(delay, 0x87);

        #[cfg(feature = "defmt")]
        defmt::info!("y: {:02x}, m: {:02x}, d: {:02x}", y, m, d);

        y = (y >> 4) * 10 + (y & 0x0f);
        m = (m >> 4) * 10 + (m & 0x0f);
        d = (d >> 4) * 10 + (d & 0x0f);

        (y, m, d)
    }

    /// Weekday 1-7
    pub fn read_day<D: DelayUs>(&mut self, delay: &mut D) -> u8 {
        let day = self.read_reg(delay, 0x8b);
        day
    }

    pub fn set_hms<D: DelayUs>(&mut self, delay: &mut D, h: u8, m: u8, s: u8) {
        self.write_reg(delay, 0x84, ((h / 10) << 4) | (h % 10));
        self.write_reg(delay, 0x82, ((m / 10) << 4) | (m % 10));
        self.write_reg(delay, 0x80, ((s / 10) << 4) | (s % 10));
    }
    pub fn set_ymd<D: DelayUs>(&mut self, delay: &mut D, y: u8, m: u8, d: u8) {
        self.write_reg(delay, 0x8c, ((y / 10) << 4) | (y % 10));
        self.write_reg(delay, 0x88, ((m / 10) << 4) | (m % 10));
        self.write_reg(delay, 0x86, ((d / 10) << 4) | (d % 10));
    }
    pub fn set_day<D: DelayUs>(&mut self, delay: &mut D, day: u8) {
        if day >= 1 && day <= 7 {
            self.write_reg(delay, 0x8a, day);
        }
    }
}
