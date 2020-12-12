//! The Dallas 1-wire driver.

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

const ROM_CMD_SEARCH_ROM: u8 = 0xf0;
const ROM_CMD_READ_ROM: u8 = 0x33;
const ROM_CMD_MATCH_ROM: u8 = 0x55;
const ROM_CMD_SKIP_ROM: u8 = 0xcc;
const ROM_CMD_ALARM_SEARCH: u8 = 0xec;

// TODO, handle errors
pub trait OneWirePinExt {
    fn to_input(&mut self) -> &mut dyn InputPin<Error = core::convert::Infallible>;
    fn to_output(&mut self) -> &mut dyn OutputPin<Error = core::convert::Infallible>;
}

#[derive(Debug)]
pub enum Error {
    WireNotLow,
    WireNotHigh,
    CrcMismatch(u8, u8),
    FamilyCodeMismatch(u8, u8),
    Debug(Option<u8>),
    PortError,
}

#[derive(Clone, PartialOrd, PartialEq)]
pub struct Device {
    pub address: [u8; 8],
}

impl Device {
    pub fn family_code(&self) -> u8 {
        self.address[0]
    }
}

impl ::core::fmt::Debug for Device {
    fn fmt(&self, f: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
        write!(
            f,
            "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            self.address[0],
            self.address[1],
            self.address[2],
            self.address[3],
            self.address[4],
            self.address[5],
            self.address[6],
            self.address[7],
        )
    }
}
impl core::str::FromStr for Device {
    type Err = core::num::ParseIntError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s.len() < 23 {
            let _ = u8::from_str_radix("", 16)?; // this causes a ParseIntError::Empty
        }
        Ok(Device {
            address: [
                u8::from_str_radix(&s[0..2], 16)?,
                u8::from_str_radix(&s[3..5], 16)?,
                u8::from_str_radix(&s[6..8], 16)?,
                u8::from_str_radix(&s[9..11], 16)?,
                u8::from_str_radix(&s[12..14], 16)?,
                u8::from_str_radix(&s[15..17], 16)?,
                u8::from_str_radix(&s[18..20], 16)?,
                u8::from_str_radix(&s[21..23], 16)?,
            ],
        })
    }
}

impl core::ops::Deref for Device {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.address[..]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum SearchState {
    Initialized,
    DeviceFound,
    End,
}

pub struct DeviceSearch<'a, P: OneWirePinExt> {
    address: [u8; 8],
    discrepancies: [u8; 8],
    state: SearchState,
    port: &'a mut OneWire<P>,
    delay: &'a mut DelayUs<u16>,
}

impl<'a, P: OneWirePinExt> DeviceSearch<'a, P> {}

pub struct OneWire<P: OneWirePinExt> {
    pin: P,
}

impl<P: OneWirePinExt> OneWire<P> {
    pub fn new(mut pin: P) -> Self {
        pin.to_output().set_high().unwrap();
        OneWire { pin }
    }

    pub fn reset(&mut self, delay: &mut impl DelayUs<u16>) -> Result<(), Error> {
        // let mut cli = DisableInterrupts::new();
        self.pin.to_output().set_high().unwrap();
        self.pin.to_output().set_low().unwrap();

        delay.delay_us(480);
        // self.pin.to_output().set_high().unwrap();
        let pin = self.pin.to_input();

        // Master Rx
        let mut val = false;
        for _ in 0..24 {
            if pin.is_low().unwrap() {
                val |= true;
            }
            delay.delay_us(10);
        }
        if !val {
            return Err(Error::WireNotLow);
        }
        val = false;
        for _ in 0..24 {
            if pin.is_high().unwrap() {
                val |= true;
            }
            delay.delay_us(10);
        }
        if !val {
            return Err(Error::WireNotHigh);
        }

        Ok(())
    }

    /// This command can only be used when there is one slave
    /// on the bus. It allows the bus master to read the slave’s
    /// 64-bit ROM code without using the Search ROM procedure.
    pub fn read_rom(&mut self, delay: &mut impl DelayUs<u16>) -> Result<Device, Error> {
        self.reset(delay)?;
        self.write_byte(delay, ROM_CMD_READ_ROM)?;
        let mut rom = [0u8; 8];
        self.read_bytes(delay, &mut rom)?;
        Ok(Device { address: rom })
    }

    pub fn search_device<'a>(&'a mut self, delay: &'a mut impl DelayUs<u16>) -> DeviceSearch<'a, P> {
        DeviceSearch {
            address: [0u8; 8],
            discrepancies: [0u8; 8],
            state: SearchState::Initialized,
            port: self,
            delay,
        }
    }

    /// Strong pullup to allow parasite_mode
    pub fn pullup(&mut self) {
        self.pin.to_output().set_high().unwrap();
    }

    pub fn write_bytes(&mut self, delay: &mut impl DelayUs<u16>, bytes: &[u8]) -> Result<(), Error> {
        for b in bytes {
            self.write_byte(delay, *b)?;
        }
        Ok(())
    }

    pub fn write_byte(&mut self, delay: &mut impl DelayUs<u16>, mut byte: u8) -> Result<(), Error> {
        for _ in 0..8 {
            self.write_bit(delay, (byte & 0x01) == 0x01)?;
            byte >>= 1;
        }
        Ok(())
    }

    fn write_bit(&mut self, delay: &mut impl DelayUs<u16>, high: bool) -> Result<(), Error> {
        // master write
        let pin = self.pin.to_output();
        pin.set_low().unwrap();
        delay.delay_us(if high { 10 } else { 65 });
        pin.set_high().unwrap();
        delay.delay_us(if high { 55 } else { 5 });
        Ok(())
    }

    pub fn read_bytes(&mut self, delay: &mut impl DelayUs<u16>, dst: &mut [u8]) -> Result<(), Error> {
        for d in dst {
            *d = self.read_byte(delay)?;
        }
        Ok(())
    }

    pub fn read_byte(&mut self, delay: &mut impl DelayUs<u16>) -> Result<u8, Error> {
        let mut byte = 0_u8;
        for _ in 0..8 {
            byte >>= 1;
            if self.read_bit(delay)? {
                byte |= 0x80;
            }
        }
        Ok(byte)
    }

    pub fn read_bit(&mut self, delay: &mut impl DelayUs<u16>) -> Result<bool, Error> {
        self.pin.to_output().set_low().unwrap();
        delay.delay_us(3);
        let pin = self.pin.to_input();
        delay.delay_us(2);

        let val = pin.is_high().unwrap();
        delay.delay_us(61);
        Ok(val)
    }
}
