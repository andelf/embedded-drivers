//! The Dallas 1-wire driver.

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

pub use self::ds18b20::DS18B20;

pub mod ds18b20;

const ADDRESS_BYTES: u8 = 8;
const ADDRESS_BITS: u8 = ADDRESS_BYTES * 8;

pub const ROM_CMD_SEARCH_ROM: u8 = 0xf0;
pub const ROM_CMD_READ_ROM: u8 = 0x33;
pub const ROM_CMD_MATCH_ROM: u8 = 0x55;
pub const ROM_CMD_SKIP_ROM: u8 = 0xcc;
pub const ROM_CMD_ALARM_SEARCH: u8 = 0xec;

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

    pub fn is_crc_ok(&self) -> bool {
        ensure_crc(&self.address[..])
    }
}

impl ::core::fmt::Debug for Device {
    fn fmt(&self, f: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
        write!(
            f,
            "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            self.address[0],
            self.address[6],
            self.address[5],
            self.address[4],
            self.address[3],
            self.address[2],
            self.address[1],
            // self.address[7],
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

pub struct DeviceSearch<'a, P: OneWirePinExt, D: DelayUs<u16>> {
    address: [u8; 8],
    discrepancies: [u8; 8],
    state: SearchState,
    port: &'a mut OneWire<P>,
    delay: &'a mut D,
}

impl<'a, P: OneWirePinExt, D: DelayUs<u16>> DeviceSearch<'a, P, D> {
    fn is_bit_set(array: &[u8], bit: u8) -> bool {
        if bit / 8 >= array.len() as u8 {
            return false;
        }
        let index = bit / 8;
        let offset = bit % 8;
        array[index as usize] & (0x01 << offset) != 0x00
    }

    fn is_bit_set_in_address(&self, bit: u8) -> bool {
        Self::is_bit_set(&self.address, bit)
    }

    fn set_bit_in_address(&mut self, bit: u8) {
        Self::set_bit(&mut self.address, bit);
    }

    fn reset_bit_in_address(&mut self, bit: u8) {
        Self::reset_bit(&mut self.address, bit);
    }

    fn write_bit_in_address(&mut self, bit: u8, value: bool) {
        if value {
            self.set_bit_in_address(bit);
        } else {
            self.reset_bit_in_address(bit);
        }
    }

    fn is_bit_set_in_discrepancies(&self, bit: u8) -> bool {
        Self::is_bit_set(&self.discrepancies, bit)
    }

    fn set_bit_in_discrepancy(&mut self, bit: u8) {
        Self::set_bit(&mut self.discrepancies, bit);
    }

    fn reset_bit_in_discrepancy(&mut self, bit: u8) {
        Self::reset_bit(&mut self.discrepancies, bit);
    }

    fn last_discrepancy(&self) -> Option<u8> {
        let mut result = None;
        for i in 0..ADDRESS_BITS {
            if self.is_bit_set_in_discrepancies(i) {
                result = Some(i);
            }
        }
        result
    }

    fn set_bit(array: &mut [u8], bit: u8) {
        if bit / 8 >= array.len() as u8 {
            return;
        }
        let index = bit / 8;
        let offset = bit % 8;
        array[index as usize] |= 0x01 << offset
    }

    fn reset_bit(array: &mut [u8], bit: u8) {
        if bit / 8 >= array.len() as u8 {
            return;
        }
        let index = bit / 8;
        let offset = bit % 8;
        array[index as usize] &= !(0x01 << offset)
    }
}

impl<'a, P: OneWirePinExt, D: DelayUs<u16>> core::iter::Iterator for DeviceSearch<'a, P, D> {
    type Item = Device;

    fn next(&mut self) -> Option<Self::Item> {
        if self.state == SearchState::End {
            return None;
        }

        let mut discrepancy_found = false;
        let last_discrepancy = self.last_discrepancy();

        if !self.port.reset(self.delay).is_ok() {
            return None;
        }

        self.port.write_byte(self.delay, ROM_CMD_SEARCH_ROM).ok()?;

        if let Some(last_discrepancy) = last_discrepancy {
            // walk previous path
            for i in 0..last_discrepancy {
                let bit0 = self.port.read_bit(self.delay).ok()?;
                let bit1 = self.port.read_bit(self.delay).ok()?;

                if bit0 && bit1 {
                    // no device responded
                    return None;
                } else {
                    let bit = self.is_bit_set_in_address(i);
                    // rom.write_bit_in_address(i, bit0);
                    // rom.write_bit_in_discrepancy(i, bit);
                    self.port.write_bit(self.delay, bit).ok()?;
                }
            }
        } else {
            // no discrepancy and device found, meaning the one found is the only one
            if self.state == SearchState::DeviceFound {
                self.state = SearchState::End;
                return None;
            }
        }

        for i in last_discrepancy.unwrap_or(0)..ADDRESS_BITS {
            let bit0 = self.port.read_bit(self.delay).ok()?; // normal bit
            let bit1 = self.port.read_bit(self.delay).ok()?; // complementar bit

            if last_discrepancy.eq(&Some(i)) {
                // be sure to go different path from before (go second path, thus writing 1)
                self.reset_bit_in_discrepancy(i);
                self.set_bit_in_address(i);
                self.port.write_bit(self.delay, true).ok()?;
            } else {
                if bit0 && bit1 {
                    // no response received
                    return None;
                }

                if !bit0 && !bit1 {
                    // addresses with 0 and 1
                    // found new path, go first path by default (thus writing 0)
                    discrepancy_found |= true;
                    self.set_bit_in_discrepancy(i);
                    self.reset_bit_in_address(i);
                    self.port.write_bit(self.delay, false).ok();
                } else {
                    // addresses only with bit0
                    self.write_bit_in_address(i, bit0);
                    self.port.write_bit(self.delay, bit0).ok();
                }
            }
        }

        if !discrepancy_found && self.last_discrepancy().is_none() {
            self.state = SearchState::End;
        } else {
            self.state = SearchState::DeviceFound;
        }
        Some(Device { address: self.address })
    }
}

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

    /// To identify all of the slave devices
    pub fn search_device<'a, D: DelayUs<u16>>(&'a mut self, delay: &'a mut D) -> DeviceSearch<'a, P, D> {
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

    pub fn read_bytes(&mut self, delay: &mut impl DelayUs<u16>, dst: &mut [u8]) -> Result<(), Error> {
        for d in dst {
            *d = self.read_byte(delay)?;
        }
        Ok(())
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

    fn write_bit(&mut self, delay: &mut impl DelayUs<u16>, high: bool) -> Result<(), Error> {
        // master write
        let pin = self.pin.to_output();
        pin.set_low().unwrap();
        delay.delay_us(if high { 10 } else { 65 });
        pin.set_high().unwrap();
        delay.delay_us(if high { 55 } else { 5 });
        Ok(())
    }
}

/// A device on wire.
pub struct OneWireDevice<'a, P: OneWirePinExt> {
    wire: &'a mut OneWire<P>,
    dev: Option<Device>,
}

impl<P: OneWirePinExt> OneWireDevice<'_, P> {
    fn select_device<D: DelayUs<u16>>(&mut self, delay: &mut D) -> Result<(), Error> {
        if let Some(ref dev) = self.dev {
            self.wire.reset(delay)?;
            self.wire.write_byte(delay, ROM_CMD_MATCH_ROM)?; // match rom
            self.wire.write_bytes(delay, dev)?;
            Ok(())
        } else {
            self.wire.reset(delay)?;
            self.wire.write_byte(delay, ROM_CMD_SKIP_ROM)?;
            Ok(())
        }
    }

    pub fn release(self) {}
}

pub fn ensure_crc(data: &[u8]) -> bool {
    compute_partial_crc8(0, data) == 0
}

fn compute_partial_crc8(crc: u8, data: &[u8]) -> u8 {
    let mut crc = crc;
    for byte in data.iter() {
        let mut byte = *byte;
        for _ in 0..8 {
            let mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if mix != 0x00 {
                crc ^= 0x8C;
            }
            byte >>= 1;
        }
    }
    crc
}
