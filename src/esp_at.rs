//! Driver for esp-at.
//!
//! Ref: https://docs.espressif.com/projects/esp-at/en/latest/AT_Command_Set/index.html

use core::fmt::Write;
use core::str;
use embedded_hal::serial;
use nb::block;

use crate::ByteMutWriter;

const CR: u8 = 0x0d;
const LF: u8 = 0x0a;

#[derive(Debug)]
pub enum EspAtError<ER, EW> {
    Busy,
    BufOverflow,
    NoConnection,
    Error,
    DirtyData,
    Read,
    SerialRead(ER),
    SerialWrite(EW),
}

impl<ER, EW> EspAtError<ER, EW> {
    fn from_read(err: ER) -> Self {
        EspAtError::SerialRead(err)
    }

    fn from_write(err: EW) -> Self {
        EspAtError::SerialWrite(err)
    }
}

impl<ER, EW> From<core::convert::Infallible> for EspAtError<ER, EW> {
    fn from(_err: core::convert::Infallible) -> Self {
        unreachable!()
    }
}

/// esp-at driver for ESP32, ESP8266, ESP8285.
pub struct EspAt<S> {
    serial: S,
    cursor: usize,
    read_buf: [u8; 1024],
}

impl<S, ER, EW> EspAt<S>
where
    S: serial::Read<u8, Error = ER> + serial::Write<u8, Error = EW>,
{
    pub fn new(serial: S) -> Self {
        Self {
            serial,
            cursor: 0,
            read_buf: [0u8; 1024],
        }
    }

    fn write_byte(&mut self, byte: u8) -> Result<(), EspAtError<ER, EW>> {
        block!(self.serial.write(byte)).map_err(EspAtError::from_write)
    }

    fn write_crlf(&mut self) -> Result<(), EspAtError<ER, EW>> {
        self.write_byte(CR)?;
        self.write_byte(LF)
    }

    fn write_all(&mut self, buf: &[u8]) -> Result<(), EspAtError<ER, EW>> {
        for byte in buf {
            self.write_byte(*byte)?;
        }
        Ok(())
    }

    fn write_str(&mut self, buf: &str) -> Result<(), EspAtError<ER, EW>> {
        for byte in buf.as_bytes() {
            self.write_byte(*byte)?;
        }
        Ok(())
    }

    fn write_quoted_str(&mut self, buf: &str) -> Result<(), EspAtError<ER, EW>> {
        self.write_byte(b'"')?;
        self.write_str(buf)?;
        self.write_byte(b'"')
    }

    pub fn send_command(&mut self, cmd: &str) -> Result<(), EspAtError<ER, EW>> {
        self.write_all(cmd.as_bytes())?;
        self.write_crlf()?;
        Ok(())
    }

    fn read_byte(&mut self) -> Result<u8, EspAtError<ER, EW>> {
        block!(self.serial.read()).map_err(EspAtError::from_read)
    }

    pub fn last_read(&self) -> &str {
        unsafe { str::from_utf8_unchecked(&self.read_buf[0..self.cursor]) }
    }

    pub fn read_response(&mut self) -> Result<&str, EspAtError<ER, EW>> {
        let buflen = self.read_buf.len();
        self.cursor = 0;
        let mut i = 0;
        loop {
            match self.read_byte()? {
                LF if i >= 1 && self.read_buf[i - 1] == CR => {
                    if i >= 3 && &self.read_buf[i - 3..i - 1] == b"OK" {
                        let resp = unsafe { str::from_utf8_unchecked(&self.read_buf[0..(i - 3)]) };
                        self.cursor = i - 3;
                        return Ok(resp.trim_end());
                    }
                    if i >= 6 && &self.read_buf[i - 6..i - 1] == b"ERROR" {
                        self.cursor = i - 6;
                        return Err(EspAtError::Error);
                    }
                    if i >= 10 && &self.read_buf[i - 10..i - 1] == b"busy p..." {
                        return Err(EspAtError::Busy);
                    }
                    self.read_buf[i] = LF;
                }
                CR => {
                    self.read_buf[i] = CR;
                }
                other => {
                    self.read_buf[i] = other;
                }
            }
            i += 1;
            if i >= buflen {
                self.skip_to_next();
                return Err(EspAtError::BufOverflow);
            }
        }
    }

    fn read_until(&mut self, c: u8) -> Result<usize, EspAtError<ER, EW>> {
        let buflen = self.read_buf.len();
        let mut i = 0;
        loop {
            let b = self.read_byte()?;
            self.read_buf[i] = b;
            if b == c {
                return Ok(i);
            }
            i += 1;
            if i >= buflen {
                self.skip_to_next();
                return Err(EspAtError::BufOverflow);
            }
        }
    }

    fn read_nbytes(&mut self, n: usize) -> Result<&[u8], EspAtError<ER, EW>> {
        if n > self.read_buf.len() {
            for _ in 0..n {
                self.read_byte()?;
            }
            return Err(EspAtError::BufOverflow);
        }

        for i in 0..n {
            self.read_buf[i] = self.read_byte()?;
        }
        Ok(&self.read_buf[0..n])
    }

    pub fn skip_to_next(&mut self) {
        loop {
            match self.serial.read() {
                Err(nb::Error::WouldBlock) => {
                    return;
                }
                Err(_) => {
                    continue;
                }
                Ok(_) => {
                    continue;
                }
            }
        }
    }

    /// ATE0
    pub fn echo_off(&mut self) -> Result<(), EspAtError<ER, EW>> {
        self.write_str("ATE0")?;
        self.write_crlf()?;

        self.read_response().map(|_| ())
    }

    /// ATE1
    pub fn echo_on(&mut self) -> Result<(), EspAtError<ER, EW>> {
        self.write_str("ATE1")?;
        self.write_crlf()?;

        self.read_response().map(|_| ())
    }

    // enter station mode
    /*
    drv.send_command("AT+CWMODE=1").unwrap();
    let raw = drv.read_response();
    writeln!(buf, "AT+CWMODE=>\n{:?}", raw.unwrap());
    */

    // AT+CIPSTA?
    // AT+CIPSTA=<ip>[,<gateway>,<netmask>]
    /*
    +CIPSTA:ip:"192.168.1.9"
    +CIPSTA:gateway:"192.168.1.1"
    +CIPSTA:netmask:"255.255.0.0"
    */
    /// (ip, gateway, netmask)
    pub fn ifconfig(&mut self) -> Result<(&str, &str, &str), EspAtError<ER, EW>> {
        self.write_str("AT+CIPSTA?")?;
        self.write_crlf()?;

        self.read_response().map(|payload| {
            let mut it = payload.lines();
            let ip = it.next().unwrap();
            let gateway = it.next().unwrap();
            let netmask = it.next().unwrap();

            (
                &ip[12..ip.len() - 1],
                &gateway[17..gateway.len() - 1],
                &netmask[17..netmask.len() - 1],
            )
        })
    }

    // AT+CWJAP?
    // AT+CWJAP=<ssid>,<pwd>[,<bssid>][,<pci_en>][,<reconn_interval>][,<listen_interval>][,<scan_mode>]
    /// <ssid>,<bssid>,<channel>,<rssi>,<pci_en>,<reconn_interval>,<listen_interval>,<scan_mode>
    pub fn iwconfig(&mut self) -> Result<(&str, &str, u8), EspAtError<ER, EW>> {
        self.write_str("AT+CWJAP?")?;
        self.write_crlf()?;

        // +CWJAP:"...","04:d9:f5:c4:93:98",11,-68,0,0,0,0
        self.read_response().and_then(|payload| {
            if payload == "No AP" {
                return Err(EspAtError::NoConnection);
            }
            let mut it = payload[7..].split(',');
            let ssid = it.next().unwrap().trim_matches('"');
            let bssid = it.next().unwrap().trim_matches('"');
            let channel = it.next().unwrap().parse().unwrap_or(255);
            Ok((ssid, bssid, channel))
        })
    }

    // AT+CWJAP="apname","password"
    pub fn cwjap(&mut self, ssid: &str, password: &str) -> Result<(), EspAtError<ER, EW>> {
        self.write_all(b"AT+CWJAP=")?;
        self.write_quoted_str(ssid)?;
        self.write_byte(b',')?;
        self.write_quoted_str(password)?;
        self.write_crlf()?;

        self.read_response().map(|_| ())
    }

    /// AT+PING
    pub fn ping(&mut self, host: &str) -> Result<u32, EspAtError<ER, EW>> {
        self.write_all(b"AT+PING=")?;
        self.write_quoted_str(host)?;
        self.write_crlf()?;

        self.read_response().map(|payload| {
            if payload.starts_with("+PING:") {
                payload[6..].parse().unwrap_or(255)
            } else {
                payload[1..].parse().unwrap_or(255)
            }
        })
    }

    pub fn tcp_send(&mut self, host: &str, port: u16, buf: &[u8]) -> Result<(), EspAtError<ER, EW>> {
        self.connect_tcp(host, port).map_err(|_| EspAtError::NoConnection)?;
        self.send(buf)?;
        // read 1 packet
        let _ = self.read();
        // read all remain packets
        self.skip_read()?;
        self.close()
    }

    pub fn connect_tcp(&mut self, host: &str, port: u16) -> Result<(), EspAtError<ER, EW>> {
        self.write_all(b"AT+CIPSTART=\"TCP\",")?;
        self.write_quoted_str(host)?;
        self.write_byte(b',')?;
        {
            let mut data = [0u8; 6];
            let mut buf = ByteMutWriter::new(&mut data[..]);
            let _ = write!(buf, "{}", port);
            self.write_str(buf.as_str())?;
        }
        self.write_crlf()?;

        self.read_response().map(|_| ()).or_else(|err| {
            if self.last_read().contains("ALREADY CONNECTED") {
                Ok(())
            } else {
                Err(err)
            }
        })
    }

    /// AT+CIPSEND=x
    pub fn send(&mut self, data: &[u8]) -> Result<(), EspAtError<ER, EW>> {
        {
            let mut buf = [0u8; 24];
            let mut buf = ByteMutWriter::new(&mut buf[..]);
            let _ = write!(buf, "AT+CIPSEND={}", data.len());
            self.write_str(buf.as_str())?;
        }
        self.write_crlf()?;
        self.read_until(b'>')?;
        self.write_all(data)?;

        self.read_response().map(|_| ())
    }

    /// Skip all possible data.
    /// Skip and send "AT", if it returns OK, then all read data is received.
    pub fn skip_read(&mut self) -> Result<(), EspAtError<ER, EW>> {
        loop {
            // TODO: use delay
            for _ in 0..5_000_000 {
                unsafe {
                    asm!("nop");
                }
            }
            self.skip_to_next();

            let _ = self.send_command("AT");
            if let Ok(_) = self.read_response() {
                break;
            }
        }
        Ok(())
    }

    /// Read a packet
    pub fn read(&mut self) -> Result<&[u8], EspAtError<ER, EW>> {
        // +IPD,103: ...
        self.read_until(b'+')?;
        let cursor = self.read_until(b':')?;
        let nbytes = unsafe {
            str::from_utf8_unchecked(&self.read_buf[4..cursor])
                .parse()
                .map_err(|_| EspAtError::DirtyData)?
        };

        self.read_nbytes(nbytes)
    }

    /// AT+CIPCLOSE
    pub fn close(&mut self) -> Result<(), EspAtError<ER, EW>> {
        self.skip_to_next();

        self.write_all(b"AT+CIPCLOSE")?;
        self.write_crlf()?;

        self.read_response().map(|_| ())
    }

    /// AT+HTTPCLIENT
    pub fn http_get(&mut self, url: &str) -> Result<&str, EspAtError<ER, EW>> {
        self.write_str("AT+HTTPCLIENT=2,0,")?;
        self.write_quoted_str(url)?;
        self.write_str(",,,")?;
        if url.starts_with("https") {
            self.write_byte(b'2')?;
        } else {
            self.write_byte(b'1')?;
        }
        self.write_crlf()?;

        // FIXME: trimed data
        self.read_response().map(|payload| {
            payload
                .bytes()
                .position(|c| c == b',')
                .map(|pos| &payload[pos + 1..])
                .unwrap_or("")
        })
    }

    pub fn http_post(&mut self, url: &str, data: &str) -> Result<&str, EspAtError<ER, EW>> {
        self.write_str("AT+HTTPCLIENT=3,0,")?;
        self.write_quoted_str(url)?;
        self.write_str(",,,")?;
        if url.starts_with("https") {
            self.write_str("2,")?;
        } else {
            self.write_str("1,")?;
        }
        self.write_quoted_str(data)?;
        self.write_crlf()?;

        self.read_response().map(|payload| {
            payload
                .bytes()
                .position(|c| c == b',')
                .map(|pos| &payload[pos + 1..])
                .unwrap_or("")
        })
    }
}
