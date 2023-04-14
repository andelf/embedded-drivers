//! BME280, BMP280 driver.
//!
//! via https://github.com/uber-foo/bme280-rs
//!
//! Humidity resolution: 0.008 %RH
//! Pressure resolution: 0.18 Pa
//! Temperature resolution: 0.01 C

use embedded_hal::delay::DelayUs;
use embedded_hal::i2c::I2c;

#[cfg(feature = "serde")]
use serde::Serialize;

const BME280_I2C_ADDR_PRIMARY: u8 = 0x76;
const BME280_I2C_ADDR_SECONDARY: u8 = 0x77;

const BME280_PWR_CTRL_ADDR: u8 = 0xF4;
const BME280_CTRL_HUM_ADDR: u8 = 0xF2;
const BME280_CTRL_MEAS_ADDR: u8 = 0xF4;
const BME280_CONFIG_ADDR: u8 = 0xF5;

const BME280_RESET_ADDR: u8 = 0xE0;
const BME280_SOFT_RESET_CMD: u8 = 0xB6;

const BME280_CHIP_ID: u8 = 0x60;
const BMP280_CHIP_ID: u8 = 0x58;
const BME280_CHIP_ID_ADDR: u8 = 0xD0;

const BME280_DATA_ADDR: u8 = 0xF7;
const BME280_P_T_H_DATA_LEN: usize = 8;

const BME280_P_T_CALIB_DATA_ADDR: u8 = 0x88;
const BME280_P_T_CALIB_DATA_LEN: usize = 26;

const BME280_H_CALIB_DATA_ADDR: u8 = 0xE1;
const BME280_H_CALIB_DATA_LEN: usize = 7;

const BME280_TEMP_MIN: f32 = -40.0;
const BME280_TEMP_MAX: f32 = 85.0;

const BME280_PRESSURE_MIN: f32 = 30000.0;
const BME280_PRESSURE_MAX: f32 = 110000.0;

const BME280_HUMIDITY_MIN: f32 = 0.0;
const BME280_HUMIDITY_MAX: f32 = 100.0;

const BME280_SLEEP_MODE: u8 = 0x00;
const BME280_FORCED_MODE: u8 = 0x01;
const BME280_NORMAL_MODE: u8 = 0x03;

const BME280_SENSOR_MODE_MSK: u8 = 0x03;

const BME280_CTRL_HUM_MSK: u8 = 0x07;

const BME280_CTRL_PRESS_MSK: u8 = 0x1C;
const BME280_CTRL_PRESS_POS: u8 = 0x02;

const BME280_CTRL_TEMP_MSK: u8 = 0xE0;
const BME280_CTRL_TEMP_POS: u8 = 0x05;

const BME280_FILTER_MSK: u8 = 0x1C;
const BME280_FILTER_POS: u8 = 0x02;
const BME280_FILTER_COEFF_16: u8 = 0x04;

const BME280_OVERSAMPLING_1X: u8 = 0x01;
const BME280_OVERSAMPLING_2X: u8 = 0x02;
const BME280_OVERSAMPLING_16X: u8 = 0x05;

macro_rules! concat_bytes {
    ($msb:expr, $lsb:expr) => {
        (($msb as u16) << 8) | ($lsb as u16)
    };
}

macro_rules! set_bits {
    ($reg_data:expr, $mask:expr, $pos:expr, $data:expr) => {
        ($reg_data & !$mask) | (($data << $pos) & $mask)
    };
}

/// BME280 errors
#[derive(Debug)]
pub enum Error<E> {
    /// Failed to compensate a raw measurement
    CompensationFailed,
    /// I²C bus error
    I2c(E),
    /// Failed to parse sensor data
    InvalidData,
    /// No calibration data is available (probably forgot to call or check BME280::init for failure)
    NoCalibrationData,
    /// Chip ID doesn't match expected value
    UnsupportedChip,
}

/// BME280 operating mode
#[derive(Debug, Copy, Clone)]
pub enum SensorMode {
    /// Sleep mode
    Sleep,
    /// Forced mode
    Forced,
    /// Normal mode
    Normal,
}

#[derive(Debug)]
struct CalibrationData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
    t_fine: i32,
}

/// Measurement data
#[cfg_attr(feature = "serde", derive(Serialize))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurements {
    /// temperature in degrees celsius
    pub temperature: f32,
    /// pressure in pascals
    pub pressure: f32,
    /// percent relative humidity (`0` with BMP280)
    pub humidity: f32,
}

impl Measurements {
    fn parse<E>(data: [u8; BME280_P_T_H_DATA_LEN], calibration: &mut CalibrationData) -> Result<Self, Error<E>> {
        let data_msb: u32 = (data[0] as u32) << 12;
        let data_lsb: u32 = (data[1] as u32) << 4;
        let data_xlsb: u32 = (data[2] as u32) >> 4;
        let pressure = data_msb | data_lsb | data_xlsb;

        let data_msb: u32 = (data[3] as u32) << 12;
        let data_lsb: u32 = (data[4] as u32) << 4;
        let data_xlsb: u32 = (data[5] as u32) >> 4;
        let temperature = data_msb | data_lsb | data_xlsb;

        let data_msb: u32 = (data[6] as u32) << 8;
        let data_lsb: u32 = data[7] as u32;
        let humidity = data_msb | data_lsb;

        let temperature = Measurements::compensate_temperature(temperature, calibration)?;
        let pressure = Measurements::compensate_pressure(pressure, calibration)?;
        let humidity = Measurements::compensate_humidity(humidity, calibration)?;

        Ok(Measurements {
            temperature,
            pressure,
            humidity,
        })
    }

    fn compensate_temperature<E>(uncompensated: u32, calibration: &mut CalibrationData) -> Result<f32, Error<E>> {
        let var1: f32 = uncompensated as f32 / 16384.0 - calibration.dig_t1 as f32 / 1024.0;
        let var1 = var1 * calibration.dig_t2 as f32;
        let var2 = uncompensated as f32 / 131072.0 - calibration.dig_t1 as f32 / 8192.0;
        let var2 = var2 * var2 * calibration.dig_t3 as f32;

        calibration.t_fine = (var1 + var2) as i32;

        let temperature = (var1 + var2) / 5120.0;
        let temperature = if temperature < BME280_TEMP_MIN {
            BME280_TEMP_MIN
        } else if temperature > BME280_TEMP_MAX {
            BME280_TEMP_MAX
        } else {
            temperature
        };
        Ok(temperature)
    }

    fn compensate_pressure<E>(uncompensated: u32, calibration: &mut CalibrationData) -> Result<f32, Error<E>> {
        let var1: f32 = calibration.t_fine as f32 / 2.0 - 64000.0;
        let var2: f32 = var1 * var1 * calibration.dig_p6 as f32 / 32768.0;
        let var2: f32 = var2 + var1 * calibration.dig_p5 as f32 * 2.0;
        let var2: f32 = var2 / 4.0 + calibration.dig_p4 as f32 * 65536.0;
        let var3: f32 = calibration.dig_p3 as f32 * var1 * var1 / 524288.0;
        let var1: f32 = (var3 + calibration.dig_p2 as f32 * var1) / 524288.0;
        let var1: f32 = (1.0 + var1 / 32768.0) * calibration.dig_p1 as f32;

        let pressure = if var1 > 0.0 {
            let pressure: f32 = 1048576.0 - uncompensated as f32;
            let pressure: f32 = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
            let var1: f32 = calibration.dig_p9 as f32 * pressure * pressure / 2147483648.0;
            let var2: f32 = pressure * calibration.dig_p8 as f32 / 32768.0;
            let pressure: f32 = pressure + (var1 + var2 + calibration.dig_p7 as f32) / 16.0;
            if pressure < BME280_PRESSURE_MIN {
                BME280_PRESSURE_MIN
            } else if pressure > BME280_PRESSURE_MAX {
                BME280_PRESSURE_MAX
            } else {
                pressure
            }
        } else {
            return Err(Error::InvalidData);
        };
        Ok(pressure)
    }

    fn compensate_humidity<E>(uncompensated: u32, calibration: &mut CalibrationData) -> Result<f32, Error<E>> {
        let var1: f32 = calibration.t_fine as f32 - 76800.0;
        let var2: f32 = calibration.dig_h4 as f32 * 64.0 + (calibration.dig_h5 as f32 / 16384.0) * var1;
        let var3: f32 = uncompensated as f32 - var2;
        let var4: f32 = calibration.dig_h2 as f32 / 65536.0;
        let var5: f32 = 1.0 + (calibration.dig_h3 as f32 / 67108864.0) * var1;
        let var6: f32 = 1.0 + (calibration.dig_h6 as f32 / 67108864.0) * var1 * var5;
        let var6: f32 = var3 * var4 * (var5 * var6);

        let humidity: f32 = var6 * (1.0 - calibration.dig_h1 as f32 * var6 / 524288.0);
        let humidity = if humidity < BME280_HUMIDITY_MIN {
            BME280_HUMIDITY_MIN
        } else if humidity > BME280_HUMIDITY_MAX {
            BME280_HUMIDITY_MAX
        } else {
            humidity
        };
        Ok(humidity)
    }
}

/// Representation of a BME280
#[derive(Debug, Default)]
pub struct BME280<I2C> {
    /// concrete I²C device implementation
    i2c: I2C,
    /// I²C device address
    address: u8,
    /// calibration data
    calibration: Option<CalibrationData>,
}

impl<I2C> BME280<I2C>
where
    I2C: I2c,
{
    /// Create a new BME280 struct using the primary I²C address `0x76`
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, BME280_I2C_ADDR_PRIMARY)
    }

    /// Create a new BME280 struct using the secondary I²C address `0x77`
    pub fn new_secondary(i2c: I2C) -> Self {
        Self::new(i2c, BME280_I2C_ADDR_SECONDARY)
    }

    /// Create a new BME280 struct using a custom I²C address
    pub fn new(i2c: I2C, address: u8) -> Self {
        BME280 {
            i2c,
            address,
            calibration: None,
        }
    }

    /// Initializes the BME280
    pub fn init<D: DelayUs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.verify_chip_id()?;
        self.soft_reset(delay)?;
        self.calibrate()?;
        self.configure(delay)
    }

    #[deprecated = "user `embedded-hal-bus` instead"]
    pub fn release(self) -> I2C {
        self.i2c
    }

    fn verify_chip_id(&mut self) -> Result<(), Error<I2C::Error>> {
        let chip_id = self.read_register(BME280_CHIP_ID_ADDR)?;
        if chip_id == BME280_CHIP_ID || chip_id == BMP280_CHIP_ID {
            Ok(())
        } else {
            Err(Error::UnsupportedChip)
        }
    }

    fn soft_reset<D: DelayUs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.write_register(BME280_RESET_ADDR, BME280_SOFT_RESET_CMD)?;
        delay.delay_ms(2); // startup time is 2ms
        Ok(())
    }

    fn calibrate(&mut self) -> Result<(), Error<I2C::Error>> {
        let pt_calib_data = self.read_pt_calib_data(BME280_P_T_CALIB_DATA_ADDR)?;
        let h_calib_data = self.read_h_calib_data(BME280_H_CALIB_DATA_ADDR)?;
        self.calibration = Some(parse_calib_data(&pt_calib_data, &h_calib_data));
        Ok(())
    }

    fn configure<D: DelayUs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        match self.mode()? {
            SensorMode::Sleep => {}
            _ => self.soft_reset(delay)?,
        };

        self.write_register(BME280_CTRL_HUM_ADDR, BME280_OVERSAMPLING_1X & BME280_CTRL_HUM_MSK)?;
        let ctrl_meas = self.read_register(BME280_CTRL_MEAS_ADDR)?;
        self.write_register(BME280_CTRL_MEAS_ADDR, ctrl_meas)?;

        let data = self.read_register(BME280_CTRL_MEAS_ADDR)?;
        let data = set_bits!(
            data,
            BME280_CTRL_PRESS_MSK,
            BME280_CTRL_PRESS_POS,
            BME280_OVERSAMPLING_16X
        );
        let data = set_bits!(data, BME280_CTRL_TEMP_MSK, BME280_CTRL_TEMP_POS, BME280_OVERSAMPLING_2X);
        self.write_register(BME280_CTRL_MEAS_ADDR, data)?;

        let data = self.read_register(BME280_CONFIG_ADDR)?;
        let data = set_bits!(data, BME280_FILTER_MSK, BME280_FILTER_POS, BME280_FILTER_COEFF_16);
        self.write_register(BME280_CONFIG_ADDR, data)
    }

    fn mode(&mut self) -> Result<SensorMode, Error<I2C::Error>> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[BME280_PWR_CTRL_ADDR], &mut data)
            .map_err(Error::I2c)?;
        match data[0] & BME280_SENSOR_MODE_MSK {
            BME280_SLEEP_MODE => Ok(SensorMode::Sleep),
            BME280_FORCED_MODE => Ok(SensorMode::Forced),
            BME280_NORMAL_MODE => Ok(SensorMode::Normal),
            _ => Err(Error::InvalidData),
        }
    }

    fn forced<D: DelayUs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.set_mode(BME280_FORCED_MODE, delay)
    }

    fn set_mode<D: DelayUs>(&mut self, mode: u8, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        match self.mode()? {
            SensorMode::Sleep => {}
            _ => self.soft_reset(delay)?,
        };
        let data = self.read_register(BME280_PWR_CTRL_ADDR)?;
        let data = set_bits!(data, BME280_SENSOR_MODE_MSK, 0, mode);
        self.write_register(BME280_PWR_CTRL_ADDR, data)
    }

    /// Captures and processes sensor data for temperature, pressure, and humidity
    pub fn measure<D: DelayUs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I2C::Error>> {
        self.forced(delay)?;
        delay.delay_ms(40); // await measurement
        let measurements = self.read_data(BME280_DATA_ADDR)?;
        match self.calibration.as_mut() {
            Some(calibration) => {
                let measurements = Measurements::parse(measurements, &mut *calibration)?;
                Ok(measurements)
            }
            None => Err(Error::NoCalibrationData),
        }
    }

    fn read_register(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data[0])
    }

    fn read_data(&mut self, register: u8) -> Result<[u8; BME280_P_T_H_DATA_LEN], Error<I2C::Error>> {
        let mut data: [u8; BME280_P_T_H_DATA_LEN] = [0; BME280_P_T_H_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data)
    }

    fn read_pt_calib_data(&mut self, register: u8) -> Result<[u8; BME280_P_T_CALIB_DATA_LEN], Error<I2C::Error>> {
        let mut data: [u8; BME280_P_T_CALIB_DATA_LEN] = [0; BME280_P_T_CALIB_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data)
    }

    fn read_h_calib_data(&mut self, register: u8) -> Result<[u8; BME280_H_CALIB_DATA_LEN], Error<I2C::Error>> {
        let mut data: [u8; BME280_H_CALIB_DATA_LEN] = [0; BME280_H_CALIB_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data)
    }

    fn write_register(&mut self, register: u8, payload: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, &[register, payload]).map_err(Error::I2c)
    }
}

fn parse_calib_data(
    pt_data: &[u8; BME280_P_T_CALIB_DATA_LEN],
    h_data: &[u8; BME280_H_CALIB_DATA_LEN],
) -> CalibrationData {
    let dig_t1 = concat_bytes!(pt_data[1], pt_data[0]);
    let dig_t2 = concat_bytes!(pt_data[3], pt_data[2]) as i16;
    let dig_t3 = concat_bytes!(pt_data[5], pt_data[4]) as i16;
    let dig_p1 = concat_bytes!(pt_data[7], pt_data[6]);
    let dig_p2 = concat_bytes!(pt_data[9], pt_data[8]) as i16;
    let dig_p3 = concat_bytes!(pt_data[11], pt_data[10]) as i16;
    let dig_p4 = concat_bytes!(pt_data[13], pt_data[12]) as i16;
    let dig_p5 = concat_bytes!(pt_data[15], pt_data[14]) as i16;
    let dig_p6 = concat_bytes!(pt_data[17], pt_data[16]) as i16;
    let dig_p7 = concat_bytes!(pt_data[19], pt_data[18]) as i16;
    let dig_p8 = concat_bytes!(pt_data[21], pt_data[20]) as i16;
    let dig_p9 = concat_bytes!(pt_data[23], pt_data[22]) as i16;
    let dig_h1 = pt_data[25];
    let dig_h2 = concat_bytes!(h_data[1], h_data[0]) as i16;
    let dig_h3 = h_data[2];
    let dig_h4 = (h_data[3] as i16 * 16) | ((h_data[4] as i16) & 0x0F);
    let dig_h5 = (h_data[5] as i16 * 16) | ((h_data[4] as i16) >> 4);
    let dig_h6 = h_data[6] as i8;

    CalibrationData {
        dig_t1,
        dig_t2,
        dig_t3,
        dig_p1,
        dig_p2,
        dig_p3,
        dig_p4,
        dig_p5,
        dig_p6,
        dig_p7,
        dig_p8,
        dig_p9,
        dig_h1,
        dig_h2,
        dig_h3,
        dig_h4,
        dig_h5,
        dig_h6,
        t_fine: 0,
    }
}
