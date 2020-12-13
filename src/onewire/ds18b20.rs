/// DS18B20 digital thermometer.
pub struct DS18B20;

/// Initiates a single temperature conversion.
pub const CMD_CONVERT_T: u8 = 0x44;
/// Writes 3 bytes of data to the DS18B20’s scratchpad.
pub const CMD_WRITE_SCRATCHPAD: u8 = 0x4e;
/// Reads the contents of the scratchpad.
pub const CMD_READ_SCRATCHPAD: u8 = 0xbe;
/// Copies the contents of the scratchpad T_H, T_L and configuration registers
/// (bytes 2, 3 and 4) to EEPROM.
pub const CMD_COPY_SCRATCHPAD: u8 = 0x48;
/// Recalls the alarm trigger values and configuration data from EEPROM and
/// places the data in bytes 2, 3, and 4, respectively, in the scratchpad memory.
pub const CMD_RECALL_EEPROM: u8 = 0xb8;
/// Determines if any DS18B20s on the bus are using parasite power.
pub const CMD_POWER_SUPPLY: u8 = 0xb4;