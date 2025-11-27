# Sensor Reader Module Documentation

## Overview

The `SensorReader` class is a utility module that handles all I2C communication with temperature sensors through the PCA9548A I2C multiplexer. It abstracts the complexity of multiplexer channel switching and sensor-specific read protocols.

## Purpose

- Manage I2C bus connection lifecycle
- Control PCA9548A multiplexer channel selection
- Read SHT3x temperature/humidity sensors
- Read BME280/BMP280 temperature/pressure sensors  
- Read Raspberry Pi CPU temperature
- Provide unified interface for all sensor types

## Class: SensorReader

### Initialization

```python
sensor_reader = SensorReader(i2c_bus=1, mux_address=0x70)
```

**Parameters:**
- `i2c_bus` (int): I2C bus number (default: 1, typically `/dev/i2c-1`)
- `mux_address` (int): PCA9548A multiplexer I2C address (default: `0x70`)

### Methods

#### connect()

Initialize and open the I2C bus connection.

```python
if sensor_reader.connect():
    print("Connected successfully")
```

**Returns:** `bool` - `True` if connection successful, `False` otherwise

**Usage:** Call once during node initialization before any sensor operations.

#### disconnect()

Close the I2C bus connection.

```python
sensor_reader.disconnect()
```

**Usage:** Called during node shutdown to cleanly close I2C bus.

#### select_mux_channel(channel)

Switch the PCA9548A multiplexer to a specific channel.

```python
sensor_reader.select_mux_channel(2)  # Select channel 2
```

**Parameters:**
- `channel` (int): Channel number (0-7)

**Returns:** `bool` - `True` if successful

**How it works:**
- Writes bitmask `1 << channel` to multiplexer
- E.g., channel 2 → `0b00000100` (0x04)
- Waits 20ms for stabilization

#### disable_mux_channels()

Disable all multiplexer channels to release the I2C bus.

```python
sensor_reader.disable_mux_channels()
```

**Usage:** Call after reading all sensors to free the bus for direct-connected devices (e.g., OLED).

**How it works:**
- Writes `0x00` to multiplexer
- All channels turned off
- Devices behind mux become inaccessible until a channel is selected again

#### read_sht3x(address, channel)

Read temperature and humidity from SHT3x sensor.

```python
result = sensor_reader.read_sht3x(0x44, 0)
# {
#   'temperature': 29.42,
#   'humidity': 34.99,
#   'success': True
# }
```

**Parameters:**
- `address` (int): I2C address of SHT3x (typically `0x44` or `0x45`)
- `channel` (int): Multiplexer channel (0-7)

**Returns:** `dict`
```python
{
    'temperature': float,  # Celsius
    'humidity': float,     # Relative humidity %
    'success': bool
}
# OR on error:
{
    'success': False,
    'error': str
}
```

**Sensor Protocol:**
1. Select mux channel
2. Write measurement trigger: `0x2400` (high repeatability, no clock stretch)
3. Wait 50ms for conversion
4. Read 6 bytes: temp MSB, temp LSB, temp CRC, hum MSB, hum LSB, hum CRC
5. Convert raw values:
   - Temperature: `-45 + (175 × raw / 65535)`
   - Humidity: `100 × raw / 65535`

#### read_bme280(address, channel)

Read temperature and pressure from BME280/BMP280 sensor.

```python
result = sensor_reader.read_bme280(0x76, 1)
# {
#   'temperature': 34.07,
#   'pressure': 1014.62,
#   'chip': 'BMP280',
#   'success': True
# }
```

**Parameters:**
- `address` (int): I2C address (typically `0x76` or `0x77`)
- `channel` (int): Multiplexer channel (0-7)

**Returns:** `dict`
```python
{
    'temperature': float,  # Celsius
    'pressure': float,     # Hectopascals (hPa)
    'chip': str,           # "BME280" or "BMP280"
    'success': bool
}
```

**Chip Detection:**
- Reads chip ID register `0xD0`
- `0x60` → BME280 (temp + pressure + humidity)
- `0x58` → BMP280 (temp + pressure only)

**Sensor Protocol:**
1. Select mux channel
2. Read chip ID to determine variant
3. Perform soft reset
4. Read calibration data (24-31 bytes depending on chip)
5. Configure oversampling and standby time
6. Wait 100ms for conversion
7. Read raw ADC values (temperature and pressure)
8. Apply calibration formula to convert to real units

**Calibration Formula:**

Temperature uses 3 calibration coefficients (dig_T1, dig_T2, dig_T3):
```
var1 = ((adc_t/16384.0) - (dig_T1/1024.0)) * dig_T2
var2 = (((adc_t/131072.0) - (dig_T1/8192.0))²) * dig_T3
t_fine = var1 + var2
temp_c = t_fine / 5120.0
```

Pressure uses 9 calibration coefficients and the t_fine value from temperature calculation (see code for full formula).

#### read_cpu_temp()

Read Raspberry Pi CPU temperature from sysfs.

```python
result = sensor_reader.read_cpu_temp()
# {
#   'temperature': 49.05,
#   'success': True
# }
```

**Returns:** `dict`
```python
{
    'temperature': float,  # Celsius
    'success': bool
}
```

**Implementation:**
- Reads `/sys/class/thermal/thermal_zone0/temp`
- Value is in millidegrees Celsius
- Converts: `temp_c = value / 1000.0`

**Note:** Does not require I2C or multiplexer - direct filesystem access.

#### read_sensor(sensor_config)

Unified interface to read any sensor based on config dictionary.

```python
sensor_config = {
    'name': 'power_supply',
    'type': 'sht3x',
    'channel': 0,
    'address': '0x44'
}

result = sensor_reader.read_sensor(sensor_config)
```

**Parameters:**
- `sensor_config` (dict): Configuration with `type`, `address`, `channel` keys

**Returns:** Result dict from appropriate read method

**Supported Types:**
- `'cpu'` → `read_cpu_temp()`
- `'sht3x'` → `read_sht3x()`
- `'bme280'` → `read_bme280()`

## Hardware Details

### PCA9548A I2C Multiplexer

**Purpose:** Allows multiple sensors with the same I2C address on separate channels

**Channel Control Register:**

| Bit | Channel |
|-----|---------|
| 0   | Channel 0 |
| 1   | Channel 1 |
| 2   | Channel 2 |
| 3   | Channel 3 |
| 4   | Channel 4 |
| 5   | Channel 5 |
| 6   | Channel 6 |
| 7   | Channel 7 |

**Examples:**
```python
0x00  # All channels disabled
0x01  # Channel 0 enabled
0x04  # Channel 2 enabled
0xFF  # All channels enabled (not recommended)
```

**Only one channel should be active at a time** for proper sensor isolation.

### SHT3x Sensors

**Specifications:**
- Temperature range: -40°C to 125°C
- Temperature accuracy: ±0.2°C
- Humidity range: 0-100% RH
- Humidity accuracy: ±2% RH
- I2C address: 0x44 (ADDR pin low) or 0x45 (ADDR pin high)

**Typical Applications:**
- Ambient temperature monitoring
- Power supply temperature (heat dissipation)
- Enclosed environments

### BME280/BMP280 Sensors

**BME280 Specifications:**
- Temperature range: -40°C to 85°C
- Temperature accuracy: ±1°C
- Pressure range: 300-1100 hPa
- Pressure accuracy: ±1 hPa
- Humidity: 0-100% RH (BME280 only)
- I2C address: 0x76 (SDO low) or 0x77 (SDO high)

**BMP280** is the same but without humidity sensor.

**Typical Applications:**
- Case ambient temperature
- Stepper driver area temperature
- Atmospheric pressure (altitude estimation)

### Raspberry Pi CPU Temperature

**Reading Method:** Linux thermal zone sysfs interface

**Location:** `/sys/class/thermal/thermal_zone0/temp`

**Throttling Thresholds:**
- 60°C: Minor throttling may begin
- 80°C: Significant CPU frequency reduction
- 85°C: Emergency shutdown

## Error Handling

### Connection Errors

```python
if not sensor_reader.connect():
    # I2C bus unavailable
    # Check permissions: user in i2c group?
    # Check hardware: i2cdetect -y 1
```

### Channel Selection Errors

```python
if not sensor_reader.select_mux_channel(channel):
    # Multiplexer not responding
    # Wrong address? Default is 0x70
    # Check: i2cdetect -y 1
```

### Sensor Read Errors

```python
result = sensor_reader.read_sht3x(0x44, 0)
if not result.get('success'):
    error_msg = result.get('error')
    # Sensor not responding
    # Wrong channel?
    # Sensor not powered?
    # Wiring issue?
```

**Common Error Codes:**
- `[Errno 121] Remote I/O error`: Sensor not responding at address
- `[Errno 5] Input/output error`: I2C bus communication failure
- `[Errno 16] Device or resource busy`: Another process using I2C

## Usage Example

```python
from sensor_reader import SensorReader

# Initialize
reader = SensorReader(i2c_bus=1, mux_address=0x70)

if reader.connect():
    # Read SHT3x on channel 0
    result = reader.read_sht3x(0x44, 0)
    if result['success']:
        print(f"Temp: {result['temperature']}°C")
        print(f"Humidity: {result['humidity']}%")
    
    # Read BME280 on channel 1
    result = reader.read_bme280(0x76, 1)
    if result['success']:
        print(f"Temp: {result['temperature']}°C")
        print(f"Pressure: {result['pressure']} hPa")
    
    # Read CPU temp (no channel needed)
    result = reader.read_cpu_temp()
    if result['success']:
        print(f"CPU: {result['temperature']}°C")
    
    # Disable mux to free bus for other devices
    reader.disable_mux_channels()
    
    # Cleanup
    reader.disconnect()
```

## Integration with Hardware Node

```python
# In hardware_node.py:

self.sensor_reader = SensorReader(
    i2c_bus=self.config['settings']['i2c_bus'],
    mux_address=int(self.config['settings']['mux_address'], 16)
)

if self.sensor_reader.connect():
    # In read loop:
    for sensor_config in self.config['sensors']:
        reading = self.sensor_reader.read_sensor(sensor_config)
        
        if reading and reading.get('success'):
            # Publish to ROS topic
            self.publish_temperature(sensor_config['name'], reading['temperature'])
    
    # After all sensors read:
    self.sensor_reader.disable_mux_channels()
```

## Troubleshooting

### No Sensors Detected

```bash
# Check I2C bus
sudo i2cdetect -y 1

# Should show multiplexer at 0x70
# Sensors won't appear unless mux channel is selected
```

### Multiplexer Not Responding

1. **Check wiring:**
   - SDA to GPIO 2 (pin 3)
   - SCL to GPIO 3 (pin 5)
   - Power (3.3V) and ground

2. **Check address:**
   - Default: 0x70
   - Can be 0x70-0x77 depending on A0-A2 pins

3. **Check permissions:**
   ```bash
   sudo usermod -a -G i2c $USER
   # Logout and login again
   ```

### Sensor Read Failures

1. **Verify channel:**
   - Sensor must be on correct mux channel
   - Only one channel active at a time

2. **Check sensor address:**
   - SHT3x: 0x44 or 0x45
   - BME280: 0x76 or 0x77

3. **Power cycling:**
   - Sensors may need reset
   - Try power cycle or mux reset

### CPU Temperature Always Same

- Reading cached value
- Check file updates: `watch cat /sys/class/thermal/thermal_zone0/temp`

## Performance Considerations

### Read Timing

| Sensor | Conversion Time | I2C Transaction |
|--------|----------------|-----------------|
| SHT3x  | 50 ms | ~5 ms |
| BME280 | 100 ms | ~10 ms |
| CPU    | Instant | N/A (filesystem) |

**Total time per sensor:** ~60-110 ms including mux switching

### Bus Frequency

- Standard: 100 kHz
- Fast mode: 400 kHz (if supported by all devices)
- Configured in `/boot/config.txt`:
  ```
  dtparam=i2c_arm=on,i2c_arm_baudrate=100000
  ```

### Polling Rate Recommendations

- **5 seconds:** Good balance for temperature monitoring
- **1 second:** Excessive, temp changes slowly
- **10+ seconds:** May miss thermal events

## Dependencies

```bash
# Python package
pip install smbus2

# Or fallback
pip install smbus
```

## See Also

- [Hardware Node Documentation](../hardware_node/README.md)
- [SHT3x Datasheet](https://www.sensirion.com/en/environmental-sensors/humidity-sensors/digital-humidity-sensors-for-various-applications/)
- [BME280 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)
- [PCA9548A Datasheet](https://www.ti.com/product/PCA9548A)
