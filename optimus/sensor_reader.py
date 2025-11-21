#!/usr/bin/env python3
"""
Sensor Reader Module
Handles I2C communication with temperature sensors via PCA9548A multiplexer
"""

import time
import struct
import logging

try:
    import smbus2 as smbus
except ImportError:
    import smbus

logger = logging.getLogger(__name__)


class SensorReader:
    def __init__(self, i2c_bus=1, mux_address=0x70):
        self.i2c_bus = i2c_bus
        self.mux_address = mux_address
        self.bus = None
        
    def connect(self):
        """Initialize I2C bus connection"""
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            logger.info(f"Connected to I2C bus {self.i2c_bus}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to I2C bus: {e}")
            return False
    
    def disconnect(self):
        """Close I2C bus connection"""
        if self.bus:
            self.bus.close()
            self.bus = None
            logger.info("Disconnected from I2C bus")
    
    def select_mux_channel(self, channel):
        """Select channel on PCA9548A multiplexer"""
        try:
            self.bus.write_byte(self.mux_address, 1 << channel)
            time.sleep(0.02)
            return True
        except Exception as e:
            logger.error(f"Failed to select mux channel {channel}: {e}")
            return False
    
    def disable_mux_channels(self):
        """Disable all multiplexer channels"""
        try:
            self.bus.write_byte(self.mux_address, 0x00)
            time.sleep(0.02)
        except Exception as e:
            logger.warning(f"Failed to disable mux channels: {e}")
    
    def read_sht3x(self, address, channel):
        """Read temperature and humidity from SHT3x sensor"""
        try:
            if not self.select_mux_channel(channel):
                return None
            
            self.bus.write_i2c_block_data(address, 0x24, [0x00])
            time.sleep(0.05)
            data = self.bus.read_i2c_block_data(address, 0x00, 6)
            
            temp_raw = (data[0] << 8) | data[1]
            temp_c = -45 + (175 * temp_raw / 65535.0)
            
            hum_raw = (data[3] << 8) | data[4]
            humidity = 100 * hum_raw / 65535.0
            
            return {
                'temperature': round(temp_c, 2),
                'humidity': round(humidity, 2),
                'success': True
            }
        except Exception as e:
            logger.error(f"Failed to read SHT3x at 0x{address:02X}, channel {channel}: {e}")
            return {'success': False, 'error': str(e)}
    
    def read_bme280(self, address, channel):
        """Read temperature and pressure from BME280/BMP280"""
        try:
            if not self.select_mux_channel(channel):
                return None
            
            chip_id = self.bus.read_byte_data(address, 0xD0)
            is_bme280 = (chip_id == 0x60)
            is_bmp280 = (chip_id == 0x58)
            
            if not (is_bme280 or is_bmp280):
                return {'success': False, 'error': f'Unknown chip: 0x{chip_id:02X}'}
            
            # Reset and configure
            self.bus.write_byte_data(address, 0xE0, 0xB6)
            time.sleep(0.01)
            
            # Read calibration
            cal = self.bus.read_i2c_block_data(address, 0x88, 24)
            if is_bme280:
                cal += self.bus.read_i2c_block_data(address, 0xE1, 7)
            
            dig_T1 = cal[0] | (cal[1] << 8)
            dig_T2 = struct.unpack('<h', bytes([cal[2], cal[3]]))[0]
            dig_T3 = struct.unpack('<h', bytes([cal[4], cal[5]]))[0]
            
            dig_P1 = cal[6] | (cal[7] << 8)
            dig_P2 = struct.unpack('<h', bytes([cal[8], cal[9]]))[0]
            dig_P3 = struct.unpack('<h', bytes([cal[10], cal[11]]))[0]
            dig_P4 = struct.unpack('<h', bytes([cal[12], cal[13]]))[0]
            dig_P5 = struct.unpack('<h', bytes([cal[14], cal[15]]))[0]
            dig_P6 = struct.unpack('<h', bytes([cal[16], cal[17]]))[0]
            dig_P7 = struct.unpack('<h', bytes([cal[18], cal[19]]))[0]
            dig_P8 = struct.unpack('<h', bytes([cal[20], cal[21]]))[0]
            dig_P9 = struct.unpack('<h', bytes([cal[22], cal[23]]))[0]
            
            if is_bme280:
                self.bus.write_byte_data(address, 0xF2, 0x01)
            self.bus.write_byte_data(address, 0xF4, 0x27)
            self.bus.write_byte_data(address, 0xF5, 0xA0)
            
            time.sleep(0.1)
            
            data = self.bus.read_i2c_block_data(address, 0xF7, 8)
            adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
            adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            
            # Temperature
            var1 = ((adc_t / 16384.0) - (dig_T1 / 1024.0)) * dig_T2
            var2 = (((adc_t / 131072.0) - (dig_T1 / 8192.0)) ** 2) * dig_T3
            t_fine = int(var1 + var2)
            temp_c = (var1 + var2) / 5120.0
            
            # Pressure
            var1 = (t_fine / 2.0) - 64000.0
            var2 = var1 * var1 * dig_P6 / 32768.0
            var2 = var2 + var1 * dig_P5 * 2.0
            var2 = (var2 / 4.0) + (dig_P4 * 65536.0)
            var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
            var1 = (1.0 + var1 / 32768.0) * dig_P1
            
            if var1 == 0:
                pressure_hpa = 0
            else:
                p = 1048576.0 - adc_p
                p = ((p - var2 / 4096.0) * 6250.0) / var1
                var1 = dig_P9 * p * p / 2147483648.0
                var2 = p * dig_P8 / 32768.0
                pressure_hpa = (p + (var1 + var2 + dig_P7) / 16.0) / 100.0
            
            chip_name = "BME280" if is_bme280 else "BMP280"
            
            return {
                'temperature': round(temp_c, 2),
                'pressure': round(pressure_hpa, 2),
                'chip': chip_name,
                'success': True
            }
            
        except Exception as e:
            logger.error(f"Failed to read BME280 at 0x{address:02X}, channel {channel}: {e}")
            return {'success': False, 'error': str(e)}
    
    def read_sensor(self, sensor_config):
        """Read sensor based on configuration"""
        sensor_type = sensor_config['type']
        address = int(sensor_config['address'], 16)
        channel = sensor_config['channel']
        
        if sensor_type == 'sht3x':
            return self.read_sht3x(address, channel)
        elif sensor_type == 'bme280':
            return self.read_bme280(address, channel)
        else:
            logger.error(f"Unknown sensor type: {sensor_type}")
            return {'success': False, 'error': f'Unknown sensor type: {sensor_type}'}
