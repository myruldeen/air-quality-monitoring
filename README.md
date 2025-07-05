# ESP32 Environmental Sensor Station

A comprehensive air quality and environmental monitoring system using ESP32 with multiple sensors, OLED display, and MQTT connectivity.

## Features

- **Multi-sensor monitoring**: PMS5003 (particulate matter), BME680 (temperature, humidity, pressure, gas), SCD41 (CO2)
- **Real-time display**: 128x64 OLED with rotating pages showing different sensor data
- **MQTT connectivity**: Publishes sensor data to MQTT broker in JSON format
- **WiFi connectivity**: Automatic reconnection with status indication
- **FreeRTOS tasks**: Efficient multi-tasking for concurrent sensor reading
- **Air quality assessment**: Visual status indicators (Good, Moderate, Poor, Unhealthy)

## Hardware Requirements

### Components
- ESP32 development board
- PMS5003 particulate matter sensor
- BME680 environmental sensor (temperature, humidity, pressure, gas)
- SCD41 CO2 sensor
- 128x64 OLED display (SSD1306)
- LED for status indication
- Breadboard and jumper wires

### Pin Connections

| Component | ESP32 Pin | Description |
|-----------|-----------|-------------|
| PMS5003 RX | GPIO 16 | Serial communication |
| PMS5003 TX | GPIO 17 | Serial communication |
| I2C SDA | GPIO 21 | I2C data line |
| I2C SCL | GPIO 22 | I2C clock line |
| Status LED | GPIO 2 | Connection status indicator |
| OLED Display | I2C (0x3C) | Via I2C bus |
| BME680 | I2C | Via I2C bus |
| SCD41 | I2C | Via I2C bus |

## Software Dependencies

### Required Libraries
```cpp
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
```

### Installation
1. Install the ESP32 board package in Arduino IDE
2. Install required libraries through Library Manager:
   - WiFi (built-in)
   - PubSubClient
   - ArduinoJson
   - Adafruit BME680 Library
   - Adafruit SSD1306
   - Adafruit GFX Library
   - Sensirion I2C SCD4x

## Configuration

### WiFi Settings
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

### MQTT Settings
```cpp
const char* mqtt_server = "YOUR_MQTT_BROKER";
const int mqtt_port = 1883;
const char* mqtt_user = "YOUR_MQTT_USERNAME";
const char* mqtt_password = "YOUR_MQTT_PASSWORD";
const char* mqtt_topic = "sensors/environmental";
const char* device_id = "env_sensor_01";
```

## System Architecture

### FreeRTOS Tasks
The system uses multiple concurrent tasks:

1. **PMS Read Task** (Core 1, Priority 2)
   - Reads PMS5003 sensor data every 2 seconds
   - Handles serial communication with error recovery

2. **BME Read Task** (Core 1, Priority 2)
   - Reads BME680 environmental data every 2 seconds
   - Automatic sensor reinitialization on failures

3. **SCD Read Task** (Core 1, Priority 2)
   - Reads SCD41 CO2 data every 5 seconds
   - Manages periodic measurement cycle

4. **Display Task** (Core 0, Priority 1)
   - Updates OLED display with sensor data
   - Rotates between 3 different pages every 3 seconds

5. **MQTT Task** (Core 1, Priority 1)
   - Publishes combined sensor data to MQTT broker
   - Handles WiFi and MQTT reconnection

### Data Flow
```
Sensors → Task Queues → Display Task → OLED
                    ↓
                MQTT Task → MQTT Broker
```

## Display Pages

The OLED display cycles through three pages:

### Page 1: Air Quality Data
- PM1.0, PM2.5, PM10 concentrations
- Air quality status (Good/Moderate/Poor/Unhealthy)

### Page 2: Environmental Data
- Temperature (°C)
- Relative Humidity (%)
- Atmospheric Pressure (hPa)
- Gas Resistance (kΩ)

### Page 3: CO2 Data
- CO2 concentration (ppm)
- Temperature from SCD41
- Humidity from SCD41
- CO2 level status

## MQTT Data Format

Published JSON structure:
```json
{
  "device_id": "env_sensor_01",
  "timestamp": 1234567890,
  "pms5003": {
    "pm1_0": 10,
    "pm2_5": 15,
    "pm10_0": 20,
    "particles_0_3": 1000,
    "particles_0_5": 800,
    "particles_1_0": 600,
    "particles_2_5": 400,
    "particles_5_0": 200,
    "particles_10_0": 100
  },
  "bme680": {
    "temperature": 25.5,
    "humidity": 60.2,
    "pressure": 1013.25,
    "gas": 15.8
  },
  "scd41": {
    "co2": 450,
    "temperature": 25.3,
    "humidity": 59.8
  }
}
```

## Air Quality Thresholds

### PM2.5 Levels (μg/m³)
- **Good**: ≤ 12
- **Moderate**: 13-35
- **Poor**: 36-55
- **Unhealthy**: > 55

### CO2 Levels (ppm)
- **Good**: ≤ 800
- **Moderate**: 801-1000
- **Poor**: > 1000

## Error Handling

- **WiFi Connection**: Automatic reconnection with ESP32 restart on failure
- **MQTT Connection**: Retry mechanism with exponential backoff
- **Sensor Failures**: Automatic reinitialization and error recovery
- **Queue Management**: Automatic queue reset on overflow
- **Serial Communication**: Mutex-protected access with timeout handling

## Status Indicators

- **LED**: Blinks on successful MQTT publish
- **Serial Monitor**: Detailed logging of all sensor readings and system status
- **OLED Display**: Real-time sensor data with status indicators

## Troubleshooting

### Common Issues

1. **WiFi Connection Failed**
   - Check SSID and password
   - Verify network connectivity
   - ESP32 will restart automatically

2. **MQTT Connection Issues**
   - Verify broker address and port
   - Check username/password if authentication is required
   - Ensure topic permissions

3. **Sensor Not Responding**
   - Check wiring connections
   - Verify I2C address (use I2C scanner)
   - Check power supply (3.3V/5V requirements)

4. **Display Issues**
   - Verify I2C connections
   - Check OLED address (default 0x3C)
   - Ensure adequate power supply

### Serial Monitor Output
Enable serial monitoring at 115200 baud for detailed system diagnostics including:
- Sensor readings
- Connection status
- Error messages
- MQTT publish confirmations

## Power Consumption

- **Active Mode**: ~200-300mA (all sensors active, WiFi connected)
- **Recommended Power Supply**: 5V, 1A minimum

## Future Enhancements

- Deep sleep mode for battery operation
- Web server for local data access
- Data logging to SD card
- OTA (Over-The-Air) firmware updates
- Additional sensor support
- Calibration routines

## License

This project is open source. Please check individual library licenses for compliance.

## Contributing

1. Fork the repository
2. Create feature branch
3. Commit changes
4. Push to branch
5. Create Pull Request

## Support

For issues and questions:
- Check the troubleshooting section
- Review serial monitor output
- Verify hardware connections
- Check library versions