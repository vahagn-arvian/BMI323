# BMI323 Arduino Example Code

This is a repository with example Arduino sketches for the BMI323 6-axis IMU (Inertial Measurement Unit) by Bosch. The code focuses on a basic setup and configuration to read and output raw sensor data without any fusion or filtering.

## Breakout Board

This repository is designed to work with the [Arvian BMI323 Breakout Board](https://www.amazon.com/dp/B0FG2ZFHNM). The board provides an easy-to-use interface for the BMI323 6-axis IMU, with pin configurations tailored for Arduino compatibility. If using another board, always refer to the documentation for specific details.

## Features

- 6-Axis Motion Sensing: 3-axis accelerometer and 3-axis gyroscope for precise motion tracking.
- High Performance: Configurable output data rates up to 6.4 kHz (accelerometer) and 12.8 kHz (gyroscope); examples use 100Hz by default.
- Multiple Interfaces: Supports I2C, SPI, and I3C communication (examples cover I2C and SPI).
- Temperature Sensing: Built-in 16-bit temperature sensor.
- Feature Engine: Includes motion detection, gesture recognition, and a plug-and-play step counter.
- Low Power: Designed for battery-powered applications with configurable power modes.

## Specifications

| Parameter | Value |
|-----------|-------|
| Accelerometer Range | ±2g, ±4g, ±8g, ±16g |
| Gyroscope Range | ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s |
| Output Data Rate | Up to 6.4 kHz (accelerometer), 12.8 kHz (gyroscope); example code uses 100Hz |
| Operating Voltage (BMI323 Chip) | 1.7V - 3.6V |
| Arvian Breakout Board Input Voltage (VDD) | 3.3V - 5.5V |
| I2C Address | 0x68 (SDO = GND); 0x69 not supported on Arvian breakout board (see note) |
| SPI Frequency | Up to 10 MHz (chip); example code uses 8 MHz for Arduino compatibility |
| Temperature Range | -40°C to +85°C |

**Note**: Do not wire SDO to VDD for the alternate address (0x69), since this input voltage is unregulated. Only wire to ground to establish the default address.

## Project Structure

```
BMI323/
├── Arduino/
│   ├── bmi323_i2c_example.ino    # I2C communication example
│   └── bmi323_spi_example.ino    # SPI communication example
└── README.md
```

## Hardware Setup

### Pin Connections

#### I2C Connection (Arduino Nano)
- Do not pull SDO to VDD for the alternate address 0x69 on the Arvian breakout board, as the input VDD voltage is unregulated and will damage the sensor.

| Arduino | BMI323 Breakout Board | Notes |
|-------------|------------|-------|
| A4 (SDA) | SDA | 4.7kΩ pull-up resistor required |
| A5 (SCL) | SCL | 4.7kΩ pull-up resistor required |
| GND | SDO | Sets I2C address to 0x68 |
| 3.3V - 5.5V | VDD | Power supply (do not exceed 3.6V for chip safety) |
| GND | GND | Ground |

#### SPI Connection (Arduino Nano)

| Arduino | BMI323 Breakout Board | Notes |
|-------------|------------|-------|
| D10 | CS | Chip select |
| D11 (MOSI) | SDI | Master out, slave in |
| D12 (MISO) | SDO | Master in, slave out |
| D13 (SCK) | SCK | Serial clock |
| 3.3V - 5.5V | VDD | Power supply |
| GND | GND | Ground |

## Usage Examples

### I2C Example
The I2C example shows how to:
- Initialize the BMI323 sensor.
- Configure accelerometer and gyroscope settings.
- Read sensor data at 20Hz (configurable up to 6.4 kHz for accelerometer, 12.8 kHz for gyroscope).
- Output data in CSV format.

```cpp
// Output format: ax,ay,az,gx,gy,gz,temp
// Units: g,g,g,deg/s,deg/s,deg/s,C
```

### SPI Example
The SPI example provides:
- High-speed SPI communication (8 MHz, chip supports up to 10 MHz).
- Same functionality as the I2C example.
- Optimized for applications needing faster data transfer.

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/vahagn-arvian/BMI323.git
   cd BMI323
   ```

2. Open Arduino IDE and load the example you want:
   - For I2C: `Arduino/bmi323_i2c_example.ino`
   - For SPI: `Arduino/bmi323_spi_example.ino`

3. Connect your hardware as shown in the pinout tables above.

4. Upload the code to your Arduino board.

5. Open the Serial Monitor at 115200 baud to view sensor data.

## Data Output

The examples output sensor data in CSV format:
```
ax,ay,az,gx,gy,gz,temp
0.123,-0.045,1.002,0.5,-1.2,0.8,25.3
```

Where:
- `ax, ay, az`: Accelerometer data in g (gravitational acceleration plus motion).
- `gx, gy, gz`: Gyroscope data in degrees per second (angular velocity).
- `temp`: Temperature in Celsius.

## Troubleshooting

### Common Issues

1. **"BMI323 failed to initialize"**
   - Check all wiring connections.
   - Verify power supply to breakout board.
   - Ensure 4.7kΩ pull-up resistors are connected (I2C only).
   - Confirm SDO is connected to GND for I2C address 0x68. (Floating address pin can cause address inconsistencies.)
   - Verify the I2C address in the code matches 0x68.

2. **No data output**
   - Ensure Serial Monitor is set to 115200 baud.
   - Check that the correct example sketch is uploaded.
   - Verify the sensor is powered correctly.

3. **Incorrect readings**
   - Confirm sensor orientation is correct.
   - Check for electromagnetic interference near the sensor.
   - Ensure a stable power supply (avoid voltage spikes).

### Debug Information

The examples include debug output during initialization, showing:
- Chip ID verification.
- Feature engine status.
- Configuration confirmation.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Resources

- [Arvian BMI323 Breakout Board](https://www.amazon.com/dp/B0FG2ZFHNM).
- [BMI323 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi323/)
- [Bosch Sensortec BMI323 Product Page](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi323/)
- [Bosch Sensortec BMI3XY Sensor API](https://github.com/boschsensortec/BMI3XY_SensorAPI)
- [Arduino Wire Library Documentation](https://www.arduino.cc/reference/en/language/functions/communication/wire/)
- [Arduino SPI Library Documentation](https://www.arduino.cc/reference/en/language/functions/communication/spi/)

## Support

If you run into issues or have questions:
1. Check the troubleshooting section above.
2. Search existing issues in this repository.
3. Create a new issue with details about your setup and problem.

---

**Note**: This repository is for educational and development purposes and includes instructions specific to the Arvian BMI323 Breakout Board. For production applications, always refer to the official BMI323 datasheet.
