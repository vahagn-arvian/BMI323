# BMI323 Breakout Board

A small collection of example code for the BMI323 6-axis IMU (Inertial Measurement Unit) breakout board. This repository provides easy-to-use Arduino and ESP32 examples for interfacing with the BMI323 sensor via I2C and SPI communication protocols.

## 🚀 Features

- **6-Axis Motion Sensing**: 3-axis accelerometer + 3-axis gyroscope
- **High Performance**: Up to 100Hz data rate with configurable ranges
- **Multiple Interfaces**: Support for both I2C and SPI communication
- **Temperature Sensing**: Built-in temperature sensor
- **Feature Engine**: Advanced motion detection and gesture recognition capabilities
- **Low Power**: Optimized for battery-powered applications

## 📋 Specifications

| Parameter | Value |
|-----------|-------|
| Accelerometer Range | ±2g, ±4g, ±8g, ±16g |
| Gyroscope Range | ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s |
| Output Data Rate | Up to 100Hz |
| Operating Voltage | 1.7V - 3.6V |
| I2C Address | 0x68 (SDO = GND) / 0x69 (SDO = VDD) |
| SPI Frequency | Up to 8MHz |
| Temperature Range | -40°C to +85°C |

## 📁 Project Structure

```
BMI323/
├── Arduino/
│   ├── bmi323_i2c_example.ino    # I2C communication example
│   └── bmi323_spi_example.ino    # SPI communication example
└── README.md
```

## 🔧 Hardware Setup

### Pin Connections

#### I2C Connection (Arduino Nano)
* Arvian BMI323 Breakout Board Specific Instructions:
    * <b>Do Not</b> pull SDO to VDD for alternate address (0x69), since this pin is the input voltage and is not regulated to the correct vddio voltage expected by the sensor.

| Arduino | BMI323 | Notes |
|-------------|------------|-------|
| A4 (SDA) | SDA | 4.7kΩ pull-up resistor required |
| A5 (SCL) | SCL | 4.7kΩ pull-up resistor required |
| GND | SDO | Set I2C address to 0x68 |
| 3.3V | VDD | Power supply |
| GND | GND | Ground |

#### SPI Connection (Arduino Nano)
| Arduino | BMI323 | Notes |
|-------------|------------|-------|
| D10 | CS | Chip select |
| D11 (MOSI) | SDI | Master out, slave in |
| D12 (MISO) | SDO | Master in, slave out |
| D13 (SCK) | SCK | Serial clock |
| 3.3V | VDD | Power supply |
| GND | GND | Ground |

## 📖 Usage Examples

### I2C Example
The I2C example demonstrates how to:
- Initialize the BMI323 sensor
- Configure accelerometer and gyroscope settings
- Read sensor data at 20Hz
- Output data in CSV format

```cpp
// Output format: ax,ay,az,gx,gy,gz,temp
// Units: g,g,g,deg/s,deg/s,deg/s,C
```

### SPI Example
The SPI example provides:
- High-speed SPI communication (8MHz)
- Same functionality as I2C example
- Optimized for applications requiring faster data transfer

## 🛠️ Installation

1. **Clone this repository**:
   ```bash
   git clone https://github.com/vahagn-arvian/BMI323.git
   cd BMI323
   ```

2. **Open Arduino IDE** and navigate to the example you want to use:
   - For I2C: `Arduino/bmi323_i2c_example.ino`
   - For SPI: `Arduino/bmi323_spi_example.ino`

3. **Connect your hardware** according to the pinout tables above

4. **Upload the code** to your Arduino board

5. **Open Serial Monitor** at 115200 baud to view the sensor data

## 📊 Data Output

The examples output sensor data in CSV format:
```
ax,ay,az,gx,gy,gz,temp
0.123,-0.045,1.002,0.5,-1.2,0.8,25.3
```

Where:
- `ax, ay, az`: Accelerometer data in g (gravitational acceleration)
- `gx, gy, gz`: Gyroscope data in degrees per second
- `temp`: Temperature in Celsius

## 🔍 Troubleshooting

### Common Issues

1. **"BMI323 failed to initialize"**
   - Check wiring connections
   - Verify power supply (3.3V - 5.5V)
   - Ensure pull-up resistors are connected (Only for I2C)
   - Check SDO pin connection (GND for I2C address 0x68)

2. **No data output**
   - Verify Serial Monitor is set to 115200 baud
   - Check that the correct example is uploaded
   - Ensure sensor is properly powered

3. **Incorrect readings**
   - Verify sensor orientation
   - Check for electromagnetic interference
   - Ensure stable power supply

### Debug Information

The examples include comprehensive debug output during initialization:
- Chip ID verification
- Feature engine status
- Configuration confirmation

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🔗 Resources

- [Arvian BMI323 Breakout Board](https://www.amazon.com/dp/B0FG2ZFHNM)
- [BMI323 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi323/)
- [Bosch Sensortec BMI323 Product Page](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi323/)
- [Arduino Wire Library Documentation](https://www.arduino.cc/reference/en/language/functions/communication/wire/)
- [Arduino SPI Library Documentation](https://www.arduino.cc/reference/en/language/functions/communication/spi/)

## 📞 Support

If you encounter any issues or have questions:
1. Check the troubleshooting section above
2. Search existing issues in this repository
3. Create a new issue with detailed information about your problem

---

**Note**: This repository is maintained for educational and development purposes, and may include directions required specifically for the BMI323 breakout board sold by Arvian. Always refer to the official BMI323 datasheet for production applications. 