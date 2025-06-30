/*
 * bmi323_spi_example.ino
 * 
 * Simple Arduino example for interfacing with the BMI323 6-axis IMU over SPI.
 * Reads accelerometer, gyroscope, and temperature data at 20Hz.
 * Outputs data in CSV format to the Serial monitor.
 * 
 * Arduino (Nano) pinout:
 *   D10 -> CS
 *   D11 -> MOSI/SDI/SDX
 *   D12 -> MISO/SDO
 *   D13 -> SCK/SCX
 *
 * Output Format: ax,ay,az,gx,gy,gz,temp
 * Units: g,g,g,deg/s,deg/s,deg/s,C
 *
 * For more details, see the BMI323 datasheet.
*/

#include <SPI.h>

// Pin and register definitions
#define BMI323_CS 10
#define CHIP_ID_REG 0x00
#define ERR_REG 0x01
#define STATUS_REG 0x02
#define ACC_CONF_REG 0x20
#define GYR_CONF_REG 0x21
#define ACC_DATA_X_REG 0x03
#define ACC_DATA_Y_REG 0x04
#define ACC_DATA_Z_REG 0x05
#define GYR_DATA_X_REG 0x06
#define GYR_DATA_Y_REG 0x07
#define GYR_DATA_Z_REG 0x08
#define TEMP_DATA_REG 0x09
#define CMD_REG 0x7E
#define FEATURE_IO0_REG 0x10
#define FEATURE_IO1_REG 0x11
#define FEATURE_IO2_REG 0x12
#define FEATURE_IO_STATUS_REG 0x14
#define FEATURE_CTRL_REG 0x40

// Configuration constants
#define SPI_FREQ 8000000  // 8MHz SPI clock
#define OUTPUT_RATE_HZ 20  // 20Hz data output
#define ACC_RANGE_LSB_PER_G 4096.0  // +-8g range
#define GYR_RANGE_LSB_PER_DPS 16.384  // +-2000 deg/s range
#define SOFT_RESET_CMD 0xDEAF  // Soft reset command
#define ACC_CONF_NORMAL_100HZ_8G 0x4028  // Accel: 100Hz, +-8g
#define GYR_CONF_NORMAL_100HZ_2000DPS 0x4048  // Gyro: 100Hz, +-2000 deg/s

// SPI settings
SPISettings bmi323Settings(SPI_FREQ, MSBFIRST, SPI_MODE0);

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000 / OUTPUT_RATE_HZ; // 20Hz

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial

  // Set up SPI and CS pin
  pinMode(BMI323_CS, OUTPUT);
  digitalWrite(BMI323_CS, HIGH);
  SPI.begin();

  delay(100);  // Wait for sensor power-up

  if (initBMI323()) {
    Serial.println("BMI323 ready");
    Serial.println("Format: ax,ay,az,gx,gy,gz,temp");
    Serial.println("Units: g,g,g,deg/s,deg/s,deg/s,C");
  } else {
    Serial.println("BMI323 failed to initialize! Check wiring.");
    while (1);  // Stop if setup fails
  }
}

bool initBMI323() {
  // Switch to SPI mode
  readRegister16(CHIP_ID_REG);
  delayMicroseconds(200);

  // Check chip ID
  uint16_t chipID = readRegister16(CHIP_ID_REG);
  if ((chipID & 0xFF) != 0x43 && (chipID & 0xFF) != 0x41) {
    Serial.print("Invalid chip ID: 0x");
    Serial.println(chipID, HEX);
    return false;
  }

  // Reset sensor
  writeRegister16(CMD_REG, SOFT_RESET_CMD);
  delay(5);

  // Set up feature engine
  if (!initializeFeatureEngine()) {
    Serial.println("Feature engine setup failed");
    return false;
  }

  // Configure accelerometer and gyroscope
  writeRegister16(ACC_CONF_REG, ACC_CONF_NORMAL_100HZ_8G);
  writeRegister16(GYR_CONF_REG, GYR_CONF_NORMAL_100HZ_2000DPS);
  delay(50);

  return true;
}

bool initializeFeatureEngine() {
  // Clear feature config
  writeRegister16(FEATURE_IO0_REG, 0x0000);
  delay(1);

  // Set startup config
  writeRegister16(FEATURE_IO2_REG, 0x012C);
  delay(1);

  // Trigger startup
  writeRegister16(FEATURE_IO_STATUS_REG, 0x0001);
  delay(1);

  // Enable feature engine
  writeRegister16(FEATURE_CTRL_REG, 0x0001);
  delay(10);

  // Check feature engine status
  int timeout = 0;
  uint16_t featureIO1Status;
  do {
    delay(10);
    featureIO1Status = readRegister16(FEATURE_IO1_REG);
    uint8_t errorStatus = featureIO1Status & 0x0F;
    if (errorStatus == 0x01) return true;
    if (errorStatus == 0x03) {
      Serial.println("Feature engine error");
      return false;
    }
    timeout++;
  } while ((featureIO1Status & 0x0F) == 0x00 && timeout < 50);

  if (timeout >= 50) {
    Serial.println("Feature engine timeout");
    return false;
  }
  return true;
}

uint16_t readRegister16(uint8_t reg) {
  SPI.beginTransaction(bmi323Settings);
  digitalWrite(BMI323_CS, LOW);
  delayMicroseconds(1);

  SPI.transfer(reg | 0x80);  // Read command
  SPI.transfer(0x00);        // Dummy byte
  uint8_t lsb = SPI.transfer(0x00);
  uint8_t msb = SPI.transfer(0x00);

  digitalWrite(BMI323_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(2);

  return (msb << 8) | lsb;
}

void writeRegister16(uint8_t reg, uint16_t data) {
  SPI.beginTransaction(bmi323Settings);
  digitalWrite(BMI323_CS, LOW);
  delayMicroseconds(1);

  SPI.transfer(reg & 0x7F);        // Write command
  SPI.transfer(data & 0xFF);       // LSB
  SPI.transfer((data >> 8) & 0xFF); // MSB

  digitalWrite(BMI323_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(2);
}

float convertAccelData(uint16_t rawData) {
  int16_t signedData = (int16_t)rawData;
  if (signedData == -32768) return NAN;
  return signedData / ACC_RANGE_LSB_PER_G;
}

float convertGyroData(uint16_t rawData) {
  int16_t signedData = (int16_t)rawData;
  if (signedData == -32768) return NAN;
  return signedData / GYR_RANGE_LSB_PER_DPS;
}

float convertTempData(uint16_t rawData) {
  int16_t signedData = (int16_t)rawData;
  if (signedData == -32768) return NAN;
  return (signedData / 512.0) + 23.0;
}

void loop() {
  // Limit output to 20Hz
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime < printInterval) return;
  lastPrintTime = currentTime;

  // Read sensor data
  uint16_t accX = readRegister16(ACC_DATA_X_REG);
  uint16_t accY = readRegister16(ACC_DATA_Y_REG);
  uint16_t accZ = readRegister16(ACC_DATA_Z_REG);
  uint16_t gyrX = readRegister16(GYR_DATA_X_REG);
  uint16_t gyrY = readRegister16(GYR_DATA_Y_REG);
  uint16_t gyrZ = readRegister16(GYR_DATA_Z_REG);
  uint16_t tempRaw = readRegister16(TEMP_DATA_REG);

  // Convert to physical units
  float ax = convertAccelData(accX);
  float ay = convertAccelData(accY);
  float az = convertAccelData(accZ);
  float gx = convertGyroData(gyrX);
  float gy = convertGyroData(gyrY);
  float gz = convertGyroData(gyrZ);
  float temp = convertTempData(tempRaw);

  // Print valid data in CSV format
  if (!isnan(ax) && !isnan(ay) && !isnan(az) &&
      !isnan(gx) && !isnan(gy) && !isnan(gz)) {
    Serial.print(ax, 3); Serial.print(",");
    Serial.print(ay, 3); Serial.print(",");
    Serial.print(az, 3); Serial.print(",");
    Serial.print(gx, 2); Serial.print(",");
    Serial.print(gy, 2); Serial.print(",");
    Serial.print(gz, 2); Serial.print(",");
    Serial.println(isnan(temp) ? "NAN" : String(temp, 1));
  }
}