#include <Wire.h>
#include <string.h>
#include "src/MPU6050-Driver/MPU6050.h"
#include "src/MPU6050-Driver/REG_OPTIONS.h"
#include "src/MPU6050-Driver/TEST_FUNCTIONS.h"

//globally declare required function types
MPU6050_REG_WRITE_TYPE* writeReg;
MPU6050_REG_READ_TYPE* readReg;
MPU6050_BURST_READ_TYPE* burstRead;
DELAY_MS_TYPE* delayMs;
TIME_MS_TYPE* getTime;

uint16_t samplePeriodMs = 200; //200ms per sample or 5Hz

int arduino_reg_read_mpu6050(uint16_t regAddress, uint8_t* data)
{
  Wire.beginTransmission(MPU_6050_ADDR);
  Wire.write(regAddress);
  Wire.endTransmission(false); //don't send a stop condition
  Wire.requestFrom(MPU_6050_ADDR, SIZE_1_BYTE);
  while(!Wire.available()) {}
  *data = Wire.read();
  uint8_t status = Wire.endTransmission(true);  
  return status;
}

int arduino_burst_read_mpu6050(uint16_t regAddress, uint8_t* data, uint16_t bytes)
{
  Wire.beginTransmission(MPU_6050_ADDR);
  Wire.write(regAddress);
  Wire.endTransmission(false); //don't send a stop condition
  Wire.requestFrom(MPU_6050_ADDR, bytes);
  while(!Wire.available()) {}
  *data = Wire.read();
  uint8_t status = Wire.endTransmission(true);  
  return status;
}

int arduino_reg_write_mpu6050(uint16_t regAddress, uint8_t data)
{
  Wire.beginTransmission(MPU_6050_ADDR);
  Wire.write(regAddress);
  Wire.write(data);
  uint8_t status = Wire.endTransmission();
  return status == 0;
}


void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Serial.begin(115200);

  //assign required functions to arduino implementations
  writeReg = arduino_reg_write_mpu6050;
  readReg = arduino_reg_read_mpu6050;
  burstRead = arduino_burst_read_mpu6050;
  delayMs = delay;
  getTime = millis;

  init_mpu6050(writeReg, delayMs);

  //configure important accelerometer and gyro settings
  
  //setup the low pass filter. 
  writeReg(REG_CONFIG, DLPF_CFG_6 | EXT_SYNC_OFF);

  //select Gyroscope's full scale range
  writeReg(REG_GYRO_CONFIG, GYRO_FS_SEL_250_DPS);   
  
  //select Accelerometer's full scale range
  writeReg(REG_ACCEL_CONFIG, ACCEL_FS_SEL_2G);

  //setup the sample rate divider
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_5Hz);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int startRead = getTime();
  float accelX = read_accel_axis(REG_ACCEL_X_MEASURE_1, ACCEL_FS_2_DIV, readReg);
  
  float accelY = read_accel_axis(REG_ACCEL_Y_MEASURE_1, ACCEL_FS_2_DIV, readReg);
  
  float accelZ = read_accel_axis(REG_ACCEL_Z_MEASURE_1, ACCEL_FS_2_DIV, readReg);

  float gyroX = read_gyro_axis(REG_GYRO_X_MEASURE_1, GYRO_FS_250_DIV, readReg);

  float gyroY = read_gyro_axis(REG_GYRO_Y_MEASURE_1, GYRO_FS_250_DIV, readReg);

  float gyroZ = read_gyro_axis(REG_GYRO_Z_MEASURE_1, GYRO_FS_250_DIV, readReg);

  Serial.print("AccelX:");
  Serial.print(accelX);
  Serial.print(",");

  Serial.print("AccelY:");
  Serial.print(accelY);
  Serial.print(",");

  Serial.print("AccelZ:");
  Serial.print(accelZ);
  Serial.print(",");

  Serial.print("GyroX:");
  Serial.print(gyroX);
  Serial.print(",");

  Serial.print("GyroY:");
  Serial.print(gyroY);
  Serial.print(",");

  Serial.print("GyroZ:");
  Serial.println(gyroZ);

  int endRead = getTime();
  int totalTime = endRead - startRead; //ms
  delay(samplePeriodMs - totalTime);

}
