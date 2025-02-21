#include <Wire.h>
#include "src/MPU6050-Driver/MPU6050.h"
#include "src/MPU6050-Driver/REG_OPTIONS.h"
#include "src/MPU6050-Driver/TEST_FUNCTIONS.h"

int arduino_reg_read(uint16_t regAddress, uint8_t* data)
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

int arduino_reg_write(uint16_t regAddress, uint8_t data)
{
  Wire.beginTransmission(MPU_6050_ADDR);
  Wire.write(regAddress);
  Wire.write(data);
  uint8_t status = Wire.endTransmission();
  return status == 0;
}

void printSetupRegResults(SETUP_REGISTERS results)
{
  //variables representing if register options match expected values
  char fsync_match = results.fsync == EXT_SYNC_OFF ? 't' : 'f';
  char dlpf_match = results.dlpf == DLPF_CFG_6 ? 't' : 'f';
  char gyro_sel_match = results.gyro_sel == GYRO_FS_SEL_250_DPS ? 't' : 'f';
  char fs_match = results.fs == ACCEL_FS_2G ? 't' : 'f';
  char rate_div_match = results.rate_div == SAMPLE_RATE_100Hz ? 't' : 'f';

  //print results
  Serial.println("Config Reg:");
  Serial.print("  FSYNC: ");
  Serial.print(results.fsync);
  Serial.print(", ");
  Serial.println(fsync_match);
  Serial.print("  DLPF: ");
  Serial.print(results.dlpf);
  Serial.print(", ");
  Serial.println(dlpf_match);

  Serial.println("Gyro Full Scale:");
  Serial.print("  FS: ");
  Serial.print(results.gyro_sel);
  Serial.print(", ");
  Serial.println(gyro_sel_match);

  Serial.println("Accel Config Reg:");
  Serial.print("  FS: ");
  Serial.print(results.fs);
  Serial.print(", ");
  Serial.println(fs_match);

  Serial.println("Sample Rate Div:");
  Serial.print("  Divider: ");
  Serial.print(results.rate_div);
  Serial.print(", ");
  Serial.println(rate_div_match);


}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  MPU6050_REG_READ_TYPE* readReg = arduino_reg_read;
  MPU6050_REG_WRITE_TYPE* writeReg = arduino_reg_write;
  DELAY_MS_TYPE* delayMs = delay;

  init_mpu6050(writeReg, delayMs);

  //setup the low pass filter. 
  writeReg(REG_CONFIG, DLPF_CFG_6 | EXT_SYNC_OFF);

  //select Gyroscope's full scale range
  writeReg(REG_GYRO_CONFIG, GYRO_FS_SEL_250_DPS);   
  
  //select Accelerometer's full scale range
  writeReg(REG_ACCEL_CONFIG, ACCEL_FS_2G);

  //setup the sample rate divider
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_100Hz);

  SETUP_REGISTERS results = read_setup_registers(readReg);
  printSetupRegResults(results);


}

void loop() {
  // put your main code here, to run repeatedly:

}
