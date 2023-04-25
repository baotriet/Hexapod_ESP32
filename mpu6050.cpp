#include "mpu6050.h"

MPU6050 mpu(Wire);

void mpu6050_update() {
  mpu.update();
}

float mpu6050_getAngle(char Axis) {
  float getAngle;
  switch (Axis) {
    case 'X':
      getAngle = mpu.getAngleX();
      break;
      
    case 'Y':
      getAngle = mpu.getAngleY();
      break;
    case 'Z':
      getAngle = mpu.getAngleZ();
      break;
  }
  return getAngle;
}

void mpu6050_init(bool isInit) {
  Wire.begin();

  byte status = mpu.begin();
#ifdef DEBUG_MPU
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
#endif                              //DEBUG_MPU
  while (status != 0 && isInit) {}  // stop everything if could not connect to MPU6050
#ifdef DEBUG_MPU
  Serial.println(F("Calculating offsets, do not move MPU6050"));
#endif  // DEBUG_MPU
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();  // gyro and accelero
#ifdef DEBUG_MPU
  Serial.println(F("Done!\n"));
#endif  // DEBUG_MPU
}