#include <PID_v2.h>
#include "mpu6050.h"
#include "setup.h"

struct pidParam {
  double Kp;
  double Ki;
  double Kd;
};
pidParam pidCoorX = { 1, 0, 0 };
pidParam pidCoorY = { 1, 0, 0 };
pidParam pidCoorZ = { 1, 0, 0 };
PID_v2 pidX(pidCoorX.Kp, pidCoorX.Ki, pidCoorX.Kd, PID::Direct);
PID_v2 pidZ(pidCoorZ.Kp, pidCoorZ.Ki, pidCoorZ.Kd, PID::Direct);
PID_v2 pidY(pidCoorY.Kp, pidCoorY.Ki, pidCoorY.Kd, PID::Direct);

void pid_init() {
  mpu6050_init(false);
  pidX.SetOutputLimits(-40, 40);
  pidZ.SetOutputLimits(-40, 40);
  pidY.SetOutputLimits(-40, 40);
  pidX.Start(mpu.getAngleX(), 0, 0);
  pidZ.Start(mpu.getAngleY(), 0, 0);
  pidY.Start(mpu.getAngleZ(), 0, 0);
}

const double *pid_update(double *output, int size) {
  mpu.update();

  const double inputX = mpu.getAngleX();
  const double inputZ = mpu.getAngleY();
  const double inputY = mpu.getAngleZ();
  output[0] = pidX.Run(inputX) * 10;
  output[1] = pidZ.Run(inputZ) * 10;
  output[2] = pidY.Run(inputY) * 10;
#ifdef DEBUG_PID
  Serial.print(F("angle"));
  Serial.print(F("\t"));
  Serial.print(inputX);
  Serial.print(F("\t"));
  Serial.print(inputY);
  Serial.print(F("\t"));
  Serial.print(inputZ);
  Serial.print(F("\t"));
  Serial.print(output[0]);
  Serial.print(F("\t"));
  Serial.print(output[1]);
  Serial.print(F("\t"));
  Serial.println(output[2]);
#endif  // DEBUG_PID

  return output;
}