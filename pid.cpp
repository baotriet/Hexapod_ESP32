#include <PID_v2.h>
#include "pid.h"
#include "mpu6050.h"
#include "setup.h"

struct pidParam
{
  float Kp;
  float Ki;
  float Kd;
};
pidParam pidCoorX = {5.0, 0.0, 0.0};
pidParam pidCoorY = {5.0, 0.0, 0.0};
pidParam pidCoorZ = {5.0, 0.0, 0.0};

PID_v2 pidX(pidCoorX.Kp, pidCoorX.Ki, pidCoorX.Kd, PID::Direct, PID::P_On::Error);
PID_v2 pidY(pidCoorY.Kp, pidCoorY.Ki, pidCoorY.Kd, PID::Direct, PID::P_On::Error);
PID_v2 pidZ(pidCoorZ.Kp, pidCoorZ.Ki, pidCoorZ.Kd, PID::Direct, PID::P_On::Error);

void pid_init()
{
  mpu6050_init(false);
  pidX.SetOutputLimits(-45, 45);
  pidY.SetOutputLimits(-45, 45);
  pidZ.SetOutputLimits(-45, 45);
  pidX.Start(mpu6050_getAngle('X'), 0, 0);
  pidY.Start(mpu6050_getAngle('Y'), 0, 0);
  pidZ.Start(mpu6050_getAngle('Z'), 0, 0);
}

const float *pid_update(float *output, int size)
{
  mpu6050_update();
  float input[3];

  input[0] = mpu6050_getAngle('X');
  input[1] = mpu6050_getAngle('Y');
  input[2] = mpu6050_getAngle('Z');
  output[0] = pidX.Run(input[0]);
  output[1] = pidZ.Run(input[1]);
  output[2] = pidY.Run(input[2]);
#ifdef DEBUG_PID
  Serial.print(F("angle"));
  Serial.print(F("\t"));
  Serial.print(input[0]);
  Serial.print(F("\t"));
  Serial.print(input[1]);
  Serial.print(F("\t"));
  Serial.print(input[2]);
  Serial.print(F("\t"));
  Serial.print(output[0]);
  Serial.print(F("\t"));
  Serial.print(output[1]);
  Serial.print(F("\t"));
  Serial.println(output[2]);
#endif // DEBUG_PID

  return output;
}