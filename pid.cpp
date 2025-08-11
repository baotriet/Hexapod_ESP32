#include <PID_v2.h>
#include "pid.h"
#include "driver/mpu6050.h"
#include "setup.h"

struct pidParam { float Kp, Ki, Kd; };
static const pidParam pidCoorX = {5.0, 0.0, 0.0};
static const pidParam pidCoorY = {5.0, 0.0, 0.0};
static const pidParam pidCoorZ = {5.0, 0.0, 0.0};

static PID_v2 pidX(pidCoorX.Kp, pidCoorX.Ki, pidCoorX.Kd, PID::Direct, PID::P_On::Error);
static PID_v2 pidY(pidCoorY.Kp, pidCoorY.Ki, pidCoorY.Kd, PID::Direct, PID::P_On::Error);
static PID_v2 pidZ(pidCoorZ.Kp, pidCoorZ.Ki, pidCoorZ.Kd, PID::Direct, PID::P_On::Error);

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
  float x = mpu6050_getAngle('X');
  float y = mpu6050_getAngle('Y');
  float z = mpu6050_getAngle('Z');
  output[0] = pidX.Run(x);
  output[1] = pidZ.Run(y);
  output[2] = pidY.Run(z);
#ifdef DEBUG_PID
  Serial.print(F("angle\t"));
  Serial.print(x); Serial.print(F("\t"));
  Serial.print(y); Serial.print(F("\t"));
  Serial.print(z); Serial.print(F("\t"));
  Serial.print(output[0]); Serial.print(F("\t"));
  Serial.print(output[1]); Serial.print(F("\t"));
  Serial.println(output[2]);
#endif
  return output;
}