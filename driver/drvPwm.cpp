#include "driver/drvPwm.h"
#include <Adafruit_PWMServoDriver.h>
#include "HexCfg.h"
#include <Wire.h>
#include "serialOutput.h"

Adafruit_PWMServoDriver pwmRight = Adafruit_PWMServoDriver(R_DRIVER);
Adafruit_PWMServoDriver pwmLeft = Adafruit_PWMServoDriver(L_DRIVER);
static int lastPulse[2][16];
static bool pulseValid[2][16];
static int queuedPulse[2][16];
static bool queued[2][16];
static bool batching;

static void rememberWrite(byte driver, byte channel, int pulse, bool success)
{
  if (!success)
  {
    if (pulseValid[driver][channel] || lastPulse[driver][channel] != -1)
    {
      Serial.println(F("Servo PWM: I2C write failed; retrying next frame."));
    }
    pulseValid[driver][channel] = false;
    lastPulse[driver][channel] = -1;
    return;
  }
  lastPulse[driver][channel] = pulse;
  pulseValid[driver][channel] = true;
}
#if BUILDCFG_MOTION_TIMING_ENABLED
static PwmTiming pwmTiming;

PwmTiming &drvPwm_timing()
{
  return pwmTiming;
}
#endif

void drvPwm_goSleep()
{
  batching = false;
  memset(queued, 0, sizeof(queued));
  memset(pulseValid, 0, sizeof(pulseValid));
  pwmRight.sleep();
  pwmLeft.sleep();
}

static bool validCommand(uint16_t addr, byte channel, int pwmPulse)
{
  if ((addr != R_DRIVER && addr != L_DRIVER) || channel >= 16 || pwmPulse < 0 ||
      pwmPulse > 4096)
  {
    Serial.println(F("Servo PWM: invalid address, channel, or pulse."));
    return false;
  }
  return true;
}

static bool writeImmediate(byte driver, byte channel, int pwmPulse)
{
  if (pulseValid[driver][channel] && lastPulse[driver][channel] == pwmPulse)
  {
    return true;
  }
  Adafruit_PWMServoDriver &pwm = driver == 0 ? pwmRight : pwmLeft;
#if BUILDCFG_MOTION_TIMING_ENABLED
  uint32_t started = micros();
#endif
  uint8_t result = pwm.setPWM(channel, 0, pwmPulse);
#if BUILDCFG_MOTION_TIMING_ENABLED
  pwmTiming.record(uint32_t(micros() - started), result != 0);
#endif
  rememberWrite(driver, channel, pwmPulse, result == 0);
  return result == 0;
}

bool drvPwm_setImmediate(uint16_t addr, byte channel, int pwmPulse)
{
  if (!validCommand(addr, channel, pwmPulse))
  {
    return false;
  }
  if (batching)
  {
    Serial.println(F("Servo PWM: immediate write rejected during batch."));
    return false;
  }
  return writeImmediate(addr == R_DRIVER ? 0 : 1, channel, pwmPulse);
}

void drvPwm_set(uint16_t addr, byte channel, int pwmPulse)
{
  if (!validCommand(addr, channel, pwmPulse))
  {
    return;
  }
  byte driver = addr == R_DRIVER ? 0 : 1;
  if (batching)
  {
    queuedPulse[driver][channel] = pwmPulse;
    queued[driver][channel] = !pulseValid[driver][channel] ||
                              lastPulse[driver][channel] != pwmPulse;
    return;
  }
  writeImmediate(driver, channel, pwmPulse);
}

void drvPwm_beginBatch()
{
  if (batching)
  {
    Serial.println(F("Servo PWM: overlapping batch rejected."));
    return;
  }
  memset(queued, 0, sizeof(queued));
  batching = true;
}

void drvPwm_endBatch()
{
  if (!batching)
  {
    Serial.println(F("Servo PWM: no batch pending."));
    return;
  }
  batching = false;
  for (byte driver = 0; driver < 2; ++driver)
  {
    byte channel = 0;
    while (channel < 16)
    {
      if (!queued[driver][channel])
      {
        ++channel;
        continue;
      }
      byte first = channel;
      // Seven channels plus the register pointer fit even a 32-byte Wire buffer.
      uint8_t buffer[29];
      byte length = 1;
      buffer[0] = 0x06 + 4 * first; // LEDn_ON_L; setPWMFreq enables auto-increment.
      while (channel < 16 && queued[driver][channel] && channel - first < 7)
      {
        int pulse = queuedPulse[driver][channel++];
        buffer[length++] = 0;
        buffer[length++] = 0;
        buffer[length++] = uint8_t(pulse);
        buffer[length++] = uint8_t(pulse >> 8);
      }
#if BUILDCFG_MOTION_TIMING_ENABLED
      uint32_t started = micros();
#endif
      Wire.beginTransmission(driver == 0 ? R_DRIVER : L_DRIVER);
      bool complete = Wire.write(buffer, length) == length;
      uint8_t result = Wire.endTransmission();
      bool success = complete && result == 0;
#if BUILDCFG_MOTION_TIMING_ENABLED
      pwmTiming.record(uint32_t(micros() - started), !success);
#endif
      for (byte sent = first; sent < channel; ++sent)
      {
        rememberWrite(driver, sent, queuedPulse[driver][sent], success);
        queued[driver][sent] = false;
      }
    }
  }
}

void drvPwm_init()
{
  batching = false;
  memset(queued, 0, sizeof(queued));
  memset(pulseValid, 0, sizeof(pulseValid));
  memset(lastPulse, 0, sizeof(lastPulse));
#ifdef DEBUG_PWM
  Serial.println(F("16 test"));
#endif // DEBUG_PWM
  // /* Wake the device up */
  pwmRight.begin();
  pwmLeft.begin();
  pwmRight.setOscillatorFrequency(R_Frequency);
  pwmLeft.setOscillatorFrequency(L_Frequency);
  pwmRight.setPWMFreq(PWM_FREQ); // Analog servos run at ~50 Hz updates
  pwmLeft.setPWMFreq(PWM_FREQ);  // Analog servos run at ~50 Hz updates
}