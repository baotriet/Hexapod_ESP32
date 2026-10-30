#ifndef SERVO_MOTION_H_
#define SERVO_MOTION_H_

#include <stdint.h>

class ServoMotion
{
public:
  void begin(const short current[3], const short target[3], uint32_t now, uint32_t duration)
  {
    started = now;
    interval = duration ? duration : 1;
    active = true;
    for (unsigned joint = 0; joint < 3; ++joint)
    {
      origin[joint] = current[joint];
      goal[joint] = target[joint];
    }
  }

  bool sample(short output[3], const short target[3], uint32_t now)
  {
    if (!active)
    {
      return false;
    }
    for (unsigned joint = 0; joint < 3; ++joint)
    {
      if (target[joint] != goal[joint])
      {
        // A contact/recovery target supersedes the segment, never continue toward a stale pose.
        active = false;
        return false;
      }
    }
    uint32_t elapsed = now - started;
    float phase = elapsed >= interval ? 1.0f : float(elapsed) / interval;
    float blend = phase * phase * (3.0f - 2.0f * phase);
    for (unsigned joint = 0; joint < 3; ++joint)
    {
      float delta = (goal[joint] - origin[joint]) * blend;
      output[joint] = origin[joint] + short(delta >= 0 ? delta + 0.5f : delta - 0.5f);
    }
    return true;
  }

  void cancel()
  {
    active = false;
  }

private:
  short origin[3] = {};
  short goal[3] = {};
  uint32_t started = 0;
  uint32_t interval = 1;
  bool active = false;
};

#endif // SERVO_MOTION_H_
