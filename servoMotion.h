#ifndef SERVO_MOTION_H_
#define SERVO_MOTION_H_

#include <stdint.h>

class ServoFrameClock
{
public:
  bool ready(uint32_t now, uint32_t interval)
  {
    if (sampled && uint32_t(now - last) < interval)
    {
      return false;
    }
    sampled = true;
    last = now;
    return true;
  }

private:
  uint32_t last = 0;
  bool sampled = false;
};

class ServoMotion
{
public:
  void begin(const short current[3], const short target[3], uint32_t now, uint32_t duration)
  {
    const float stopped[3] = {};
    begin(current, target, now, duration, stopped, stopped);
  }

  void begin(const short current[3], const short target[3], uint32_t now, uint32_t duration,
             const float entry[3], const float exit[3])
  {
    started = now;
    interval = duration ? duration : 1;
    active = true;
    for (unsigned joint = 0; joint < 3; ++joint)
    {
      origin[joint] = current[joint];
      goal[joint] = target[joint];
      float delta = goal[joint] - origin[joint];
      // Tangents in [0, delta] give a monotone cubic with peak speed <= 1.5 * |delta| / duration.
      entryTangent[joint] = boundedTangent(entry[joint] * interval, delta);
      exitTangent[joint] = boundedTangent(exit[joint] * interval, delta);
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
      delta += entryTangent[joint] * phase * (1.0f - phase) * (1.0f - phase) -
               exitTangent[joint] * phase * phase * (1.0f - phase);
      output[joint] = origin[joint] + short(delta >= 0 ? delta + 0.5f : delta - 0.5f);
    }
    return true;
  }

  void cancel()
  {
    active = false;
  }

  bool finished(uint32_t now) const
  {
    return active && uint32_t(now - started) >= interval;
  }

  float velocity(unsigned joint, uint32_t now) const
  {
    if (!active || joint >= 3 || uint32_t(now - started) > interval) return 0;
    float p = float(uint32_t(now - started)) / interval;
    return ((goal[joint] - origin[joint]) * 6 * p * (1 - p) +
            entryTangent[joint] * (1 - 4 * p + 3 * p * p) +
            exitTangent[joint] * (3 * p * p - 2 * p)) / interval;
  }

  void stopAtTarget(const short current[3], const short target[3], uint32_t now,
                    uint32_t minimumDuration)
  {
    uint32_t elapsed = now - started;
    uint32_t remaining = active && elapsed < interval ? interval - elapsed : 1;
    float entry[] = {velocity(0, now), velocity(1, now), velocity(2, now)};
    const float stopped[3] = {};
    begin(current, target, now, remaining > minimumDuration ? remaining : minimumDuration,
          entry, stopped);
  }

  static float boundaryVelocity(short previous, short waypoint, short next,
                                uint32_t duration, uint32_t nextDuration)
  {
    float incoming = float(waypoint - previous) / (duration ? duration : 1);
    float outgoing = float(next - waypoint) / (nextDuration ? nextDuration : 1);
    if (incoming * outgoing <= 0.0f) return 0.0f;
    return incoming > 0 ? (incoming < outgoing ? incoming : outgoing) :
                          (incoming > outgoing ? incoming : outgoing);
  }

private:
  static float boundedTangent(float tangent, float delta)
  {
    if (delta * tangent <= 0.0f) return 0.0f;
    if (delta > 0) return tangent < delta ? tangent : delta;
    return tangent > delta ? tangent : delta;
  }

  short origin[3] = {};
  short goal[3] = {};
  float entryTangent[3] = {};
  float exitTangent[3] = {};
  uint32_t started = 0;
  uint32_t interval = 1;
  bool active = false;
};

#endif // SERVO_MOTION_H_
