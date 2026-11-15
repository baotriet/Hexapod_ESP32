#ifndef MOTION_TIMING_H_
#define MOTION_TIMING_H_

#include <stdint.h>

class WorkTiming
{
public:
  void record(uint32_t elapsed)
  {
    total += elapsed;
    ++count;
    if (elapsed > maximum)
    {
      maximum = elapsed;
    }
  }

  uint32_t average() const
  {
    return count ? uint32_t(total / count) : 0;
  }

  uint32_t count = 0, maximum = 0;

private:
  uint64_t total = 0;
};

class PwmTiming
{
public:
  void beginPass()
  {
    passTime = 0;
    inPass = true;
  }

  void record(uint32_t elapsed, bool failed)
  {
    totalTime += elapsed;
    ++writes;
    if (failed)
    {
      ++failures;
    }
    if (elapsed > maxWrite)
    {
      maxWrite = elapsed;
    }
    if (inPass)
    {
      passTime += elapsed;
    }
  }

  void endPass()
  {
    totalPassTime += passTime;
    ++passes;
    if (passTime > maxPass)
    {
      maxPass = passTime;
    }
    inPass = false;
  }

  uint32_t averageWrite() const
  {
    return writes ? uint32_t(totalTime / writes) : 0;
  }

  uint32_t averagePass() const
  {
    return passes ? uint32_t(totalPassTime / passes) : 0;
  }

  uint32_t writes = 0, failures = 0, maxWrite = 0, passes = 0, maxPass = 0;

private:
  uint64_t totalTime = 0, totalPassTime = 0;
  uint32_t passTime = 0;
  bool inPass = false;
};

class MotionTiming
{
public:
  void begin(uint32_t now)
  {
    if (sampled)
    {
      uint32_t gap = now - previous;
      totalGap += gap;
      ++count;
      if (gap > maxGap)
      {
        maxGap = gap;
      }
      if (gap > 25000)
      {
        ++late;
      }
    }
    sampled = true;
    previous = now;
  }

  void end(uint32_t now)
  {
    uint32_t work = now - previous;
    if (work > maxWork)
    {
      maxWork = work;
    }
  }

  uint32_t average() const
  {
    return count ? uint32_t(totalGap / count) : 0;
  }
  uint32_t count = 0, maxGap = 0, maxWork = 0, late = 0;

private:
  uint64_t totalGap = 0;
  uint32_t previous = 0;
  bool sampled = false;
};

#endif
