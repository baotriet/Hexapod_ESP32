#include "footLanding.h"
#include "footSensor.h"

enum LandingPhase { SUPPORT, SWING, DESCENDING, SEARCHING, TOUCHED };
struct LandingLeg
{
  LandingPhase phase;
  long offset;
  long current;
  long target;
  long planned;
  long start;
  bool released;
  bool supported;
  bool nominal;
  uint32_t began;
  uint32_t phaseStarted;
  uint32_t duration;
  uint32_t searched;
  uint32_t lastSearch;
};
static LandingLeg legs[NUM_OF_LEGS];
static bool faulted;
static bool engaged;
static bool timedOut;
static bool restoring;
static bool restored;
static uint32_t restoreStarted;
static uint32_t restoreFrame;
static const unsigned HISTORY_CAPACITY = 256;
struct FootFrame { short x, y, z, rotation; };
struct FootHistory
{
  FootFrame frames[HISTORY_CAPACITY];
  unsigned count;
  unsigned cursor;
  bool valid;
  bool needed;
};
static FootHistory histories[NUM_OF_LEGS];
static const char *const names[] = {"RF", "RM", "RR", "LF", "LM", "LR"};

static_assert(FOOT_WALK_EXTRA_DOWN_MM > 0 && FOOT_WALK_SEARCH_MM_S > 0 &&
                  FOOT_WALK_TIMEOUT_MS > 0, "Walking search limits must be positive");

void footLanding_init()
{
  memset(legs, 0, sizeof(legs));
  faulted = engaged = false;
  timedOut = restoring = restored = false;
  memset(histories, 0, sizeof(histories));
}

void footLanding_fail(const char *reason, byte leg)
{
  if (!faulted)
  {
    Serial.print(F("Foot landing HOLD: "));
    Serial.print(reason);
    Serial.print(F(" leg="));
    Serial.println(leg < NUM_OF_LEGS ? names[leg] : "all");
    Serial.println(F("Protection remains enabled. Restore support and center sticks before recovery."));
  }
  faulted = true;
}

void footLanding_plan(const long previousY[], const long plannedY[], uint32_t duration)
{
  bool first = !engaged;
  engaged = true;
  if (!footSensor_mappingValid())
  {
    footLanding_fail("invalid sensor mapping", NUM_OF_LEGS);
    return;
  }
  uint32_t now = millis();
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    LandingLeg &s = legs[leg];
    if (first && s.phase == SUPPORT)
      s.supported = footSensor_isTriggered(leg);
    if (plannedY[leg] < previousY[leg] && plannedY[leg] < 0)
    {
      if (s.phase == SUPPORT || s.phase == TOUCHED)
      {
        s.released = false;
        s.offset = 0;
        // Keep the last grounded frame as the beginning of this swing's return path.
        histories[leg].count = histories[leg].valid ? 1 : 0;
      }
      s.phase = SWING;
      s.supported = false;
    }
    else if (plannedY[leg] > previousY[leg] && previousY[leg] < 0)
    {
      s.nominal = plannedY[leg] >= 0;
      if (s.phase == TOUCHED)
      {
        s.offset = s.current - plannedY[leg];
        if (s.nominal)
          s.phase = SUPPORT;
        continue;
      }
      s.start = previousY[leg] + s.offset;
      s.current = s.start;
      s.target = plannedY[leg] + s.offset;
      s.planned = plannedY[leg];
      s.began = s.phaseStarted = now;
      s.duration = duration ? duration : 1;
      s.phase = DESCENDING;
    }
  }
}

void footLanding_update()
{
  if (!engaged || faulted || restoring)
    return;
  uint32_t now = millis();
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    LandingLeg &s = legs[leg];
    if (s.phase != SUPPORT && footSensor_isReleasedStable(leg))
      s.released = true;
    if (s.phase == SUPPORT || s.phase == TOUCHED)
    {
      if (s.supported && footSensor_isReleasedStable(leg))
      {
        footLanding_fail("support contact lost", leg);
        return;
      }
      continue;
    }
    if (s.phase != DESCENDING && s.phase != SEARCHING)
      continue;
    if (s.released && footSensor_isTriggered(leg))
    {
      s.offset = s.current - s.planned;
      s.phase = s.nominal ? SUPPORT : TOUCHED;
      s.supported = true;
      Serial.print(F("Touchdown "));
      Serial.print(names[leg]);
      Serial.print(F(" gait Y="));
      Serial.println(s.current);
      continue;
    }
    if (s.phase == SEARCHING &&
        uint32_t(now - s.searched) >= FOOT_WALK_TIMEOUT_MS)
    {
      timedOut = s.released;
      footLanding_fail(s.released ? "no ground within landing timeout" : "switch never released", leg);
      return;
    }
    // Any raw contact pauses descent immediately, including switch debounce.
    if (footSensor_isRawTriggered(leg) || !footSensor_isReleasedStable(leg))
    {
      if (s.phase == DESCENDING)
      {
        s.began = now;
        s.start = s.current;
        if (uint32_t(now - s.phaseStarted) >= s.duration + FOOT_WALK_TIMEOUT_MS)
        {
          footLanding_fail(s.released ? "contact did not stabilize" : "switch never released", leg);
          return;
        }
      }
      s.lastSearch = now;
      continue;
    }
    if (s.phase == DESCENDING)
    {
      uint32_t elapsed = now - s.began;
      long next = s.start + (s.target - s.start) *
                  long(elapsed < s.duration ? elapsed : s.duration) / s.duration;
      s.current = next;
      if (elapsed >= s.duration)
      {
        if (!s.nominal)
          s.phase = SWING;
        else
        {
          s.phase = SEARCHING;
          s.searched = s.lastSearch = now;
        }
      }
    }
    else if (uint32_t(now - s.lastSearch) >= (1000U + FOOT_WALK_SEARCH_MM_S - 1) / FOOT_WALK_SEARCH_MM_S)
    {
      s.lastSearch = now;
      if (s.current >= s.target + FOOT_WALK_EXTRA_DOWN_MM)
      {
        footLanding_fail("extra downward limit reached", leg);
        return;
      }
      ++s.current;
    }
  }
}

bool footLanding_busy()
{
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    if (legs[leg].phase == DESCENDING || legs[leg].phase == SEARCHING)
      return true;
  return false;
}

bool footLanding_fault() { return faulted; }
bool footLanding_engaged() { return engaged; }
bool footLanding_direct(byte leg)
{
  return legs[leg].phase == DESCENDING || legs[leg].phase == SEARCHING ||
         legs[leg].phase == TOUCHED || legs[leg].supported;
}

long footLanding_y(byte leg, long gaitY)
{
  LandingLeg &s = legs[leg];
  return s.phase == DESCENDING || s.phase == SEARCHING || s.phase == TOUCHED
             ? s.current : gaitY + s.offset;
}

bool footLanding_retry()
{
  if (!faulted)
    return false;
  if (!footSensor_mappingValid())
  {
    Serial.println(F("Landing retry rejected: invalid mapping."));
    return false;
  }

  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    if (legs[leg].phase == SUPPORT && legs[leg].supported && !footSensor_isTriggered(leg))
    {
      Serial.println(F("Landing retry rejected: restore all previously supporting feet."));
      return false;
    }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    LandingLeg &s = legs[leg];
    if (s.phase == SEARCHING)
      s.searched = s.lastSearch = millis();
    else if (s.phase == DESCENDING)
    {
      s.start = s.current;
      s.began = s.phaseStarted = millis();
    }
  }
  faulted = false;
  timedOut = false;
  restoring = false;
  Serial.println(F("Landing retry accepted; gait waits for contact."));
  return true;
}

  bool footLanding_timedOut() { return timedOut; }
  bool footLanding_restoring() { return restoring; }
  bool footLanding_restored() { return restored; }

  bool footLanding_allSupported()
  {
    return footSensor_mappingValid() && footSensor_isAllLegsTriggered();
  }

  void footLanding_record(const long x[], const long y[], const long z[], const long rotation[])
  {
    if (restoring || faulted)
      return;
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      FootHistory &h = histories[leg];
      FootFrame frame = {short(x[leg]), short(y[leg]), short(z[leg]), short(rotation[leg])};
      if (legs[leg].phase == SUPPORT && footSensor_isTriggered(leg))
      {
        h.frames[0] = frame;
        h.count = 1;
        h.valid = true;
      }
      else if (h.valid)
      {
        if (h.count == HISTORY_CAPACITY)
          h.valid = false; // Never silently replay a truncated path.
        else
          h.frames[h.count++] = frame;
      }
    }
  }

  bool footLanding_beginRestore()
  {
    if (!timedOut || restoring)
      return false;
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      FootHistory &h = histories[leg];
      h.needed = legs[leg].phase != SUPPORT;
      if (h.needed && (!h.valid || !h.count))
      {
        Serial.println(F("Return path unavailable; opposite travel requires all six contacts."));
        return false;
      }
    }
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
      histories[leg].cursor = histories[leg].count ? histories[leg].count - 1 : 0;
    restoreStarted = restoreFrame = millis();
    restoring = true;
    restored = false;
    faulted = timedOut = false;
    Serial.println(F("Recovery: retracing airborne feet; support-foot targets held."));
    return true;
  }

  void footLanding_restoreTargets(long x[], long y[], long z[], long rotation[])
  {
    if (!restoring || faulted)
      return;
    uint32_t now = millis();
    bool due = uint32_t(now - restoreFrame) >= 20;
    if (due)
      restoreFrame = now;
    bool complete = true;
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      FootHistory &h = histories[leg];
      if (!h.needed)
      {
        if (!footSensor_isTriggered(leg))
        {
          footLanding_fail("support lost during path recovery", leg);
          return;
        }
        continue;
      }
      if (due && h.cursor)
        --h.cursor;
      FootFrame &frame = h.frames[h.cursor];
      x[leg] = frame.x;
      y[leg] = frame.y;
      z[leg] = frame.z;
      rotation[leg] = frame.rotation;
      complete &= h.cursor == 0 && footSensor_isTriggered(leg);
    }
    if (complete)
    {
      for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
      {
        legs[leg].phase = SUPPORT;
        legs[leg].offset = y[leg];
        legs[leg].supported = true;
      }
      restoring = false;
      restored = true;
      Serial.println(F("Recovery: all six feet contact; preparing opposite travel."));
    }
    else if (uint32_t(now - restoreStarted) >= 10000)
    {
      footLanding_fail("return-path contact timeout", NUM_OF_LEGS);
      restoring = false;
    }
  }
