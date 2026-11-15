#include "footLanding.h"
#include "footSensor.h"
#include <math.h>
#include "serialOutput.h"

enum LandingPhase
{
  // Track each foot independently so one early touchdown does not finish a whole tripod landing.
  SUPPORT,
  SWING,
  DESCENDING,
  SEARCHING,
  TOUCHED
};
struct LandingLeg
{
  LandingPhase phase;
  long offset;
  long current;
  long target;
  long planned;
  long start;
  long supportY;
  bool released;
  bool supported;
  bool nominal;
  uint32_t began;
  uint32_t phaseStarted;
  uint32_t duration;
  uint32_t searched;
  uint32_t lastSearch;
  uint32_t releasedAt;
  bool releasePending;
  bool supportSearch;
  bool seating;
  long seatStartY;
  uint32_t seatStarted;
  uint32_t lastSeat;
  // Support foot that lost ground (e.g. table edge): held at its last support height without
  // searching again until it touches or its next swing places it elsewhere.
  bool lost;
  bool recoveryMove;
};
static LandingLeg legs[NUM_OF_LEGS];
static bool faulted;
static FootFault faultKind;
static long supportX[NUM_OF_LEGS], supportZ[NUM_OF_LEGS];
static bool supportGeometryValid;
static char faultReason[112];
static bool engaged;
static bool timedOut;
static bool supportLost;
// Only the leg whose search actually expired is a recovery failure; other searches continue.
static byte timeoutLeg = NUM_OF_LEGS;
static const char *const names[] = {"RF", "RM", "RR", "LF", "LM", "LR"};
static LandingLeg savedLegs[NUM_OF_LEGS];
static bool savedEngaged;

static_assert(FOOT_WALK_EXTRA_DOWN_MM > 0 && FOOT_WALK_SEARCH_MM_S > 0 && FOOT_WALK_TIMEOUT_MS > 0,
              "Walking search limits must be positive");
static_assert(FOOT_WALK_SEAT_MM > 0 && FOOT_WALK_SEAT_MM_S > 0,
              "Walking seating distance and speed must be positive");

void footLanding_init()
{
  // Reset transactions, not the footprint: activation computes geometry before initializing
  // landing.
  memset(legs, 0, sizeof(legs));
  faulted = engaged = false;
  faultReason[0] = 0;
  timedOut = supportLost = false;
  timeoutLeg = NUM_OF_LEGS;
}

void footLanding_save()
{
  memcpy(savedLegs, legs, sizeof(legs));
  savedEngaged = engaged;
}

void footLanding_rollback()
{
  memcpy(legs, savedLegs, sizeof(legs));
  engaged = savedEngaged;
}

void footLanding_delay(uint32_t ms)
{
  // Paused time must not count toward descent progress, grace, seating, or search timeouts.
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    LandingLeg &s = legs[leg];
    s.began += ms;
    s.phaseStarted += ms;
    s.searched += ms;
    s.lastSearch += ms;
    s.releasedAt += ms;
    s.seatStarted += ms;
    s.lastSeat += ms;
  }
}

long footLanding_searchDepth(byte leg)
{
  const LandingLeg &s = legs[leg];
  if (s.phase != SEARCHING && !s.seating)
  {
    return 0;
  }
  return s.current > s.target ? s.current - s.target : 0;
}

void footLanding_noGround(byte leg)
{
  // Classify unreachable search depth exactly like a missing-ground timeout for recovery.
  const LandingLeg &s = legs[leg];
  timedOut = s.released;
  supportLost = s.supportSearch;
  timeoutLeg = leg;
  footLanding_fail(s.supportSearch ? "support search beyond leg reach"
                                   : "no ground within leg reach",
                   leg);
}

void footLanding_resume(const long plannedY[], const long offsets[])
{
  // Re-enable sensing at the retained pose instead of snapping searched feet back to nominal
  // ground.
  footLanding_init();
  engaged = true;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    legs[leg].planned = plannedY[leg];
    legs[leg].offset = offsets[leg];
    legs[leg].current = plannedY[leg] + offsets[leg];
    legs[leg].supportY = legs[leg].current;
    legs[leg].supported = footSensor_isTriggered(leg);
  }
}

void footLanding_fail(const char *reason, byte leg, FootFault kind)
{
  // A later sensor error must not downgrade a battery/geometry fault into a cancelable sensor HOLD.
  if (!faulted || kind != FOOT_FAULT_SENSOR)
  {
    faultKind = kind;
    snprintf(faultReason, sizeof(faultReason), "%s (%s)", reason,
             leg < NUM_OF_LEGS ? names[leg] : "all");
    Serial.print(F("Foot landing HOLD: "));
    Serial.print(reason);
    Serial.print(F(" leg="));
    Serial.println(leg < NUM_OF_LEGS ? names[leg] : "all");
    Serial.println(
        kind == FOOT_FAULT_SENSOR
            ? F("Protection remains enabled. Center inputs; Foot sensor can cancel sensor recovery "
                "and hold this pose.")
            : F("Battery/geometry fault remains latched. Correct the fault and restart."));
  }
  faulted = true;
}

const char *footLanding_reason()
{
  return faultReason;
}
bool footLanding_hardFault()
{
  return faulted && faultKind != FOOT_FAULT_SENSOR;
}

void footLanding_supportGeometry(const long x[], const long z[])
{
  memcpy(supportX, x, sizeof(supportX));
  memcpy(supportZ, z, sizeof(supportZ));
  supportGeometryValid = true;
}

static bool insideEdge(byte a, byte b, int orientation)
{
  float dx = supportX[b] - supportX[a], dz = supportZ[b] - supportZ[a];
  float cross = dx * -supportZ[a] - dz * -supportX[a];
  return orientation * cross >= FOOT_WALK_SUPPORT_MARGIN_MM * sqrtf(dx * dx + dz * dz);
}

bool footLanding_supportReady(byte exclude)
{
  if (!supportGeometryValid || !footSensor_mappingValid())
  {
    return false;
  }
  byte mask = footSensor_getSensorMask() & ~exclude;
  // Check actual contact placement, not an all-six count; four distributed feet can still support a
  // lift. Hull edges avoid rejecting a valid polygon just because its center lies on a
  // triangulation diagonal. The origin is only an estimated body-center projection, not measured
  // load or center of mass.
  bool polygon = false;
  for (byte a = 0; a < NUM_OF_LEGS; ++a)
  {
    for (byte b = a + 1; b < NUM_OF_LEGS; ++b)
    {
      if (!(mask & (1 << a)) || !(mask & (1 << b)))
      {
        continue;
      }
      bool positive = false, negative = false;
      for (byte c = 0; c < NUM_OF_LEGS; ++c)
      {
        if (mask & (1 << c))
        {
          long area = (supportX[b] - supportX[a]) * (supportZ[c] - supportZ[a]) -
                      (supportZ[b] - supportZ[a]) * (supportX[c] - supportX[a]);
          positive |= area > 0;
          negative |= area < 0;
        }
      }
      if (positive == negative)
      {
        continue;
      }
      polygon = true;
      if (!insideEdge(a, b, positive ? 1 : -1))
      {
        return false;
      }
    }
  }
  return polygon;
}

bool footLanding_liftReady(const long nextY[])
{
  // Evaluate support after the proposed lift; descending/searching feet cannot be counted as
  // anchors.
  byte exclude = 0;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (nextY[leg] < 0 || legs[leg].phase == DESCENDING || legs[leg].phase == SEARCHING)
    {
      exclude |= 1 << leg;
    }
  }
  return footLanding_supportReady(exclude);
}

void footLanding_plan(const long previousY[], const long plannedY[], uint32_t duration,
                      uint32_t firstFrameMs)
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
    {
      s.supported = footSensor_isTriggered(leg);
    }
    if (s.phase == SUPPORT)
    {
      s.planned = plannedY[leg];
      s.current = plannedY[leg] + s.offset;
    }
    if (plannedY[leg] < previousY[leg] && plannedY[leg] < 0)
    {
      if (s.phase == SUPPORT || s.phase == TOUCHED)
      {
        s.released = false;
        // A new swing starts its own ground search; the previous touchdown offset must not
        // accumulate.
        if (!s.recoveryMove)
        {
          s.offset = 0;
        }
      }
      s.phase = SWING;
      s.seating = false;
      s.supported = false;
      s.lost = false;
      s.releasePending = s.supportSearch = false;
    }
    else if (plannedY[leg] > previousY[leg] && previousY[leg] < 0)
    {
      s.nominal = plannedY[leg] >= 0;
      if (s.phase == TOUCHED)
      {
        // Keep early-contact height fixed while the nominal gait finishes lowering this foot.
        s.offset = s.current - plannedY[leg];
        if (s.nominal)
        {
          s.phase = SUPPORT;
        }
        continue;
      }
      s.start = previousY[leg] + s.offset;
      s.current = s.start;
      s.target = plannedY[leg] + s.offset;
      s.planned = plannedY[leg];
      s.began = s.phaseStarted = now;
      s.duration = duration ? duration : 1;
      s.phase = DESCENDING;
      if (footSensor_isReleasedStable(leg))
      {
        uint32_t firstFrame = firstFrameMs < s.duration ? firstFrameMs : s.duration;
        s.current = s.start + (s.target - s.start) * long(firstFrame) / s.duration;
        s.began -= firstFrame;
      }
    }
  }
}

void footLanding_update()
{
  if (!engaged || faulted)
  {
    return;
  }
  uint32_t now = millis();
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    LandingLeg &s = legs[leg];
    if (s.phase != SUPPORT && footSensor_isReleasedStable(leg))
    {
      s.released = true;
    }
    if (s.phase == SUPPORT || s.phase == TOUCHED)
    {
      if (s.lost)
      {
        // Recovery already gave up on this foot; regained contact makes it a support foot again.
        if (footSensor_isTriggered(leg))
        {
          s.lost = false;
          s.supported = true;
          s.releasePending = false;
          s.supportY = s.current;
          s.recoveryMove = false;
        }
        continue;
      }
      // A brief release may be switch bounce or uneven loading, not immediate loss of walkability.
      if (footSensor_isReleasedStable(leg))
      {
        if (!s.releasePending)
        {
          s.releasePending = true;
          s.releasedAt = now;
        }
        if (uint32_t(now - s.releasedAt) >= FOOT_WALK_SUPPORT_GRACE_MS)
        {
          // Search from the last support target so a persistently missing switch causes real extra
          // descent.
          s.supported = false;
          s.supportSearch = s.nominal = s.released = true;
          s.target = s.current;
          s.phase = SEARCHING;
          s.seating = false;
          s.searched = s.lastSearch = now;
          Serial.print(F("Support search "));
          Serial.println(names[leg]);
        }
      }
      else
      {
        s.releasePending = false;
      }
      continue;
    }
    if (s.phase != DESCENDING && s.phase != SEARCHING)
    {
      continue;
    }
    if (s.released && footSensor_isTriggered(leg))
    {
      // Only a release-then-contact sequence is a new touchdown; a stuck-pressed switch cannot
      // qualify.
      s.offset = s.current - s.planned;
      s.phase = s.nominal ? SUPPORT : TOUCHED;
      s.supported = true;
      s.seating = false;
      s.releasePending = s.supportSearch = false;
      s.supportY = s.current;
      s.recoveryMove = false;
      Serial.print(F("Touchdown "));
      Serial.print(names[leg]);
      Serial.print(F(" gait Y="));
      Serial.println(s.current);
      continue;
    }
    if (s.phase == SEARCHING && uint32_t(now - s.searched) > FOOT_WALK_TIMEOUT_MS)
    {
      // A failed swing landing or a support foot that lost ground (table edge) may trigger
      // retreat; a switch that never released may not.
      timedOut = s.released;
      supportLost = s.supportSearch;
      timeoutLeg = leg;
      footLanding_fail(s.supportSearch ? "support search timeout"
                       : s.released    ? "no ground within landing timeout"
                                       : "switch never released",
                       leg);
      return;
    }
    // First raw contact switches to bounded slow seating instead of freezing a lightly engaged
    // switch. Require prior stable release so a stuck-pressed switch cannot authorize extra
    // downward motion.
    if (!s.seating && s.released && footSensor_isRawTriggered(leg))
    {
      s.seating = true;
      s.seatStartY = s.current;
      s.seatStarted = s.lastSeat = now;
      Serial.print(F("Foot seating "));
      Serial.println(names[leg]);
    }
    if (s.seating)
    {
      // Never restart this budget on bounce or catch up missed output frames with a larger step.
      const uint32_t seatStepMs = (1000U + FOOT_WALK_SEAT_MM_S - 1) / FOOT_WALK_SEAT_MM_S;
      const uint32_t seatTimeout =
          FOOT_WALK_SEAT_MM * seatStepMs +
          (BUILDCFG_FOOT_SENSOR_DEBOUNCE_ENABLED ? FOOT_SENSOR_CONTACT_MS : 0U) +
          FOOT_SENSOR_SAMPLE_MS;
      if (uint32_t(now - s.seatStarted) > seatTimeout)
      {
        footLanding_fail("contact did not stabilize within seating limit", leg);
        return;
      }
      if (uint32_t(now - s.lastSeat) >= seatStepMs &&
          s.current < s.seatStartY + FOOT_WALK_SEAT_MM &&
          s.current < s.target + FOOT_WALK_EXTRA_DOWN_MM)
      {
        ++s.current;
        s.lastSeat = now;
        Serial.print(F("Foot seating "));
        Serial.print(names[leg]);
        Serial.print(F(" extra mm="));
        Serial.println(s.current - s.seatStartY);
      }
      continue;
    }
    // Without prior release, raw contact still pauses descent; it is not proof of a new touchdown.
    if (footSensor_isRawTriggered(leg) || !footSensor_isReleasedStable(leg))
    {
      if (s.phase == DESCENDING)
      {
        // Restart interpolation from the held target so an unconfirmed switch cannot cause a
        // catch-up jump.
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
        {
          s.phase = SWING;
        }
        else
        {
          s.phase = SEARCHING;
          s.searched = s.lastSearch = now;
          Serial.print(F("Landing search "));
          Serial.println(names[leg]);
        }
      }
    }
    else if (uint32_t(now - s.lastSearch) >=
             (1000U + FOOT_WALK_SEARCH_MM_S - 1) / FOOT_WALK_SEARCH_MM_S)
    {
      s.lastSearch = now;
      // Bound travel independently of time: reaching the distance cap waits for the search timeout.
      if (s.current >= s.target + FOOT_WALK_EXTRA_DOWN_MM)
      {
        continue;
      }
      ++s.current;
      Serial.print(F("Foot search "));
      Serial.print(names[leg]);
      Serial.print(F(" extra mm="));
      Serial.println(s.current - s.target);
    }
  }
}

bool footLanding_busy()
{
  // Gate the whole next gait step until every pending foot has landed or finished its descent
  // segment.
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (legs[leg].phase == DESCENDING || legs[leg].phase == SEARCHING)
    {
      return true;
    }
  }
  return false;
}

bool footLanding_fault()
{
  return faulted;
}
bool footLanding_engaged()
{
  return engaged;
}
bool footLanding_direct(byte leg)
{
  // Contact-critical targets must not leave a smoothed downward command queued after touchdown.
  return legs[leg].phase == DESCENDING || legs[leg].phase == SEARCHING ||
         legs[leg].phase == TOUCHED || legs[leg].supported;
}

long footLanding_y(byte leg, long gaitY)
{
  LandingLeg &s = legs[leg];
  return s.phase == DESCENDING || s.phase == SEARCHING || s.phase == TOUCHED ? s.current
                                                                             : gaitY + s.offset;
}

long footLanding_horizontal(byte leg, long previous, long planned)
{
  const LandingLeg &s = legs[leg];
  if (s.phase != DESCENDING || s.target <= s.start)
  {
    return planned;
  }
  // X/Z/rotation must follow actual descent, not jump ahead while the foot is still fully lifted.
  long progress = s.current - s.start;
  long distance = s.target - s.start;
  if (progress < 0)
  {
    progress = 0;
  }
  else if (progress > distance)
  {
    progress = distance;
  }
  return previous + (planned - previous) * progress / distance;
}

bool footLanding_retry()
{
  if (!faulted)
  {
    return false;
  }
  if (footLanding_hardFault())
  {
    // Automatic recovery may retry contact placement, never unreachable geometry or low power.
    Serial.println(F("Landing retry rejected: battery/geometry fault requires restart."));
    return false;
  }
  if (!footSensor_mappingValid())
  {
    Serial.println(F("Landing retry rejected: invalid mapping."));
    return false;
  }

  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (legs[leg].phase == SUPPORT && legs[leg].supported && !footSensor_isTriggered(leg))
    {
      Serial.println(F("Landing retry rejected: restore all previously supporting feet."));
      return false;
    }
  }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    LandingLeg &s = legs[leg];
    // Retrying must not grant a bouncing contact a fresh seating distance or erase its used budget.
    if (s.phase == SEARCHING && s.supportSearch)
    {
      // No ground under this support foot (table edge): stop searching at its last support height.
      s.current = s.target;
      s.phase = SUPPORT;
      s.supportSearch = s.seating = s.supported = s.releasePending = false;
      s.lost = true;
    }
    else if (s.phase == SEARCHING)
    {
      s.searched = s.lastSearch = millis();
    }
    else if (s.phase == DESCENDING)
    {
      s.start = s.current;
      s.began = s.phaseStarted = millis();
    }
  }
  faulted = false;
  timedOut = supportLost = false;
  timeoutLeg = NUM_OF_LEGS;
  Serial.println(F("Landing retry accepted; gait waits for contact."));
  return true;
}

byte footLanding_recoveryMask()
{
  byte mask = 0;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (legs[leg].lost || leg == timeoutLeg)
    {
      mask |= 1 << leg;
    }
  }
  return mask;
}

bool footLanding_startRecovery()
{
  if (!faulted || faultKind != FOOT_FAULT_SENSOR || !timedOut ||
      !footSensor_mappingValid())
  {
    return false;
  }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (legs[leg].phase == SUPPORT && legs[leg].supported &&
        !footSensor_isTriggered(leg))
    {
      return false;
    }
  }
  bool found = false;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    LandingLeg &s = legs[leg];
    // A search still in progress has not failed: keep it searching and let recovery wait for it.
    if (s.phase == SEARCHING && (leg == timeoutLeg || s.lost))
    {
      s.current = s.supportY;
      s.offset = s.supportY - s.planned;
      s.phase = SUPPORT;
      s.supportSearch = s.seating = s.supported = s.releasePending = false;
      s.lost = s.recoveryMove = true;
      found = true;
    }
    else if (s.lost)
    {
      s.current = s.supportY;
      s.offset = s.supportY - s.planned;
      s.recoveryMove = true;
      found = true;
    }
  }
  if (!found)
  {
    return false;
  }
  faulted = timedOut = supportLost = false;
  timeoutLeg = NUM_OF_LEGS;
  return true;
}

bool footLanding_clearRecovery()
{
  if (faulted || footLanding_busy() || !footSensor_isAllLegsTriggered())
  {
    return false;
  }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    legs[leg].lost = legs[leg].recoveryMove = false;
  }
  timedOut = supportLost = false;
  timeoutLeg = NUM_OF_LEGS;
  return true;
}

bool footLanding_timedOut()
{
  return timedOut;
}
bool footLanding_supportLost()
{
  return supportLost;
}
byte footLanding_lostMask()
{
  byte mask = 0;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (legs[leg].lost)
    {
      mask |= 1 << leg;
    }
  }
  return mask;
}
bool footLanding_allSupported()
{
  // Recovery needs a usable support pattern, not necessarily six functioning/contacting switches.
  return footLanding_supportReady();
}
