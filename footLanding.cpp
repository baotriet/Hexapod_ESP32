#include "footLanding.h"
#include "footSensor.h"
#include <math.h>

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
static bool restoring;
static bool restoreForLoss;
static bool restored;
static uint32_t restoreStarted;
static uint32_t restoreFrame;
// Bound ESP32 RAM usage; an overflow invalidates recovery rather than inventing a shortened path.
static const unsigned HISTORY_CAPACITY = 256;
struct FootFrame
{
  short x, y, z, rotation;
};
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
static LandingLeg savedLegs[NUM_OF_LEGS];
static unsigned savedCount[NUM_OF_LEGS], savedCursor[NUM_OF_LEGS];
static bool savedEngaged, savedRestoring, savedRestored;
static uint32_t savedRestoreFrame;

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
  timedOut = supportLost = restoring = restored = false;
  memset(histories, 0, sizeof(histories));
}

void footLanding_save()
{
  memcpy(savedLegs, legs, sizeof(legs));
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    savedCount[leg] = histories[leg].count;
    savedCursor[leg] = histories[leg].cursor;
  }
  savedEngaged = engaged;
  savedRestoring = restoring;
  savedRestored = restored;
  savedRestoreFrame = restoreFrame;
}

void footLanding_rollback()
{
  // History frames are only appended after output, so restoring counts/cursors undoes plan/replay.
  memcpy(legs, savedLegs, sizeof(legs));
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    histories[leg].count = savedCount[leg];
    histories[leg].cursor = savedCursor[leg];
  }
  engaged = savedEngaged;
  restoring = savedRestoring;
  restored = savedRestored;
  restoreFrame = savedRestoreFrame;
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
  restoreStarted += ms;
  restoreFrame += ms;
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
  // Same classification as the search timeout so the existing retrace/reverse recovery applies.
  const LandingLeg &s = legs[leg];
  timedOut = s.released;
  supportLost = s.supportSearch;
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
        s.offset = 0;
        // Keep the last grounded frame as the beginning of this swing's return path.
        histories[leg].count = histories[leg].valid ? 1 : 0;
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
  if (!engaged || faulted || restoring)
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
          FOOT_WALK_SEAT_MM * seatStepMs + FOOT_SENSOR_CONTACT_MS + FOOT_SENSOR_SAMPLE_MS;
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
      // No ground under this support foot (table edge): raise it back to its last support height
      // and stop searching, so the reverse walk can step it back onto the surface.
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
  restoring = false;
  Serial.println(F("Landing retry accepted; gait waits for contact."));
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
void footLanding_abandon(byte leg)
{
  // A re-placed foot found no ground: stop its search at the planned landing height and treat it
  // like a support foot that lost ground, so the reverse walk is not blocked by it.
  if (leg >= NUM_OF_LEGS || legs[leg].phase != SEARCHING)
  {
    return;
  }
  LandingLeg &s = legs[leg];
  s.current = s.target;
  s.offset = s.target - s.planned;
  s.phase = SUPPORT;
  s.supportSearch = s.seating = s.supported = s.releasePending = false;
  s.lost = true;
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
bool footLanding_restoring()
{
  return restoring;
}
bool footLanding_restored()
{
  return restored;
}

bool footLanding_allSupported()
{
  // Recovery needs a usable support pattern, not necessarily six functioning/contacting switches.
  return footLanding_supportReady();
}

void footLanding_record(const long x[], const long y[], const long z[], const long rotation[])
{
  // Record issued commands from the last grounded placement, not hypothetical future gait targets.
  if (restoring || faulted)
  {
    return;
  }
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
      {
        h.valid = false; // Never silently replay a truncated path.
      }
      else
      {
        h.frames[h.count++] = frame;
      }
    }
  }
}

bool footLanding_beginRestore()
{
  // Retrace only when all returning feet have complete history; guessing a return path can hit
  // obstacles.
  if (!timedOut || restoring)
  {
    return false;
  }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    FootHistory &h = histories[leg];
    h.needed = legs[leg].phase != SUPPORT;
    if (h.needed && (!h.valid || !h.count))
    {
      Serial.println(
          F("Return path unavailable; opposite travel requires confirmed supporting geometry."));
      return false;
    }
  }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    histories[leg].cursor = histories[leg].count ? histories[leg].count - 1 : 0;
    // A support foot that lost ground returns to its last support frame, where ground is gone.
    legs[leg].lost = legs[leg].phase == SEARCHING && legs[leg].supportSearch;
  }
  restoreStarted = restoreFrame = millis();
  restoring = true;
  restored = false;
  restoreForLoss = supportLost;
  faulted = timedOut = supportLost = false;
  Serial.println(F("Recovery: retracing airborne feet; support-foot targets held."));
  return true;
}

void footLanding_restoreTargets(long x[], long y[], long z[], long rotation[])
{
  // Hold supporting feet while replaying airborne feet backward; commanded coordinates are not
  // feedback.
  if (!restoring || faulted)
  {
    return;
  }
  uint32_t now = millis();
  bool due = uint32_t(now - restoreFrame) >= 20;
  if (due)
  {
    restoreFrame = now;
  }
  bool complete = true;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    FootHistory &h = histories[leg];
    if (!h.needed)
    {
      continue;
    }
    if (due && h.cursor)
    {
      --h.cursor;
    }
    FootFrame &frame = h.frames[h.cursor];
    x[leg] = frame.x;
    y[leg] = frame.y;
    z[leg] = frame.z;
    rotation[leg] = frame.rotation;
    // Reaching an old coordinate is not proof of ground: returning feet must also confirm contact.
    // A lost support foot only needs to reach its old support height; its ground is gone.
    complete &= h.cursor == 0 && (footSensor_isTriggered(leg) || legs[leg].lost);
  }
  byte airborne = 0;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (histories[leg].needed)
    {
      airborne |= 1 << leg;
    }
  }
  // After a support foot loses ground, the held feet are already the robot's only support; the
  // retrace only lowers airborne feet back onto ground they left, so it must not wait on them.
  if (!restoreForLoss && !footLanding_supportReady(airborne))
  {
    // Returning feet cannot supply the support needed to move themselves along the recorded path.
    footLanding_fail("insufficient support during path recovery", NUM_OF_LEGS);
    return;
  }
  if (complete)
  {
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      legs[leg].phase = SUPPORT;
      legs[leg].offset = y[leg];
      legs[leg].supported = footSensor_isTriggered(leg);
      legs[leg].supportSearch = legs[leg].seating = legs[leg].releasePending = false;
      legs[leg].lost &= !legs[leg].supported;
    }
    restoring = false;
    restored = true;
    Serial.println(
        F("Recovery: returning feet contact and support confirmed; preparing opposite travel."));
  }
  else if (uint32_t(now - restoreStarted) >= 10000)
  {
    footLanding_fail("return-path contact timeout", NUM_OF_LEGS);
    restoring = false;
  }
}
