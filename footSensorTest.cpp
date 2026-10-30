#include "HexCfg.h"
#include "footSensor.h"
#include "footSensorTest.h"

#ifdef FOOT_SENSOR_TEST_ENABLED
// Exclude other servo-driving modes so this test cannot unexpectedly move the remaining legs.
#if !defined(FOOT_SENSOR_ENABLED) || defined(CALIBRATION_ENABLED) || defined(DEBUG_LEG)
#error "Foot test requires FOOT_SENSOR_ENABLED and excludes calibration/DEBUG_LEG"
#endif

enum TestState
{
  IDLE,
  POSITIONED,
  LOWERING,
  SEARCHING,
  CONTACT,
  NO_GROUND,
  STOPPED,
  FAULT
};
static TestState state;
static FootTestMove moveFoot;
static short targetY;
static uint32_t started;
static uint32_t lastStep;
static char command[16];
static byte commandLength;
static bool overflow;
static const char *const names[] = {"IDLE",    "POSITIONED", "LOWERING", "SEARCHING",
                                    "CONTACT", "NO_GROUND",  "STOPPED",  "FAULT"};

static void setState(TestState next)
{
  state = next;
  Serial.print(F("Foot test: "));
  Serial.print(names[state]);
  Serial.print(F(" target Y mm="));
  Serial.println(targetY);
}

void footSensorTest_init(FootTestMove move)
{
  moveFoot = move;
  state = IDLE;
  targetY = 0;
  commandLength = 0;
  overflow = false;
  Serial.println(F("SUPPORTED BODY TEST ONLY. App/walking disabled. No automatic reverse."));
  Serial.println(F("POSITION moves the selected leg to the configured start pose."));
  Serial.println(
      F("Then LOWER searches for contact. STOP holds the last target; it does not remove torque."));
  Serial.println(F("Commands (newline terminated): POSITION, LOWER, STOP, STATUS"));
}

static bool limitsValid()
{
  // Reject incomplete measurements and excessive bounds before any POSITION command can drive PWM.
  if (FOOT_TEST_LEG < 0 || FOOT_TEST_LEG >= NUM_OF_LEGS)
  {
    return false;
  }
  return footSensor_mappingValid() && FOOT_TEST_START_Y_MM > 0 &&
         FOOT_TEST_GROUND_Y_MM >= FOOT_TEST_START_Y_MM &&
         long(FOOT_TEST_GROUND_Y_MM) + FOOT_TEST_EXTRA_DOWN_MM <= 32767 &&
         FOOT_TEST_EXTRA_DOWN_MM > 0 && FOOT_TEST_EXTRA_DOWN_MM <= 100 &&
         FOOT_TEST_SPEED_MM_S > 0 && FOOT_TEST_SPEED_MM_S <= 50 && FOOT_TEST_TIMEOUT_MS > 0 &&
         FOOT_TEST_TIMEOUT_MS <= 60000;
}

static void processCommand()
{
  if (strcmp(command, "STOP") == 0)
  {
    setState(STOPPED);
  }
  else if (strcmp(command, "STATUS") == 0)
  {
    setState(state);
    footSensor_report();
    Serial.println(limitsValid() ? F("Configuration valid.")
                                 : F("MOTION LOCKED: configure mapping and measured limits."));
    Serial.print(F("Start Y="));
    Serial.print(FOOT_TEST_START_Y_MM);
    Serial.print(F(" ground Y="));
    Serial.print(FOOT_TEST_GROUND_Y_MM);
    Serial.print(F(" max Y="));
    Serial.println(long(FOOT_TEST_GROUND_Y_MM) + FOOT_TEST_EXTRA_DOWN_MM);
  }
  else if (strcmp(command, "POSITION") == 0)
  {
    // Positioning is explicit so entering test firmware does not immediately move the selected leg.
    if (state == LOWERING || state == SEARCHING)
    {
      Serial.println(F("STOP the search before repositioning."));
    }
    else if (!limitsValid())
    {
      Serial.println(F("POSITION rejected: mapping/limits invalid."));
    }
    else if (!moveFoot(FOOT_TEST_LEG, FOOT_TEST_START_Y_MM))
    {
      setState(FAULT);
    }
    else
    {
      targetY = FOOT_TEST_START_Y_MM;
      setState(POSITIONED);
    }
  }
  else if (strcmp(command, "LOWER") == 0)
  {
    // A known start and released switch distinguish a real new contact from a stuck input.
    if (state != POSITIONED || !footSensor_isReleasedStable(FOOT_TEST_LEG))
    {
      Serial.println(F("LOWER rejected: POSITION first, allow pose to settle, and verify stable "
                       "released switch."));
    }
    else
    {
      started = lastStep = millis();
      setState(LOWERING);
    }
  }
  else
  {
    Serial.println(F("Unknown command. Use POSITION, LOWER, STOP, STATUS."));
  }
}

void footSensorTest_update()
{
  // Bound serial work so a noisy sender cannot starve switch sampling.
  for (byte count = 0; count < 32 && Serial.available(); ++count)
  {
    char ch = Serial.read();
    if (ch == '\r')
    {
      continue;
    }
    if (ch == '\n')
    {
      command[commandLength] = '\0';
      if (overflow)
      {
        Serial.println(F("Command too long; discarded."));
      }
      else if (commandLength)
      {
        processCommand();
      }
      commandLength = 0;
      overflow = false;
    }
    else if (commandLength < sizeof(command) - 1)
    {
      // Accept Serial monitor typing in either case without changing the command protocol.
      command[commandLength++] = (ch >= 'a' && ch <= 'z') ? ch - 'a' + 'A' : ch;
    }
    else
    {
      overflow = true;
    }
  }

  if (state != LOWERING && state != SEARCHING)
  {
    return;
  }
  uint32_t now = millis();
  if (footSensor_isTriggered(FOOT_TEST_LEG))
  {
    setState(CONTACT);
  }
  else if (uint32_t(now - started) >= FOOT_TEST_TIMEOUT_MS)
  {
    // Include contact pauses in the total timeout so a bouncing switch cannot prolong the test
    // forever.
    setState(NO_GROUND);
  }
  else if (footSensor_isRawTriggered(FOOT_TEST_LEG) || !footSensor_isReleasedStable(FOOT_TEST_LEG))
  {
    lastStep = now; // Pause immediately during contact debounce; never catch up.
  }
  else
  {
    const uint32_t stepMs =
        (1000U + FOOT_TEST_SPEED_MM_S - 1) / (FOOT_TEST_SPEED_MM_S > 0 ? FOOT_TEST_SPEED_MM_S : 1);
    if (uint32_t(now - lastStep) < stepMs)
    {
      return;
    }
    lastStep = now;
    // One millimeter per due update avoids a catch-up jump after blocking I2C or Serial work.
    if (targetY >= long(FOOT_TEST_GROUND_Y_MM) + FOOT_TEST_EXTRA_DOWN_MM)
    {
      setState(NO_GROUND);
      return;
    }
    short nextY = targetY + 1;
    if (!moveFoot(FOOT_TEST_LEG, nextY))
    {
      setState(FAULT);
      return;
    }
    targetY = nextY;
    if (targetY > FOOT_TEST_GROUND_Y_MM && state != SEARCHING)
    {
      setState(SEARCHING);
    }
  }
}
#endif
