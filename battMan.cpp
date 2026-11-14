#include "driver/ads1115.h"
#include "driver/buzzer.h"
#include "HexCfg.h"
#include "battMan.h"
#include <math.h>
#include "serialOutput.h"

static bool isSanityCheckOk(float cell1, float cell2);
static float cachedCell1 = NAN, cachedCell2 = NAN, pendingCell1;
static uint32_t lastPair, lastAttempt;
static bool sampled, attempted, converting, powerOk;
static byte channel;
static bool faultReported;

static bool checkSample(float cell1, float cell2)
{
  bool retVal = true;

#ifdef DEBUG_BATTERY
  Serial.print("Cell_1: ");
  Serial.println(cell1);
  Serial.print("Cell_2: ");
  Serial.println(cell2);
#endif

  // static buffers hold the last BATT_STABLE_COUNT readings
  static float buf1[BATT_STABLE_COUNT];
  static float buf2[BATT_STABLE_COUNT];
  static uint8_t idx = 0;
  static bool filled = false;

  buf1[idx] = cell1;
  buf2[idx] = cell2;
  idx++;
  if (idx >= BATT_STABLE_COUNT)
  {
    idx = 0;
    filled = true;
  }

  // If buffer not yet filled, be conservative and don't trigger sanity check
  // failures (avoid false alarms during startup). Return OK.
  if (!filled)
  {
    return true;
  }

  // Compute min/max over buffer for each cell to see if readings are stable
  float min1 = buf1[0], max1 = buf1[0];
  float min2 = buf2[0], max2 = buf2[0];
  for (uint8_t i = 1; i < BATT_STABLE_COUNT; ++i)
  {
    if (buf1[i] < min1)
    {
      min1 = buf1[i];
    }
    if (buf1[i] > max1)
    {
      max1 = buf1[i];
    }
    if (buf2[i] < min2)
    {
      min2 = buf2[i];
    }
    if (buf2[i] > max2)
    {
      max2 = buf2[i];
    }
  }

  float delta1 = max1 - min1;
  float delta2 = max2 - min2;

#ifdef DEBUG_BATTERY
  Serial.print("Delta1: ");
  Serial.println(delta1);
  Serial.print("Delta2: ");
  Serial.println(delta2);
#endif

  // Require both channels to be stable before running the sanity checks.
  if (delta1 <= BATT_STABLE_DELTA && delta2 <= BATT_STABLE_DELTA)
  {
    if (!isSanityCheckOk(cell1, cell2))
    {
      buzzer_on();
      retVal = false;
    }
  }

  return retVal;
}

static void reportFault(const char *reason)
{
  if (!faultReported)
  {
    Serial.println(reason);
    faultReported = true;
    buzzer_on();
  }
  powerOk = false;
}

void battMan_update()
{
  uint32_t now = millis();
  if (sampled && uint32_t(now - lastPair) > 500)
  {
    reportFault("Battery: stale readings; motion blocked.");
  }
  if (!converting)
  {
    if (attempted && uint32_t(now - lastAttempt) < 100)
    {
      return;
    }
    attempted = true;
    lastAttempt = now;
    channel = CELL1;
    converting = ads1115_start(channel);
    if (!converting)
    {
      reportFault("Battery: ADC start failed; motion blocked.");
    }
    return;
  }
  float voltage;
  AdcReadStatus status = ads1115_poll(voltage);
  if (status == AdcReadStatus::Pending)
  {
    return;
  }
  converting = false;
  if (status == AdcReadStatus::Error || !isfinite(voltage))
  {
    reportFault("Battery: ADC read failed; motion blocked.");
    return;
  }
  if (channel == CELL1)
  {
    pendingCell1 = voltage + BATT_OFFSET_1;
    channel = CELL2;
    converting = ads1115_start(channel);
    if (!converting)
    {
      reportFault("Battery: ADC start failed; motion blocked.");
    }
    return;
  }
  cachedCell1 = pendingCell1;
  cachedCell2 = voltage + BATT_OFFSET_2;
  sampled = true;
  lastPair = now;
  if (faultReported)
  {
    Serial.println(F("Battery: fresh ADC readings restored."));
    faultReported = false;
  }
  bool safe = checkSample(cachedCell1, cachedCell2);
  if (!safe && powerOk)
  {
    Serial.println(F("Battery: voltage protection triggered; motion blocked."));
  }
  powerOk = safe;
}

bool battMan_isPowerOk()
{
  if (!sampled || faultReported || uint32_t(millis() - lastPair) > 500)
  {
    reportFault("Battery: readings unavailable; motion blocked.");
    return false;
  }
  return powerOk;
}

/**
 * @brief Perform sanity checks on two battery cell voltages.
 *
 * Checks for low voltage (below BATT_VOLT_LOW) and large imbalance (>= 0.2V).
 * This function is internal to the battery manager.
 *
 * @param cell1 Voltage of cell 1 in volts
 * @param cell2 Voltage of cell 2 in volts
 * @return true if voltages pass checks, false otherwise
 */
static bool isSanityCheckOk(float cell1, float cell2)
{
  bool retVal = true;
  // low voltage
  if (cell1 < BATT_VOLT_LOW || cell2 < BATT_VOLT_LOW)
  {
    retVal = false;
  }
  // unbalanced cell
  if (fabsf(cell1 - cell2) >= 0.2f)
  {
    retVal = false;
  }
  return retVal;
}

void battMan_getVoltages(float *cell1, float *cell2)
{
  bool valid = sampled && !faultReported && uint32_t(millis() - lastPair) <= 500;
  if (cell1)
  {
    *cell1 = valid ? cachedCell1 : NAN;
  }
  if (cell2)
  {
    *cell2 = valid ? cachedCell2 : NAN;
  }
}