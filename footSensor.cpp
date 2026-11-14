#include "HexCfg.h"
#include "footSensor.h"
#include "driver/pcf8575.h"
#include "serialOutput.h"

// Translate expander pins once so gait code and diagnostics always use physical leg indices.
static const int sensorPins[NUM_OF_LEGS] = {FOOT_SENSOR_RF_PIN, FOOT_SENSOR_RM_PIN,
                                            FOOT_SENSOR_RR_PIN, FOOT_SENSOR_LF_PIN,
                                            FOOT_SENSOR_LM_PIN, FOOT_SENSOR_LR_PIN};
static const char *const sensorNames[NUM_OF_LEGS] = {"RF", "RM", "RR", "LF", "LM", "LR"};
static_assert(RF == 0 && RM == 1 && RR == 2 && LF == 3 && LM == 4 && LR == 5,
              "Update sensorPins for the configured leg order");
static_assert(FOOT_SENSOR_SAMPLE_MS > 0 && FOOT_SENSOR_CONTACT_MS > 0 &&
                  FOOT_SENSOR_RELEASE_MS > 0 && FOOT_SENSOR_REPORT_MS > 0,
              "Sensor timing must be positive");

static uint8_t rawMask;
static uint8_t stablePins;
static uint8_t candidatePins;
static uint8_t sensorMask;
static uint32_t candidateSince[NUM_OF_LEGS];
static uint32_t lastSample;
#ifdef DEBUG_FOOT_SENSOR
static uint32_t lastReport;
#endif
static bool sampled;

bool footSensor_mappingValid()
{
  // Duplicate or missing assignments could stop one leg using another leg's contact.
  uint8_t used = 0;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    int pin = sensorPins[leg];
    if (pin < 0 || pin >= NUM_OF_LEGS || (used & (1U << pin)))
    {
      return false;
    }
    used |= 1U << pin;
  }
  return true;
}

void footSensor_init()
{
  rawMask = stablePins = candidatePins = sensorMask = 0;
  sampled = false;
  lastSample = millis();
#ifdef DEBUG_FOOT_SENSOR
  lastReport = lastSample;
#endif
  for (byte pin = 0; pin < NUM_OF_LEGS; ++pin)
  {
    candidateSince[pin] = lastSample;
  }
  Serial.println(footSensor_mappingValid()
                     ? F("Foot sensors: mapping configured; verify each physical leg.")
                     : F("Foot sensors: UNMAPPED. Configure footSensorCfg.h; raw bits P05..P00 are "
                         "available."));
}

void footSensor_update()
{
  uint32_t now = millis();
  if (sampled && uint32_t(now - lastSample) < FOOT_SENSOR_SAMPLE_MS)
  {
    return;
  }
  lastSample = now;
#if BUILDCFG_FOOT_SENSOR_DEBOUNCE_ENABLED
  bool first = !sampled;
#endif
  sampled = true;
  rawMask = pcf8575_getValue(0, NUM_OF_LEGS - 1);
#if BUILDCFG_FOOT_SENSOR_DEBOUNCE_ENABLED
  // Require an unchanged candidate for the full debounce interval; count time, not loop iterations.
  for (byte pin = 0; pin < NUM_OF_LEGS; ++pin)
  {
    uint8_t bit = 1U << pin;
    if (first || ((rawMask ^ candidatePins) & bit))
    {
      candidatePins = (candidatePins & ~bit) | (rawMask & bit);
      candidateSince[pin] = now;
    }
    uint32_t debounce = (candidatePins & bit) ? FOOT_SENSOR_CONTACT_MS : FOOT_SENSOR_RELEASE_MS;
    if (uint32_t(now - candidateSince[pin]) >= debounce)
    {
      stablePins = (stablePins & ~bit) | (candidatePins & bit);
    }
  }
#else
  candidatePins = stablePins = rawMask;
#endif

  uint8_t previousMask = sensorMask;
  sensorMask = 0;
  if (footSensor_mappingValid())
  {
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      if (stablePins & (1U << sensorPins[leg]))
      {
        sensorMask |= 1U << leg;
      }
    }
  }

  uint8_t changed = previousMask ^ sensorMask;
  // Named change-only messages make wiring checks readable without flooding Serial during walking.
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (changed & (1U << leg))
    {
      Serial.print(sensorNames[leg]);
      Serial.println((sensorMask & (1U << leg)) ? F(": CONTACT") : F(": RELEASED"));
    }
  }

#ifdef DEBUG_FOOT_SENSOR
  if (uint32_t(now - lastReport) >= FOOT_SENSOR_REPORT_MS)
  {
    lastReport = now;
    footSensor_report();
  }
#endif
}

void footSensor_report()
{
  if (!footSensor_mappingValid())
  {
    Serial.print(F("Foot sensors UNMAPPED: raw(P05..P00)="));
    Serial.print(rawMask, BIN);
    Serial.print(F(" stable="));
    Serial.print(stablePins, BIN);
    Serial.println(F(""));
    return;
  }
  Serial.print(F("Feet: "));
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (leg)
    {
      Serial.print(F(", "));
    }
    Serial.print(sensorNames[leg]);
    Serial.print((sensorMask & (1U << leg)) ? F("=CONTACT") : F("=RELEASED"));
  }
  Serial.println(F(""));
}

bool footSensor_isAnyLegTriggered()
{
  return sensorMask != 0;
}

bool footSensor_isAllLegsTriggered()
{
  // "All" requires every configured bit; a nonzero mask means only "any".
  return sensorMask == ((1U << NUM_OF_LEGS) - 1);
}

bool footSensor_isTriggered(byte legNr)
{
  if (legNr >= NUM_OF_LEGS)
  {
    Serial.println(F("Foot sensor: invalid leg index."));
    return false;
  }
  return (sensorMask & (1U << legNr)) != 0;
}

bool footSensor_isRawTriggered(byte legNr)
{
  // Landing can pause on the first electrical contact rather than descend through debounce.
  if (legNr >= NUM_OF_LEGS || !footSensor_mappingValid())
  {
    Serial.println(F("Foot sensor: raw contact requested with invalid leg/mapping."));
    return false;
  }
  return (rawMask & (1U << sensorPins[legNr])) != 0;
}

uint8_t footSensor_getSensorMask()
{
  return sensorMask;
}

bool footSensor_isReleasedStable(byte legNr)
{
  // A lifted foot must really release; an uninitialized or stuck-pressed switch is not touchdown
  // proof.
  if (legNr >= NUM_OF_LEGS || !footSensor_mappingValid())
  {
    Serial.println(F("Foot sensor: release requested with invalid leg/mapping."));
    return false;
  }
  int pin = sensorPins[legNr];
  uint8_t bit = 1U << pin;
#if BUILDCFG_FOOT_SENSOR_DEBOUNCE_ENABLED
  return sampled && !(rawMask & bit) && !(stablePins & bit) &&
         uint32_t(millis() - candidateSince[pin]) >= FOOT_SENSOR_RELEASE_MS;
#else
  return sampled && !(rawMask & bit) && !(stablePins & bit);
#endif
}

void footSensor_setSensorMask(uint8_t mask)
{
  sensorMask = mask & ((1U << NUM_OF_LEGS) - 1);
}
