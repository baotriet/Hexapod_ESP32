#include "driver/ads1115.h"
#include "driver/buzzer.h"
#include "HexCfg.h"
#include "battMan.h"
#include "http.h"
#include "mqtt.h"
#include "bluetooth.h"
#include <ArduinoJson.h>

static bool isSanityCheckOk(float cell1, float cell2);

// Maintain short rolling history to detect stable voltage readings. We only
// perform the sanity check when both cells' recent readings are within the
// configured delta (BATT_STABLE_DELTA) across BATT_STABLE_COUNT samples.
bool battMan_isPowerOk()
{
    bool retVal = true;
    float cell1 = ads1115_readVolt(CELL1) + BATT_OFFSET_1;
    float cell2 = ads1115_readVolt(CELL2) + BATT_OFFSET_2;

#ifdef DEBUG_BATTERY
    Serial.print("Cell_1: ");
    Serial.println(cell1);
    Serial.print("Cell_2: ");
    Serial.println(cell2);
#endif

    // static buffers hold the last BATT_STABLE_COUNT readings
    static float buf1[ BATT_STABLE_COUNT ];
    static float buf2[ BATT_STABLE_COUNT ];
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
        if (buf1[i] < min1) min1 = buf1[i];
        if (buf1[i] > max1) max1 = buf1[i];
        if (buf2[i] < min2) min2 = buf2[i];
        if (buf2[i] > max2) max2 = buf2[i];
    }

    float delta1 = max1 - min1;
    float delta2 = max2 - min2;

#ifdef DEBUG_BATTERY
    Serial.print("Delta1: "); Serial.println(delta1);
    Serial.print("Delta2: "); Serial.println(delta2);
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
    if (cell1)
        *cell1 = ads1115_readVolt(CELL1) + BATT_OFFSET_1;
    if (cell2)
        *cell2 = ads1115_readVolt(CELL2) + BATT_OFFSET_2;
}

bool battMan_sendVoltages(BattTransport transport, const char *endpoint)
{
    float v1;
    float v2;
    bool result = false;
    if (transport != BATT_TRANSPORT_HTTP)
    {
        float v1 = ads1115_readVolt(CELL1) + BATT_OFFSET_1;
        float v2 = ads1115_readVolt(CELL2) + BATT_OFFSET_2;
    }
    
    switch (transport)
    {
    case BATT_TRANSPORT_HTTP:
    {
        break;
    }
    case BATT_TRANSPORT_MQTT:
    {
        if (endpoint == NULL)
        {
            result = false;
            break;
        }
        result = mqtt_publishVoltages(endpoint, v1, v2);
        break;
    }
    case BATT_TRANSPORT_BLUETOOTH:
    {
        result = bluetooth_sendVoltages(v1, v2);
        break;
    }
    default:
    {
        result = false;
        break;
    }
    }

    return result;
}