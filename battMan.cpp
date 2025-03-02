#include "driver/ads1115.h"
#include "driver/buzzer.h"
#include "HexCfg.h"

static bool isSanityCheckOk(float cell1, float cell2);

bool battMan_isPowerOk()
{
    bool retVal = true;
    float cell1, cell2;
    cell1 = ads1115_readVolt(CELL1) + BATT_OFFSET_1;
    cell2 = ads1115_readVolt(CELL2) + BATT_OFFSET_2;
    if (!isSanityCheckOk(cell1, cell2))
    {
        buzzer_on();
        retVal = false;
    }
    return retVal;
}

static bool isSanityCheckOk(float cell1, float cell2)
{
    bool retVal = true;
    // low voltage
    if (cell1 < BATT_VOLT_LOW || cell2 < BATT_VOLT_LOW)
    {
        retVal = false;
    }
    // unbalanced cell
    if (abs(cell1 - cell2) >= 0.1)
    {
        retVal = false;
    }
    return retVal;
}