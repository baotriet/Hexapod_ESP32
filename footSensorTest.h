#ifndef FOOT_SENSOR_TEST_H_
#define FOOT_SENSOR_TEST_H_

#include "setup.h"
#include "footSensorCfg.h"

#ifdef FOOT_SENSOR_TEST_ENABLED
typedef bool (*FootTestMove)(byte leg, short y);
void footSensorTest_init(FootTestMove move);
void footSensorTest_update();
#endif

#endif
