#ifndef FOOT_LANDING_H_
#define FOOT_LANDING_H_

#include "setup.h"
#include "HexCfg.h"

void footLanding_init();
void footLanding_plan(const long previousY[], const long plannedY[], uint32_t duration);
void footLanding_update();
bool footLanding_busy();
bool footLanding_fault();
bool footLanding_engaged();
long footLanding_y(byte leg, long gaitY);
bool footLanding_direct(byte leg);
bool footLanding_retry();
void footLanding_fail(const char *reason, byte leg);
bool footLanding_timedOut();
bool footLanding_allSupported();
bool footLanding_beginRestore();
bool footLanding_restoring();
bool footLanding_restored();
void footLanding_record(const long x[], const long y[], const long z[], const long rotation[]);
void footLanding_restoreTargets(long x[], long y[], long z[], long rotation[]);

#endif
