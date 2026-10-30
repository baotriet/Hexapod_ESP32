#ifndef FOOT_LANDING_H_
#define FOOT_LANDING_H_

#include "setup.h"
#include "HexCfg.h"

void footLanding_init();
void footLanding_resume(const long plannedY[], const long offsets[]);
void footLanding_plan(const long previousY[], const long plannedY[], uint32_t duration,
                      uint32_t firstFrameMs = 0);
void footLanding_update();
bool footLanding_busy();
bool footLanding_fault();
const char *footLanding_reason();
bool footLanding_engaged();
long footLanding_y(byte leg, long gaitY);
long footLanding_horizontal(byte leg, long previous, long planned);
bool footLanding_direct(byte leg);
bool footLanding_retry();
// Sensor exit may waive contact checks, but must never turn an invalid pose or low power into
// motion.
enum FootFault
{
  FOOT_FAULT_SENSOR,
  FOOT_FAULT_LANDING,
  FOOT_FAULT_GEOMETRY,
  FOOT_FAULT_POWER
};
void footLanding_fail(const char *reason, byte leg, FootFault kind = FOOT_FAULT_SENSOR);
bool footLanding_hardFault();
bool footLanding_returnPending();
void footLanding_unreachable(byte leg);
void footLanding_supportGeometry(const long x[], const long z[]);
bool footLanding_supportReady(byte exclude = 0);
bool footLanding_liftReady(const long nextY[]);
bool footLanding_timedOut();
// The missing-ground timeout came from a support foot losing ground, not a landing foot.
bool footLanding_supportLost();
// Support feet that lost ground and wait for their next swing (bit per leg).
byte footLanding_lostMask();
// Stop a failed re-place search for this leg and mark it lost (raised to its landing height).
void footLanding_abandon(byte leg);
bool footLanding_allSupported();
bool footLanding_beginRestore();
bool footLanding_restoring();
bool footLanding_restored();
void footLanding_record(const long x[], const long y[], const long z[], const long rotation[]);
void footLanding_restoreTargets(long x[], long y[], long z[], long rotation[]);
// Joint-limit pause: undo a rejected frame and freeze landing timers while no landing checks run.
void footLanding_save();
void footLanding_rollback();
void footLanding_delay(uint32_t ms);
// Extra depth beyond the planned landing target while searching or seating (0 otherwise).
long footLanding_searchDepth(byte leg);
// Treat a search that left the leg's reach like the search timeout: missing ground, not geometry.
void footLanding_noGround(byte leg);

#endif
