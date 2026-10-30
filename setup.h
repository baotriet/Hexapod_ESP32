#ifndef SETUP_H_ /* Include guard */
#define SETUP_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// #define DEBUG
// #define CALIBRATION_ENABLED
// #define DEBUG_PID
// #define DEBUG_LEG
// #define DEBUG_ON
// #define DEBUG_MPU
// #define DEBUG_PWM
// #define DEBUG_INPUT_DATA
// #define DEBUG_HTTP_DATA
// #define DEBUG_CAMERA
// #define DEBUG_WIFI
// #define DEBUG_BATTERY
// #define DEBUG_FOOT_SENSOR

#define HTTP_ENABLED
#define WEB_CONTROLLER_ENABLED
// Preserve the old app endpoint only when explicitly requested; it shares the browser ownership
// lease. #define LEGACY_HTTP_COMMAND_ENABLED
#define CAMERA_ENABLED
#define MPU_ENABLED
#define FOOT_SENSOR_ENABLED
// Compile walking protection separately from runtime activation; boot never enables it
// automatically.
#define FOOT_SENSOR_WALK_ENABLED
// Return along the recorded path on landing timeout/unreachable landing; then reverse three cycles.
// Set to 0 to retain the original landing HOLD policy. Mechanical/power protections stay enabled.
#ifndef FOOT_SENSOR_RETURN_ON_FAILURE
#define FOOT_SENSOR_RETURN_ON_FAILURE 1
#endif
// Keep the isolated supported-leg test here so all firmware mode switches have one entry point.
// #define FOOT_SENSOR_TEST_ENABLED
// #define FOOT_SENSOR_INT_ENABLED
#define REVERSE_CMD_ENABLED
// #define ROTATE_180_MPU6050

#if defined(WEB_CONTROLLER_ENABLED) && !defined(HTTP_ENABLED)
#error "WEB_CONTROLLER_ENABLED requires HTTP_ENABLED"
#endif
#ifdef CAMERA_ENABLED
#define CAMERA_MODEL_AI_THINKER
#endif // CAMERA_ENABLED

#endif // SETUP_H_