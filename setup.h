#ifndef SETUP_H_ /* Include guard */
#define SETUP_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// #define DEBUG
/**
 * @brief Enable Serial-based servo calibration instead of normal robot operation.
 */
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

#ifndef BUILDCFG_MOTION_TIMING_ENABLED
#define BUILDCFG_MOTION_TIMING_ENABLED 0
#endif

// Ordinary WALK only; set to 0 for the original stop-at-each-waypoint trajectory.
#ifndef BUILDCFG_WALK_CONTINUITY_ENABLED
#define BUILDCFG_WALK_CONTINUITY_ENABLED 1
#endif

/**
 * @brief Enable Wi-Fi HTTP services for robot control and browser firmware updates.
 */
#define HTTP_ENABLED
/**
 * @brief Enable the browser controller with command ownership and a connection watchdog.
 * @details Requires HTTP_ENABLED.
 */
#define WEB_CONTROLLER_ENABLED
/**
 * @brief Enable the legacy app command endpoint alongside the browser controller.
 * @details Legacy input shares the controller ownership lease.
 */
// #define LEGACY_HTTP_COMMAND_ENABLED
/**
 * @brief Enable camera streaming and select the AI Thinker camera pin layout.
 */
#define CAMERA_ENABLED
/**
 * @brief Enable MPU6050-based body orientation correction.
 */
#define MPU_ENABLED
/**
 * @brief Enable foot-contact sensing; runtime walking protection is separate.
 */
#define FOOT_SENSOR_ENABLED
/**
 * @brief Compile contact-aware walking protection.
 * @details Requires FOOT_SENSOR_ENABLED. Protection must be enabled at runtime;
 * it is not automatically activated at boot.
 */
#define FOOT_SENSOR_WALK_ENABLED
/**
 * @brief Apply software debounce to foot-contact readings.
 * @details Set to 1 to require the configured stable contact/release interval.
 * Default OFF (0) assumes the sensor hardware already debounces its output.
 */
#ifndef BUILDCFG_FOOT_SENSOR_DEBOUNCE_ENABLED
#define BUILDCFG_FOOT_SENSOR_DEBOUNCE_ENABLED 0
#endif
/**
 * @brief Allow automatic slow-wave fallback when the selected gait cannot safely lift.
 * @details Set to 1 to enable, or 0 to wait for support without changing gait.
 * Does not bypass support checks or disable the separate missing-ground recovery retreat.
 */
#ifndef BUILDCFG_SLOW_WAVE_FALLBACK_ENABLED
#define BUILDCFG_SLOW_WAVE_FALLBACK_ENABLED 0
#endif
/**
 * @brief Allow bounded walk-away escalation after recovery body-shift attempts fail.
 * @details Set to 1 for at most two support-foot re-center / 15 mm shift cycles.
 * Default ON (1): the lost foot stays lifted at 20 mm through each cycle and probes once
 * afterward. Set to 0 to retain clearance/body-shift recovery without walk-away cycles.
 */
#ifndef BUILDCFG_RECOVERY_WALK_AWAY_ENABLED
#define BUILDCFG_RECOVERY_WALK_AWAY_ENABLED 1
#endif
/**
 * @brief Enable geometry-guided recovery when a foot cannot find ground.
 * @details Set to 1 for bounded clearance/placement attempts and cautious retreat,
 * without recorded paths or previous travel direction. Set to 0 for missing-ground HOLD.
 * Mechanical and power protections remain enabled.
 */
#ifndef FOOT_SENSOR_RETURN_ON_FAILURE
#define FOOT_SENSOR_RETURN_ON_FAILURE 1
#endif
/**
 * @brief Replace normal operation with the isolated, Serial-controlled foot test.
 * @details Normal networking and all-leg startup positioning are skipped.
 */
// #define FOOT_SENSOR_TEST_ENABLED
/**
 * @brief Enable interrupt-assisted foot-sensor updates instead of polling alone.
 */
// #define FOOT_SENSOR_INT_ENABLED
/**
 * @brief Reverse the configured input-axis convention before command handling.
 */
#define REVERSE_CMD_ENABLED
/**
 * @brief Reserved switch for a rotated MPU6050 mounting.
 * @details Currently unused by the implementation; defining it has no effect.
 */
// #define ROTATE_180_MPU6050

#if defined(WEB_CONTROLLER_ENABLED) && !defined(HTTP_ENABLED)
#error "WEB_CONTROLLER_ENABLED requires HTTP_ENABLED"
#endif
#ifdef CAMERA_ENABLED
/**
 * @brief Select the AI Thinker ESP32-CAM camera and LED GPIO definitions.
 */
#define CAMERA_MODEL_AI_THINKER
#endif // CAMERA_ENABLED

/**
 * @brief Suppress the three buzzer signals after normal initialization.
 * @details Define to suppress, or leave undefined to sound the buzzer.
 * The three red LED initialization blinks are unaffected.
 */
#ifndef BUILDCFG_BUZZER_AFTER_INIT_ENABLED
#define BUILDCFG_BUZZER_AFTER_INIT_ENABLED 0
#endif

#endif // SETUP_H_