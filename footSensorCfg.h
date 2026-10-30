#ifndef FOOT_SENSOR_CFG_H_
#define FOOT_SENSOR_CFG_H_

// Keep wiring separate from leg order so a physical switch stops the correct leg.
// PCF8575 bit numbers: P00=0 ... P05=5. -1 means not mapped yet.
#ifndef FOOT_SENSOR_RF_PIN
#define FOOT_SENSOR_RF_PIN 3
#endif
#ifndef FOOT_SENSOR_RM_PIN
#define FOOT_SENSOR_RM_PIN 4
#endif
#ifndef FOOT_SENSOR_RR_PIN
#define FOOT_SENSOR_RR_PIN 5
#endif
#ifndef FOOT_SENSOR_LF_PIN
#define FOOT_SENSOR_LF_PIN 2
#endif
#ifndef FOOT_SENSOR_LM_PIN
#define FOOT_SENSOR_LM_PIN 1
#endif
#ifndef FOOT_SENSOR_LR_PIN
#define FOOT_SENSOR_LR_PIN 0
#endif

// Debounce contact and release separately so switch bounce cannot fake touchdown or lift-off.
#define FOOT_SENSOR_SAMPLE_MS 5
#define FOOT_SENSOR_CONTACT_MS 15
#define FOOT_SENSOR_RELEASE_MS 15
// Limit optional raw diagnostics so Serial output does not dominate the motion loop.
#define FOOT_SENSOR_REPORT_MS 500

// Bound downward travel, rate, and time independently; a missing surface must not cause endless
// descent.
#define FOOT_WALK_EXTRA_DOWN_MM 10
#define FOOT_WALK_SEARCH_MM_S 10
#define FOOT_WALK_TIMEOUT_MS 2000
// A lightly touching foot may need switch travel; bound seating independently and stop on stable
// contact.
#define FOOT_WALK_SEAT_MM 2
#define FOOT_WALK_SEAT_MM_S 5
// Tolerate brief support-switch releases before searching or latching insufficient-support HOLD.
#define FOOT_WALK_SUPPORT_GRACE_MS 200
// Reject support polygons whose edge is too close to the estimated body-center projection.
#define FOOT_WALK_SUPPORT_MARGIN_MM 2
// Give the one-leg fallback more time to place each foot than the normal fast gaits.
#define FOOT_WALK_FALLBACK_STEP_MS 400

// Isolate one leg so wiring and touchdown can be checked before enabling six-leg walking.
#define FOOT_TEST_LEG RF
// Use a measured reachable start, not the deliberately folded walking home pose.
// Start Y is a coxa-relative coordinate, positive down; zero locks the test until configured.
#ifndef FOOT_TEST_START_Y_MM
#define FOOT_TEST_START_Y_MM 35
#endif
// A separate ground reference lets the supported-body test run independently of standing height.
#ifndef FOOT_TEST_GROUND_Y_MM
#define FOOT_TEST_GROUND_Y_MM FOOT_TEST_START_Y_MM
#endif
#ifndef FOOT_TEST_EXTRA_DOWN_MM
// Keep a failed switch or absent surface from driving the test leg downward indefinitely.
#define FOOT_TEST_EXTRA_DOWN_MM 5
#endif
#ifndef FOOT_TEST_SPEED_MM_S
#define FOOT_TEST_SPEED_MM_S 1
#endif
#ifndef FOOT_TEST_TIMEOUT_MS
#define FOOT_TEST_TIMEOUT_MS 10000
#endif

#endif
