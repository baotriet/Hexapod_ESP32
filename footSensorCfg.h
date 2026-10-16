#ifndef FOOT_SENSOR_CFG_H_
#define FOOT_SENSOR_CFG_H_

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

#define FOOT_SENSOR_SAMPLE_MS 5
#define FOOT_SENSOR_CONTACT_MS 15
#define FOOT_SENSOR_RELEASE_MS 15
#define FOOT_SENSOR_REPORT_MS 500

#define FOOT_WALK_EXTRA_DOWN_MM 5
#define FOOT_WALK_SEARCH_MM_S 1
#define FOOT_WALK_TIMEOUT_MS 3000

#define FOOT_TEST_LEG RF
// Leave zero until measured. Start Y is a coxa-relative coordinate, positive down.
#ifndef FOOT_TEST_START_Y_MM
#define FOOT_TEST_START_Y_MM 35
#endif
// Expected surface coordinate for this isolated test, independent of walking home.
#ifndef FOOT_TEST_GROUND_Y_MM
#define FOOT_TEST_GROUND_Y_MM FOOT_TEST_START_Y_MM
#endif
#ifndef FOOT_TEST_EXTRA_DOWN_MM
#define FOOT_TEST_EXTRA_DOWN_MM 5
#endif
#ifndef FOOT_TEST_SPEED_MM_S
#define FOOT_TEST_SPEED_MM_S 1
#endif
#ifndef FOOT_TEST_TIMEOUT_MS
#define FOOT_TEST_TIMEOUT_MS 10000
#endif

#endif
