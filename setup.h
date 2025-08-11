#ifndef SETUP_H_   /* Include guard */
#define SETUP_H_

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define DEBUG
// #define DEBUG_PID
// #define DEBUG_LEG
// #define DEBUG_ON
// #define DEBUG_WIFI_MQTT
// #define DEBUG_BLUETOOTH
// #define DEBUG_MPU
// #define DEBUG_PWM
// #define DEBUG_INPUT_DATA
// #define DEBUG_HTTP_DATA
// #define DEBUG_CAMERA
// #define DEBUG_WIFI
// #define DEBUG_BATTERY

// #define BLUETOOTH_ENABLED
// #define MQTT_ENABLED
#define HTTP_ENABLED
#define CAMERA_ENABLED
#define MPU_ENABLED
// #define FOOT_SENSOR_ENABLED
// #define FOOT_SENSOR_INT_ENABLED
#define REVERSE_CMD_ENABLED
// #define ROTATE_180_MPU6050

#ifdef CAMERA_ENABLED
#define CAMERA_MODEL_AI_THINKER
#endif // CAMERA_ENABLED

#endif // SETUP_H_