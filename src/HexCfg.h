/*
 * HexCfg.h
 *
 *  Created on: Feb 11, 2023
 *      Author: baotriet
 */

#ifndef HEXCFG_H_
#define HEXCFG_H_

#include "setup.h"

/****************************************************************
 * VARIABLE
 ****************************************************************/

#define c1DEC 10
#define c2DEC 100
#define c4DEC 10000
#define c6DEC 1000000

// PWM parameters
#define PWM_FREQ 50
#define MIN_PULSE 80
#define MAX_PULSE 512

#define L_DRIVER 0x41
#define R_DRIVER 0x40

#define L_Frequency 26450000
#define R_Frequency 26240000

#define SDA_PIN 13
#define SCL_PIN 14

#define RR 2
#define RM 1
#define RF 0

#define LR 5
#define LM 4
#define LF 3

#define COXA 0
#define FEMUR 1
#define TIBIA 2

#define CELL1 0
#define CELL2 1
#define BATT_VOLT_LOW 3.30
#define BATT_VOLT_HIGH 4.20
#define BATT_OFFSET_1 0.08
#define BATT_OFFSET_2 0.01
// Stability debounce for battery voltage readings: sanity checks will only
// run after readings stay within BATT_STABLE_DELTA for BATT_STABLE_COUNT samples.
// Tune these for your sensor noise and sampling rate.
#ifndef BATT_STABLE_DELTA
#define BATT_STABLE_DELTA 0.02
#endif
#ifndef BATT_STABLE_COUNT
#define BATT_STABLE_COUNT 3
#endif

#define STRING_INPUT_SIZE 50

#define NUM_OF_LEGS 6
#define NUM_LEGS_PER_SIDE 3
#define NUM_SERVOS_PER_LEG 3
#define NUM_OF_SERVOS 18
#define NUM_SERVOS_PER_PCA 9

#ifndef DEFAULT_GAIT_SPEED
#define DEFAULT_GAIT_SPEED 60
#define DEFAULT_SLOW_GAIT 70
#endif

#define SERVO_ANGLE_6V 60
#define SERVO_TIME_6V 0.08
#define SERVO_ANGLE_PER_SECOND SERVO_ANGLE_6V / SERVO_TIME_6V
#define SERVO_ANGLE_PER_MILISEC 1000 / SERVO_ANGLE_PER_SECOND

#define SmDiv 5    // 3 to 5
#define cGPlimit 2 // GP=GaitPos testing different limits
//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1 (600) // Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1 (0)    // Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1 (-600) // Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1 (600) // Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1 (0)    // Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1 (-600) // Default Coxa setup angle, decimals = 1

#define CHexInitXZ 35
#define CHexInitXZCos60 ((int32_t)(CHexInitXZ * 0.5))   // cos(60) = .5
#define CHexInitXZSin60 ((int32_t)(CHexInitXZ * 0.866)) // sin(60) = .866
#define CHexInitY                                                                                  \
  25 // Preserve the intentional folded home; standing presets add an offset above it.

#define cRRInitPosX CHexInitXZCos60 // Start positions of the Right Rear leg
#define cRRInitPosY CHexInitY
#define cRRInitPosZ -CHexInitXZSin60

#define cRMInitPosX CHexInitXZ // Start positions of the Right Middle leg
#define cRMInitPosY CHexInitY
#define cRMInitPosZ 0

#define cRFInitPosX CHexInitXZCos60 // Start positions of the Right Front leg
#define cRFInitPosY CHexInitY
#define cRFInitPosZ CHexInitXZSin60

#define cLRInitPosX CHexInitXZCos60 // Start positions of the Left Rear leg
#define cLRInitPosY CHexInitY
#define cLRInitPosZ -CHexInitXZSin60

#define cLMInitPosX CHexInitXZ // Start positions of the Left Middle leg
#define cLMInitPosY CHexInitY
#define cLMInitPosZ 0

#define cLFInitPosX CHexInitXZCos60 // Start positions of the Left Front leg
#define cLFInitPosY CHexInitY
#define cLFInitPosZ CHexInitXZSin60

#define cXXCoxaLength 25
#define cXXFemurLength 35
#define cXXTibiaLength 65

#define cRRCoxaLength cXXCoxaLength // Right Rear leg
#define cRRFemurLength cXXFemurLength
#define cRRTibiaLength cXXTibiaLength

#define cRMCoxaLength cXXCoxaLength // Right middle leg
#define cRMFemurLength cXXFemurLength
#define cRMTibiaLength cXXTibiaLength

#define cRFCoxaLength cXXCoxaLength // Right front leg
#define cRFFemurLength cXXFemurLength
#define cRFTibiaLength cXXTibiaLength

#define cLRCoxaLength cXXCoxaLength // Left Rear leg
#define cLRFemurLength cXXFemurLength
#define cLRTibiaLength cXXTibiaLength

#define cLMCoxaLength cXXCoxaLength // Left middle leg
#define cLMFemurLength cXXFemurLength
#define cLMTibiaLength cXXTibiaLength

#define cLFCoxaLength cXXCoxaLength // Left front leg
#define cLFFemurLength cXXFemurLength
#define cLFTibiaLength cXXTibiaLength

#define HALF_BODY_LENGHT 70
#define HALF_BODY_MIDDLE_WIDTH 75
#define HALF_BODY_FR_WIDTH 35

#define cRROffsetX -HALF_BODY_FR_WIDTH // Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ -HALF_BODY_LENGHT   // Distance Z from center of the body to the Right Rear coxa

#define cRMOffsetX                                                                                 \
  -HALF_BODY_MIDDLE_WIDTH // Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ 0      // Distance Z from center of the body to the Right Middle coxa

#define cRFOffsetX -HALF_BODY_FR_WIDTH // Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ HALF_BODY_LENGHT    // Distance Z from center of the body to the Right Front coxa

#define cLROffsetX HALF_BODY_FR_WIDTH // Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ -HALF_BODY_LENGHT  // Distance Z from center of the body to the Left Rear coxa

#define cLMOffsetX                                                                                 \
  HALF_BODY_MIDDLE_WIDTH // Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ 0     // Distance Z from center of the body to the Left Middle coxa

#define cLFOffsetX HALF_BODY_FR_WIDTH // Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ HALF_BODY_LENGHT   // Distance Z from center of the body to the Left Front coxa

#define COXA_MinANG -900
#define COXA_MaxANG 900
#define FEMUR_MinANG -900
#define FEMUR_MaxANG 900
#define TIBIA_MinANG -900
#define TIBIA_MaxANG 900
//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1 COXA_MinANG // Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax1 COXA_MaxANG
#define cRRFemurMin1 FEMUR_MinANG
#define cRRFemurMax1 FEMUR_MaxANG
#define cRRTibiaMin1 TIBIA_MinANG
#define cRRTibiaMax1 TIBIA_MaxANG

#define cRMCoxaMin1 COXA_MinANG // Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1 COXA_MaxANG
#define cRMFemurMin1 FEMUR_MinANG
#define cRMFemurMax1 FEMUR_MaxANG
#define cRMTibiaMin1 TIBIA_MinANG
#define cRMTibiaMax1 TIBIA_MaxANG

#define cRFCoxaMin1 COXA_MinANG // Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1 COXA_MaxANG
#define cRFFemurMin1 FEMUR_MinANG
#define cRFFemurMax1 FEMUR_MaxANG
#define cRFTibiaMin1 TIBIA_MinANG
#define cRFTibiaMax1 TIBIA_MaxANG

#define cLRCoxaMin1 COXA_MinANG // Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1 COXA_MaxANG
#define cLRFemurMin1 FEMUR_MinANG
#define cLRFemurMax1 FEMUR_MaxANG
#define cLRTibiaMin1 TIBIA_MinANG
#define cLRTibiaMax1 TIBIA_MaxANG

#define cLMCoxaMin1 COXA_MinANG // Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1 COXA_MaxANG
#define cLMFemurMin1 FEMUR_MinANG
#define cLMFemurMax1 FEMUR_MaxANG
#define cLMTibiaMin1 TIBIA_MinANG
#define cLMTibiaMax1 TIBIA_MaxANG

#define cLFCoxaMin1 COXA_MinANG // Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1 COXA_MaxANG
#define cLFFemurMin1 FEMUR_MinANG
#define cLFFemurMax1 FEMUR_MaxANG
#define cLFTibiaMin1 TIBIA_MinANG
#define cLFTibiaMax1 TIBIA_MaxANG

#define DEFINE_HEX_GLOBALS

// Height presets and motion limits.
#if 1
#define CNT_HEX_INITS 4
// Operator-selected lifts: keep creep/fallback below normal lift; these are not validated
// envelopes.
#define CREEP_BODY_Y_MM 45
#define CREEP_LEG_LIFT_MM 20
#define FALLBACK_LEG_LIFT_MM 20
#define CREEP_TRAVEL_MM 16
// Recovery chooses the highest safe 20..8 mm clearance, then bounded placement/slide probes.
#define FOOT_RECOVERY_PLACE_MM 8
#define FOOT_RECOVERY_CLEARANCE_MM 20
#define FOOT_RECOVERY_CLEARANCE_STEP_DOWN_MM 2
#define FOOT_RECOVERY_CLEARANCE_MIN_MM 8
// Separate 5 mm servo moves tuck at most 15 mm before sliding; stop at the IK margin.
#define FOOT_RECOVERY_TUCK_STEP_MM 5
#define FOOT_RECOVERY_TUCK_MAX_MM 15
#define FOOT_RECOVERY_RETREAT_MM 8
// Runtime IK (5 mm / 5 degree margin) and support polygon, not this ceiling, limit shifts.
// Approximate radial room at Y=45/56/65/85 is 14/31/34/13 mm respectively.
#define FOOT_RECOVERY_SHIFT_STEP_MM 6
#define FOOT_RECOVERY_SHIFT_MAX_MM 30
#define FOOT_RECOVERY_WALK_STEPS 2
#define FOOT_RECOVERY_WALK_STEP_MM 15
#define CREEP_STEP_MS 250
#define DEFAULT_LEG_LIFT_MM 30
// Independent high value avoids doubling the new normal lift into the previously rejected 60 mm.
#define HIGH_LEG_LIFT_MM 40
// Operator-selected browser +/-5 lift range; 50 mm is not a verified reach envelope at low heights.
#define MIN_LEG_LIFT_MM 25
#define MAX_LEG_LIFT_MM 50
#define LEG_LIFT_STEP_MM 5
// Finer browser body step lets the operator tune height between the four presets.
#define BODY_HEIGHT_STEP_MM 5
#define MAX_BODY_Y 85
// Body commands store an offset, while the user-facing presets are absolute foot Y coordinates.
#define MAX_BODY_Y_OFFSET (MAX_BODY_Y - CHexInitY)

#ifdef DEFINE_HEX_GLOBALS
// Vertical tibia at neutral standing: XZ = coxa + sqrt(femur^2 - (height - tibia)^2).
// Rounded millimeter widths apply only at the paired presets on level ground, not during gait/body
// motion.
const byte g_abHexIntXZ[] PROGMEM = {54, 59, 60, 54};
// Absolute coxa-relative foot Y at neutral body rotation; folded Y is separate.
const byte g_abHexMaxBodyY[] PROGMEM = {CREEP_BODY_Y_MM, 56, 65, MAX_BODY_Y};
#else
extern const byte g_abHexIntXZ[];
extern const byte g_abHexMaxBodyY[];
#endif
#endif // Height presets and motion limits.

#define CONTROL_MODE 4
#define GAIT_TYPE 6
enum
{
  MODE_WALK = 0,
  MODE_TRANSLATE,
  MODE_ROTATE,
  MODE_SINGLE_LEG
};

//                          RF RM RR LF LM LR
// Left: CW +   CCW -        Right: CW + CCW -
const short CoxaAngleOffset[NUM_OF_LEGS] PROGMEM = {-50, -50, -50, 50, 0, -50};
// Left: Up   Down      Right: Up  Down (L==R)
const short FemurAngleOffset[NUM_OF_LEGS] PROGMEM = {0, -50, -50, 0, 0, 0};
// Left: Up +  Down -       Right: Up + Down -
const short TibiaAngleOffset[NUM_OF_LEGS] PROGMEM = {-50, 0, -80, 0, -50, 0};

#endif /* HEXCFG_H_ */
