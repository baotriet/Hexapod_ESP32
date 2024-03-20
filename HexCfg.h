/*
 * HexCfg.h
 *
 *  Created on: Feb 11, 2023
 *      Author: baotriet
 */

#ifndef HEXCFG_H_
#define HEXCFG_H_

#define LED_RED GPIO_PIN_1
#define LED_GREEN GPIO_PIN_2
#define LED_BLUE GPIO_PIN_3

/****************************************************************
 * VARIABLE
 ****************************************************************/

#define c1DEC 10
#define c2DEC 100
#define c4DEC 10000
#define c6DEC 1000000

// PWM parameters
#define PWM_FREQ 50
#define MIN_PULSE 210
#define MAX_PULSE 800

#define L_DRIVER 0x41
#define R_DRIVER 0x40


#define RRearCoxaServo 0
#define RRearFemurServo 1
#define RRearTibiaServo 2

#define RCenterCoxaServo 3
#define RCenterFemurServo 4
#define RCenterTibiaServo 5

#define RFrontCoxaServo 6
#define RFrontFemurServo 7
#define RFrontTibiaServo 8

#define LRearCoxaServo 9
#define LRearFemurServo 10
#define LRearTibiaServo 11

#define LCenterCoxaServo 12
#define LCenterFemurServo 13
#define LCenterTibiaServo 14

#define LFrontCoxaServo 15
#define LFrontFemurServo 16
#define LFrontTibiaServo 17

#define RR 0
#define RM 1
#define RF 2

#define LR 3
#define LM 4
#define LF 5

#define NUMBER_OF_LEGS 6
#define STRING_INPUT_SIZE 50

#ifndef DEFAULT_GAIT_SPEED
#define DEFAULT_GAIT_SPEED 60
#define DEFAULT_SLOW_GAIT 70
#endif

#define SmDiv 5     // 3 to 5
#define cGPlimit 2  // GP=GaitPos testing different limits
//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1 (-600)  //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1 (0)     //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1 (600)   //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1 (-600)  //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1 (0)     //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1 (600)   //Default Coxa setup angle, decimals = 1

#define CHexInitXZ 65
#define CHexInitXZCos60 ((int32_t)(CHexInitXZ * 0.5))    // cos(60) = .5
#define CHexInitXZSin60 ((int32_t)(CHexInitXZ * 0.866))  // sin(60) = .866
#define CHexInitY 50                                     //Typical standing height (body above ground) in [mm]

#define cRRInitPosX CHexInitXZCos60  //Start positions of the Right Rear leg
#define cRRInitPosY CHexInitY
#define cRRInitPosZ CHexInitXZSin60

#define cRMInitPosX CHexInitXZ  //Start positions of the Right Middle leg
#define cRMInitPosY CHexInitY
#define cRMInitPosZ 0

#define cRFInitPosX CHexInitXZCos60  //Start positions of the Right Front leg
#define cRFInitPosY CHexInitY
#define cRFInitPosZ -CHexInitXZSin60

#define cLRInitPosX CHexInitXZCos60  //Start positions of the Left Rear leg
#define cLRInitPosY CHexInitY
#define cLRInitPosZ CHexInitXZSin60

#define cLMInitPosX CHexInitXZ  //Start positions of the Left Middle leg
#define cLMInitPosY CHexInitY
#define cLMInitPosZ 0

#define cLFInitPosX CHexInitXZCos60  //Start positions of the Left Front leg
#define cLFInitPosY CHexInitY
#define cLFInitPosZ -CHexInitXZSin60

#define cXXCoxaLength 28 
#define cXXFemurLength 50
#define cXXTibiaLength 80

#define cRRCoxaLength cXXCoxaLength  //Right Rear leg
#define cRRFemurLength cXXFemurLength
#define cRRTibiaLength cXXTibiaLength

#define cRMCoxaLength cXXCoxaLength  //Right middle leg
#define cRMFemurLength cXXFemurLength
#define cRMTibiaLength cXXTibiaLength

#define cRFCoxaLength cXXCoxaLength  //Right front leg
#define cRFFemurLength cXXFemurLength
#define cRFTibiaLength cXXTibiaLength

#define cLRCoxaLength cXXCoxaLength  //Left Rear leg
#define cLRFemurLength cXXFemurLength
#define cLRTibiaLength cXXTibiaLength

#define cLMCoxaLength cXXCoxaLength  //Left middle leg
#define cLMFemurLength cXXFemurLength
#define cLMTibiaLength cXXTibiaLength

#define cLFCoxaLength cXXCoxaLength  //Left front leg
#define cLFFemurLength cXXFemurLength
#define cLFTibiaLength cXXTibiaLength

#define HALF_BODY_LENGHT 73
#define HALF_BODY_MIDDLE_WIDTH 66
#define HALF_BODY_FR_WIDTH 55

#define cRROffsetX -HALF_BODY_FR_WIDTH  //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ HALF_BODY_LENGHT     //Distance Z from center of the body to the Right Rear coxa

#define cRMOffsetX -HALF_BODY_MIDDLE_WIDTH  //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ 0                        //Distance Z from center of the body to the Right Middle coxa

#define cRFOffsetX -HALF_BODY_FR_WIDTH  //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ -HALF_BODY_LENGHT    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX HALF_BODY_FR_WIDTH  //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ HALF_BODY_LENGHT    //Distance Z from center of the body to the Left Rear coxa

#define cLMOffsetX HALF_BODY_MIDDLE_WIDTH  //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ 0                       //Distance Z from center of the body to the Left Middle coxa

#define cLFOffsetX HALF_BODY_FR_WIDTH  //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ -HALF_BODY_LENGHT   //Distance Z from center of the body to the Left Front coxa

#define COXA_MinANG -800
#define COXA_MaxANG 800
#define FEMUR_MinANG -800
#define FEMUR_MaxANG 800
#define TIBIA_MinANG -800
#define TIBIA_MaxANG 800
//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1 COXA_MinANG  //Mechanical limits of the Right Rear Leg, decimals = 1
#define cRRCoxaMax1 COXA_MaxANG
#define cRRFemurMin1 FEMUR_MinANG
#define cRRFemurMax1 FEMUR_MaxANG
#define cRRTibiaMin1 TIBIA_MinANG
#define cRRTibiaMax1 TIBIA_MaxANG

#define cRMCoxaMin1 COXA_MinANG  //Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1 COXA_MaxANG
#define cRMFemurMin1 FEMUR_MinANG
#define cRMFemurMax1 FEMUR_MaxANG
#define cRMTibiaMin1 TIBIA_MinANG
#define cRMTibiaMax1 TIBIA_MaxANG

#define cRFCoxaMin1 COXA_MinANG  //Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1 COXA_MaxANG
#define cRFFemurMin1 FEMUR_MinANG
#define cRFFemurMax1 FEMUR_MaxANG
#define cRFTibiaMin1 TIBIA_MinANG
#define cRFTibiaMax1 TIBIA_MaxANG

#define cLRCoxaMin1 COXA_MinANG  //Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1 COXA_MaxANG
#define cLRFemurMin1 FEMUR_MinANG
#define cLRFemurMax1 FEMUR_MaxANG
#define cLRTibiaMin1 TIBIA_MinANG
#define cLRTibiaMax1 TIBIA_MaxANG

#define cLMCoxaMin1 COXA_MinANG  //Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1 COXA_MaxANG
#define cLMFemurMin1 FEMUR_MinANG
#define cLMFemurMax1 FEMUR_MaxANG
#define cLMTibiaMin1 TIBIA_MinANG
#define cLMTibiaMax1 TIBIA_MaxANG

#define cLFCoxaMin1 COXA_MinANG  //Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1 COXA_MaxANG
#define cLFFemurMin1 FEMUR_MinANG
#define cLFFemurMax1 FEMUR_MaxANG
#define cLFTibiaMin1 TIBIA_MinANG
#define cLFTibiaMax1 TIBIA_MaxANG

#define DEFINE_HEX_GLOBALS

// Lets try some multi leg positions depending on height settings.
#if 1  // Start first without...
#define CNT_HEX_INITS 3
#define MAX_BODY_Y 115

#ifdef DEFINE_HEX_GLOBALS
const byte g_abHexIntXZ[] PROGMEM = { CHexInitXZ, 80, 65 };
const byte g_abHexMaxBodyY[] PROGMEM = { CHexInitY, 70, MAX_BODY_Y };
#else
extern const byte g_abHexIntXZ[];
extern const byte g_abHexMaxBodyY[];
#endif
#endif  // if 0

#define CONTROL_MODE 4
#define GAIT_TYPE 6
enum {
  MODE_WALK = 0,
  MODE_TRANSLATE,
  MODE_ROTATE,
  MODE_SINGLE_LEG
};

// int offsetAngle[18] = {-50, -50, -100,
//                        -50, -100, 0,
//                        100, -100, 0,
//                        -100, 50, 150,
//                        -100, -100, -100,
//                        -200, -150, -200};


//                          RR RM RF LR LM LF
// Left: Up   Down       Right: Up - Down +
const word CoxaAngleOffset[NUMBER_OF_LEGS] PROGMEM = { 100, -100, 0, -100, -100, 0 }; //reverse left
// Left: Up   Down       Right: Up - Down +
const word FemurAngleOffset[NUMBER_OF_LEGS] PROGMEM = { 0, 0, 100, 0, 0, 0 }; //reverse right
// Left: Up +  Down -       Right: Up + Down -
const word TibiaAngleOffset[NUMBER_OF_LEGS] PROGMEM = { 0, 0, 200, 70, -40, 100 }; //reverse left

#endif /* HEXCFG_H_ */
