/****************************************************************
 * IMPORT
 ****************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "PWMServoDriver.h"
#include "Hex_cfg.h"
#include "Hex_input.h"
#include "bluetooth.h"
#include "pid.h"
#include "setup.h"

//#define FOOT_SENSOR

#ifndef BALANCE_DELAY
#define BALANCE_DELAY 100
#endif

int tempServoAngle[19] = { 0 };

byte reData[7];
boolean dataReady;
//Min / imax values
const short cCoxaMin1[] PROGMEM = {
  cRRCoxaMin1, cRMCoxaMin1, cRFCoxaMin1, cLRCoxaMin1, cLMCoxaMin1, cLFCoxaMin1
};
const short cCoxaMax1[] PROGMEM = {
  cRRCoxaMax1, cRMCoxaMax1, cRFCoxaMax1, cLRCoxaMax1, cLMCoxaMax1, cLFCoxaMax1
};
const short cFemurMin1[] PROGMEM = {
  cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1
};
const short cFemurMax1[] PROGMEM = {
  cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1
};
const short cTibiaMin1[] PROGMEM = {
  cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1
};
const short cTibiaMax1[] PROGMEM = {
  cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1
};

//Start positions for the leg
const short cInitPosX[] PROGMEM = {
  cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX
};
const short cInitPosZ[] PROGMEM = {
  cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ
};
const short cInitPosY[] PROGMEM = {
  cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY
};

//Default leg angle
const short cCoxaAngle1[] PROGMEM = {
  cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1
};

//Leg Lengths
const byte cCoxaLength[] PROGMEM = {
  cRRCoxaLength, cRMCoxaLength, cRFCoxaLength, cLRCoxaLength, cLMCoxaLength, cLFCoxaLength
};
const byte cFemurLength[] PROGMEM = {
  cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength
};
const byte cTibiaLength[] PROGMEM = {
  cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength
};

//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = {
  cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX
};
const short cOffsetZ[] PROGMEM = {
  cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ
};

// Angles
short CoxaAngle[NUMBER_OF_LEGS];  //Actual Angle of the Leg
short FemurAngle[NUMBER_OF_LEGS];
short TibiaAngle[NUMBER_OF_LEGS];

// Actual positions, messured in mm from the coxa offset to the tips of the feet
short LegPosX[NUMBER_OF_LEGS];  //Actual Position of the Leg
short LegPosZ[NUMBER_OF_LEGS];
short LegPosY[NUMBER_OF_LEGS];

//--------------------------------------------------------------------
//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1.
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//-    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//-    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//-    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total bytes: 277

static const byte GetACos[] PROGMEM = {
  255, 254, 252, 251, 250, 249, 247, 246, 245, 243, 242, 241, 240, 238, 237, 236, 234, 233, 232, 231, 229, 228, 227, 225,
  224, 223, 221, 220, 219, 217, 216, 215, 214, 212, 211, 210, 208, 207, 206, 204, 203, 201, 200, 199, 197, 196, 195, 193,
  192, 190, 189, 188, 186, 185, 183, 182, 181, 179, 178, 176, 175, 173, 172, 170, 169, 167, 166, 164, 163, 161, 160, 158,
  157, 155, 154, 152, 150, 149, 147, 146, 144, 142, 141, 139, 137, 135, 134, 132, 130, 128, 127, 125, 123, 121, 119, 117,
  115, 113, 111, 109, 107, 105, 103, 101, 98, 96, 94, 92, 89, 87, 84, 81, 79, 76, 73, 73, 73, 72, 72, 72, 71, 71, 71, 70, 70,
  70, 70, 69, 69, 69, 68, 68, 68, 67, 67, 67, 66, 66, 66, 65, 65, 65, 64, 64, 64, 63, 63, 63, 62, 62, 62, 61, 61, 61, 60, 60, 59,
  59, 59, 58, 58, 58, 57, 57, 57, 56, 56, 55, 55, 55, 54, 54, 53, 53, 53, 52, 52, 51, 51, 51, 50, 50, 49, 49, 48, 48, 47, 47, 47,
  46, 46, 45, 45, 44, 44, 43, 43, 42, 42, 41, 41, 40, 40, 39, 39, 38, 37, 37, 36, 36, 35, 34, 34, 33, 33, 32, 31, 31, 30, 29, 28,
  28, 27, 26, 25, 24, 23, 23, 23, 23, 22, 22, 22, 22, 21, 21, 21, 21, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 17, 17, 17, 17, 16,
  16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 6, 6, 5, 3, 0
};  //

//Sin table 90 deg, persision 0.5 deg [180 values]
static const word GetSin[] PROGMEM = {
  0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564,
  1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007,
  3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383,
  4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664,
  5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819,
  6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826,
  7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660,
  8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304,
  9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743,
  9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969,
  9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000
};  //

short AngleDeg1;
short sin4;
short cos4;
short AngleRad4;

short AtanX;
short AtanY;
short Atan4;
long XYhyp2;

//Body Inverse Kinematics
short PosX;
short PosZ;
short PosY;
long BodyFKPosX;  //Output Position X of feet with Rotation
long BodyFKPosY;  //Output Position Y of feet with Rotation
long BodyFKPosZ;  //Output Position Z of feet with Rotation

//Leg Inverse Kinematics
long IKFemurAngle;  //Output Angle of Femur in degrees
long IKTibiaAngle;  //Output Angle of Tibia in degrees
long IKCoxaAngle;   //Output Angle of Coxa in degrees

word ServoMoveTime;      //Time for servo updates
word PrevServoMoveTime;  //Previous time for the servo updates

//[Balance]
long TotalTransX;
long TotalTransY;
long TotalTransZ;
long TotalXBal1;
long TotalYBal1;
long TotalZBal1;

//Single leg control
byte PrevSelectedLeg;
boolean AllDown;

//Gait
short NomGaitSpeed;
byte TLDivFactor;
byte NrLiftedPos;
byte LiftDivFactor;  // Nr of steps that a leg is on the floor while waling
byte FrontDownPos;
byte HalfLiftHeight;  // does the step sequence has half lifted legs
byte TravelRequest;   //Temp to check if the gait is in motion
byte StepsInGait;     // Nr of steps in gait

byte LastLeg;

byte GaitLegNr[NUMBER_OF_LEGS];
byte GaitLegNrIn;

// Relative position corresponding to the Gait
long GaitPosX[NUMBER_OF_LEGS];
long GaitPosY[NUMBER_OF_LEGS];
long GaitPosZ[NUMBER_OF_LEGS];
long GaitRotY[NUMBER_OF_LEGS];  //Relative rotation corresponding to the Gait

static short g_BodyYShift;
static short g_BodyYOffset;

const byte BalanceDivFactor = 6;
boolean fWalking;  //1 if the robot are walking
byte bExtraCycle;  //Forcing some extra timed cycles for avoiding

// Time
unsigned long lTimerStart;  //Start time of the calculation cycles
unsigned long lTimerEnd;    //End time of the calculation cycles
byte CycleTime;

struct RobotStatus {
  byte bHexON;
  // Body position
  long BodyPosX;  //Global Input for the position of the body
  long BodyPosY;  //Global Input for the position of the body
  long BodyPosZ;  //Global Input for the position of the body
  long BodyRotOffsetX;
  long BodyRotOffsetY;
  long BodyRotOffsetZ;

  // Body Inverse Kinematics
  long BodyRotX;  //Global Input pitch of the body
  long BodyRotY;  //Global Input rotation of the body
  long BodyRotZ;  //Global Input roll of the body

  byte GaitType;
  //Init position of the leg
  short LegLiftHeight;   //Current Travel height
  long TravelLengthX;    //Current Travel length X (sideways)
  long TravelLengthZ;    //Current Travel length Z (front - back)
  long TravelRotationY;  //Current Travel Rotation Y

  //Single Leg Control
  byte SelectedLeg;
  long SLLegX;
  long SLLegY;
  long SLLegZ;
  boolean bSLHold;  //Single leg control mode

  //Grip
  long GripPosition;

  //Balance
  boolean bBalanceMode;

  //Timing
  byte InputTimeDelay;    //Delay that depends on the input to get the "sneaking" effect
  word SpeedControl;      //Adjustible Delay
  byte ForceGaitStepCnt;  // new to allow us to force a step even when not moving
  byte OperationMode;

  byte bDblTravel;
  byte bDoubleHeight;
  byte bAdjustLegPositions;
  byte GaitStep;  //Global Input Gait step
} Hex;

long OffsetX[NUMBER_OF_LEGS];  //Distance X from center of the body to the coxa
long OffsetZ[NUMBER_OF_LEGS];  //Distance Z from center of the body to the coxa

//***************************************************

byte readyData = 0;
const byte cTravelDeadZone = 4;

//*************************************************************************
// int min(int a, int b) {
//   return a <= b ? a : b;
// }
// int max(int a, int b) {
//   return a >= b ? a : b;
// }
//****************************************************************************

/******GaitSelect**************************************************************************
 GaitSelect()
 Choose how hexapod moves including tripod, wave,...
 ******************************************************************************************/
void GaitSelect() {
  // Hex Gaits
  //  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground,
  //                      { RR, <RM> RF, LR, <LM>, LF}

  if (Hex.GaitType == 0) {  //Ripple Gait 12 steps (one leg at a time alt. between sides)
    GaitLegNr[RR] = 7;
    GaitLegNr[RM] = 11;
    GaitLegNr[RF] = 3;
    GaitLegNr[LR] = 1;
    GaitLegNr[LM] = 5;
    GaitLegNr[LF] = 9;
    //        GaitLegNr[]={7,11,3,1,5,9};


    StepsInGait = 12;
    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    TLDivFactor = 8;
    HalfLiftHeight = 3;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    Hex.GaitType = 0;
  } else if (Hex.GaitType == 1) {  // Tripod 8
    GaitLegNr[RR] = 1;
    GaitLegNr[RM] = 5;
    GaitLegNr[RF] = 1;
    GaitLegNr[LR] = 5;
    GaitLegNr[LM] = 1;
    GaitLegNr[LF] = 5;
    //        GaitLegNr[]={1,5,1,5,1,5};

    StepsInGait = 8;
    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    TLDivFactor = 4;
    HalfLiftHeight = 3;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    Hex.GaitType = 1;
  } else if (Hex.GaitType == 2) {  //Triple Tripod 12 steps
    GaitLegNr[RR] = 5;
    GaitLegNr[RM] = 10;
    GaitLegNr[RF] = 3;
    GaitLegNr[LR] = 11;
    GaitLegNr[LM] = 4;
    GaitLegNr[LF] = 9;
    //        GaitLegNr[]={5,10,3,11,4,9};

    StepsInGait = 12;
    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    TLDivFactor = 8;
    HalfLiftHeight = 3;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    Hex.GaitType = 2;
  } else if (Hex.GaitType == 3) {  /// Triple Tripod 16 steps, use 5 lifted positions
    GaitLegNr[RR] = 6;
    GaitLegNr[RM] = 13;
    GaitLegNr[RF] = 4;
    GaitLegNr[LR] = 14;
    GaitLegNr[LM] = 5;
    GaitLegNr[LF] = 12;
    //        GaitLegNr[]={6,13,4,14,5,12};

    StepsInGait = 16;
    NrLiftedPos = 5;
    FrontDownPos = 3;
    LiftDivFactor = 4;
    TLDivFactor = 10;
    HalfLiftHeight = 1;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    Hex.GaitType = 3;
  } else if (Hex.GaitType == 4) {  //Wave 24 steps
    GaitLegNr[RR] = 13;
    GaitLegNr[RM] = 17;
    GaitLegNr[RF] = 21;
    GaitLegNr[LR] = 1;
    GaitLegNr[LM] = 5;
    GaitLegNr[LF] = 9;
    //        GaitLegNr[]={13,17,21,1,5,9};

    StepsInGait = 24;
    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    TLDivFactor = 20;
    HalfLiftHeight = 3;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    Hex.GaitType = 4;
  } else if (Hex.GaitType == 5) {  //Tripod 6 steps
    GaitLegNr[RR] = 1;
    GaitLegNr[RM] = 4;
    GaitLegNr[RF] = 1;
    GaitLegNr[LR] = 4;
    GaitLegNr[LM] = 1;
    GaitLegNr[LF] = 4;
    //        GaitLegNr[]={1,4,1,4,1,4};

    StepsInGait = 6;
    NrLiftedPos = 2;
    FrontDownPos = 1;
    LiftDivFactor = 2;
    TLDivFactor = 4;
    HalfLiftHeight = 1;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    Hex.GaitType = 5;
  }
}

/******Gait*********************************************************************************
 Gait()
 Calculate Gait of hexapod
 ******************************************************************************************/
void Gait(byte curLegNr) {

  if ((TravelRequest && (NrLiftedPos == 1 || NrLiftedPos == 3 || NrLiftedPos == 5) && Hex.GaitStep == GaitLegNr[curLegNr]) || (!TravelRequest && Hex.GaitStep == GaitLegNr[curLegNr] && ((abs(GaitPosX[curLegNr]) > 2) || (abs(GaitPosZ[curLegNr]) > 2) || (abs(GaitRotY[curLegNr]) > 2)))) {  //Up
    GaitPosX[curLegNr] = 0;
    GaitPosY[curLegNr] = -Hex.LegLiftHeight;
    GaitPosZ[curLegNr] = 0;
    GaitRotY[curLegNr] = 0;
  }
  //Optional Half height Rear (2, 3, 5 lifted positions)
  else if (((NrLiftedPos == 2 && Hex.GaitStep == GaitLegNr[curLegNr]) || (NrLiftedPos >= 3 && (Hex.GaitStep == GaitLegNr[curLegNr] - 1 || Hex.GaitStep == GaitLegNr[curLegNr] + (StepsInGait - 1))))
           && TravelRequest) {
    GaitPosX[curLegNr] = -Hex.TravelLengthX / LiftDivFactor;
    GaitPosY[curLegNr] = -3 * Hex.LegLiftHeight / (3 + HalfLiftHeight);  //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[curLegNr] = -Hex.TravelLengthZ / LiftDivFactor;
    GaitRotY[curLegNr] = -Hex.TravelRotationY / LiftDivFactor;
  }
  // _A_
  // Optional Half height front (2, 3, 5 lifted positions)
  else if ((NrLiftedPos >= 2) && (Hex.GaitStep == GaitLegNr[curLegNr] + 1 || Hex.GaitStep == GaitLegNr[curLegNr] - (StepsInGait - 1)) && TravelRequest) {
    GaitPosX[curLegNr] = Hex.TravelLengthX / LiftDivFactor;
    GaitPosY[curLegNr] = -3 * Hex.LegLiftHeight / (3 + HalfLiftHeight);  // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[curLegNr] = Hex.TravelLengthZ / LiftDivFactor;
    GaitRotY[curLegNr] = Hex.TravelRotationY / LiftDivFactor;
  }

  //Optional Half height Rear 5 LiftedPos (5 lifted positions)
  else if (((NrLiftedPos == 5 && (Hex.GaitStep == GaitLegNr[curLegNr] - 2))) && TravelRequest) {
    GaitPosX[curLegNr] = -Hex.TravelLengthX / 2;
    GaitPosY[curLegNr] = -Hex.LegLiftHeight / 2;
    GaitPosZ[curLegNr] = -Hex.TravelLengthZ / 2;
    GaitRotY[curLegNr] = -Hex.TravelRotationY / 2;
  }

  //Optional Half height Front 5 LiftedPos (5 lifted positions)
  else if ((NrLiftedPos == 5) && (Hex.GaitStep == GaitLegNr[curLegNr] + 2 || Hex.GaitStep == GaitLegNr[curLegNr] - (StepsInGait - 2)) && TravelRequest) {
    GaitPosX[curLegNr] = Hex.TravelLengthX / 2;
    GaitPosY[curLegNr] = -Hex.LegLiftHeight / 2;
    GaitPosZ[curLegNr] = Hex.TravelLengthZ / 2;
    GaitRotY[curLegNr] = Hex.TravelRotationY / 2;
  }
  //_B_
  //Leg front down position //bug here?  From _A_ to _B_ there should only be one Hex.GaitStep, not 2!
  //For example, where is the case of Hex.GaitStep==GaitLegNr[curLegNr]+2 executed when NRLiftedPos=3?
  else if ((Hex.GaitStep == GaitLegNr[curLegNr] + FrontDownPos || Hex.GaitStep == GaitLegNr[curLegNr] - (StepsInGait - FrontDownPos))
           && GaitPosZ[curLegNr] < 0) {
    GaitPosX[curLegNr] = Hex.TravelLengthX / 2;
    GaitPosZ[curLegNr] = Hex.TravelLengthZ / 2;
    GaitPosY[curLegNr] = 0;
    GaitRotY[curLegNr] = Hex.TravelRotationY / 2;
  }

  //Move body forward
  else {
    GaitPosX[curLegNr] = GaitPosX[curLegNr] - (Hex.TravelLengthX / TLDivFactor);
    GaitPosY[curLegNr] = 0;
    GaitPosZ[curLegNr] = GaitPosZ[curLegNr] - (Hex.TravelLengthZ / TLDivFactor);
    GaitRotY[curLegNr] = GaitRotY[curLegNr] - (Hex.TravelRotationY / TLDivFactor);
  }
}

//--------------------------------------------------------------------
////[GAIT Sequence]
void GaitSeq(void) {
  int LegNr;
  //Check if the Gait is in motion
  if (fWalking || (Hex.ForceGaitStepCnt != 0))
    TravelRequest = 1;  // Is walking or was walking...
  else
    TravelRequest = ((abs(Hex.TravelLengthX) > cTravelDeadZone)
                     || (abs(Hex.TravelLengthZ) > cTravelDeadZone)
                     || (abs(Hex.TravelRotationY) > cTravelDeadZone)
                     || (Hex.ForceGaitStepCnt != 0) || fWalking);

  //Clear values under the cTravelDeadZone
  if (!TravelRequest) {
    Hex.TravelLengthX = 0;
    Hex.TravelLengthZ = 0;
    Hex.TravelRotationY = 0;  //Gait NOT in motion, return to home position
  }

  //Calculate Gait sequence
  LastLeg = 0;
  for (LegNr = 0; LegNr < NUMBER_OF_LEGS; ++LegNr) {  // for all legs
    if (LegNr == 5)                                   // last leg
      LastLeg = 1;
    Gait(LegNr);
  }  // next leg

  //Advance to the next step
  if (LastLeg) {  //The last leg in this step
    Hex.GaitStep++;
    if (Hex.GaitStep > StepsInGait)
      Hex.GaitStep = 1;
  }

  // If we have a force count decrement it now...
  if (Hex.ForceGaitStepCnt)
    Hex.ForceGaitStepCnt--;
}

//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1     - Input Angle in degrees
//sin4        - Output Sinus of AngleDeg
//cos4          - Output Cosinus of AngleDeg
void GetSinCos(short AngleDeg1) {
  short ABSAngleDeg1;  //Absolute value of the Angle in Degrees, decimals = 1
  //Get the absolute value of AngleDeg
  if (AngleDeg1 < 0)
    ABSAngleDeg1 = AngleDeg1 * -1;
  else
    ABSAngleDeg1 = AngleDeg1;

  //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
  if (AngleDeg1 < 0)  //Negative values
    AngleDeg1 = 3600 - (ABSAngleDeg1 - (3600 * (ABSAngleDeg1 / 3600)));
  else  //Positive values
    AngleDeg1 = ABSAngleDeg1 - (3600 * (ABSAngleDeg1 / 3600));

  if (AngleDeg1 >= 0 && AngleDeg1 <= 900)  // 0 to 90 deg
  {
    sin4 = pgm_read_word(&GetSin[AngleDeg1 / 5]);
    cos4 = pgm_read_word(&GetSin[(900 - (AngleDeg1)) / 5]);
  }

  else if (AngleDeg1 > 900 && AngleDeg1 <= 1800)  // 90 to 180 deg
  {
    sin4 = pgm_read_word(&GetSin[(900 - (AngleDeg1 - 900)) / 5]);
    cos4 = -pgm_read_word(&GetSin[(AngleDeg1 - 900) / 5]);
  } else if (AngleDeg1 > 1800 && AngleDeg1 <= 2700)  // 180 to 270 deg
  {
    sin4 = -pgm_read_word(&GetSin[(AngleDeg1 - 1800) / 5]);
    cos4 = -pgm_read_word(&GetSin[(2700 - AngleDeg1) / 5]);
  }

  else if (AngleDeg1 > 2700 && AngleDeg1 <= 3600)  // 270 to 360 deg
  {
    sin4 = -pgm_read_word(&GetSin[(3600 - AngleDeg1) / 5]);
    cos4 = pgm_read_word(&GetSin[(AngleDeg1 - 2700) / 5]);
  }
}


//--------------------------------------------------------------------
//(GETARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//cos4        - Input Cosinus
//AngleRad4     - Output Angle in AngleRad4
int GetArcCos(short cos4) {
  boolean NegativeValue /*:1*/;
  if (cos4 < 0) {
    cos4 = -cos4;
    NegativeValue = 1;
  } else
    NegativeValue = 0;

  cos4 = min(cos4, c4DEC);

  if ((cos4 >= 0) && (cos4 < 9000)) {
    AngleRad4 = (byte)pgm_read_byte(&GetACos[cos4 / 79]);
    AngleRad4 = ((long)AngleRad4 * 616) / c1DEC;
  } else if ((cos4 >= 9000) && (cos4 < 9900)) {
    AngleRad4 = (byte)pgm_read_byte(&GetACos[(cos4 - 9000) / 8 + 114]);
    AngleRad4 = (long)((long)AngleRad4 * 616) / c1DEC;
  } else if ((cos4 >= 9900) && (cos4 <= 10000)) {
    AngleRad4 = (byte)pgm_read_byte(&GetACos[(cos4 - 9900) / 2 + 227]);
    AngleRad4 = (long)((long)AngleRad4 * 616) / c1DEC;
  }

  if (NegativeValue)
    AngleRad4 = 31416 - AngleRad4;

  return AngleRad4;
}

unsigned long isqrt32(unsigned long n)  //
{
  unsigned long root;
  unsigned long remainder;
  unsigned long place;

  root = 0;
  remainder = n;
  place = 0x40000000;  // OR place = 0x4000; OR place = 0x40; - respectively

  while (place > remainder)
    place = place >> 2;
  while (place) {
    if (remainder >= root + place) {
      remainder = remainder - root - place;
      root = root + (place << 1);
    }
    root = root >> 1;
    place = place >> 2;
  }
  return root;
}


//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4          - Output ARCTAN2(X/Y)
//XYhyp2            - Output presenting Hypotenuse of X and Y
short GetATan2(short AtanX, short AtanY) {
  XYhyp2 = isqrt32(((long)AtanX * AtanX * c4DEC) + ((long)AtanY * AtanY * c4DEC));
  GetArcCos(((long)AtanX * (long)c6DEC) / (long)XYhyp2);

  if (AtanY < 0)
    Atan4 = -AngleRad4;
  else
    Atan4 = AngleRad4;
  return Atan4;
}
//--------------------------------------------------------------------
//[BalCalcOneLeg]
void BalCalcOneLeg(long PosX, long PosZ, long PosY, byte BalLegNr) {
  long CPR_X;  //Final X value for centerpoint of rotation
  long CPR_Y;  //Final Z value for centerpoint of rotation
  long CPR_Z;  //Final Y value for centerpoint of rotation
  long lAtan;

  //Calculating totals from center of the body to the feet
  CPR_Z = (short)pgm_read_word(&cOffsetZ[BalLegNr]) + PosZ;
  CPR_X = (short)pgm_read_word(&cOffsetX[BalLegNr]) + PosX;
  CPR_Y = 150 + PosY;  // using the value 150 to lower the centerpoint of rotation 'Hex.BodyPosY +

  TotalTransY += (long)PosY;
  TotalTransZ += (long)CPR_Z;
  TotalTransX += (long)CPR_X;

  lAtan = GetATan2(CPR_X, CPR_Z);
  TotalYBal1 += (lAtan * 1800) / 31415;

  lAtan = GetATan2(CPR_X, CPR_Y);
  TotalZBal1 += ((lAtan * 1800) / 31415) - 900;  //Rotate balance circle 90 deg

  lAtan = GetATan2(CPR_Z, CPR_Y);
  TotalXBal1 += ((lAtan * 1800) / 31415) - 900;  //Rotate balance circle 90 deg
}
//--------------------------------------------------------------------
//[BalanceBody]
void BalanceBody(void) {
  TotalTransZ = TotalTransZ / BalanceDivFactor;
  TotalTransX = TotalTransX / BalanceDivFactor;
  TotalTransY = TotalTransY / BalanceDivFactor;

  if (TotalYBal1 > 0)  //Rotate balance circle by +/- 180 deg
    TotalYBal1 -= 1800;
  else
    TotalYBal1 += 1800;

  if (TotalZBal1 < -1800)  //Compensate for extreme balance positions that causes owerflow
    TotalZBal1 += 3600;

  if (TotalXBal1 < -1800)  //Compensate for extreme balance positions that causes owerflow
    TotalXBal1 += 3600;

  //Balance rotation
  TotalYBal1 = -TotalYBal1 / BalanceDivFactor;
  TotalXBal1 = -TotalXBal1 / BalanceDivFactor;
  TotalZBal1 = TotalZBal1 / BalanceDivFactor;
}
//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
void SingleLegControl(void) {
  int LegNr;
  //Check if all legs are down
  if ((LegPosY[RF] == (short)pgm_read_word(&cInitPosY[RF]))
      && (LegPosY[RM] == (short)pgm_read_word(&cInitPosY[RM]))
      && (LegPosY[RR] == (short)pgm_read_word(&cInitPosY[RR]))
      && (LegPosY[LR] == (short)pgm_read_word(&cInitPosY[LR]))
      && (LegPosY[LM] == (short)pgm_read_word(&cInitPosY[LM]))
      && (LegPosY[LF] == (short)pgm_read_word(&cInitPosY[LF]))) {
    AllDown = 1;
  }

  if (Hex.SelectedLeg < NUMBER_OF_LEGS) {
    if (Hex.SelectedLeg != PrevSelectedLeg) {
      if (AllDown) {  //Lift leg a bit when it got selected
        LegPosY[Hex.SelectedLeg] = (short)pgm_read_word(&cInitPosY[Hex.SelectedLeg]) - 20;

        //Store current status
        PrevSelectedLeg = Hex.SelectedLeg;
      } else {  //Return prev leg back to the init position
        LegPosX[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosX[PrevSelectedLeg]);
        LegPosY[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosY[PrevSelectedLeg]);
        LegPosZ[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosZ[PrevSelectedLeg]);
      }
    } else if (!Hex.bSLHold) {
      LegPosY[Hex.SelectedLeg] = LegPosY[Hex.SelectedLeg] + Hex.SLLegY;
      //LegPosY[Hex.SelectedLeg] = (short)pgm_read_word(&cInitPosY[Hex.SelectedLeg]) + Hex.SLLegY;  // Using DIY remote Zenta prefer it this way
      LegPosX[Hex.SelectedLeg] = (short)pgm_read_word(&cInitPosX[Hex.SelectedLeg]) + Hex.SLLegX;
      LegPosZ[Hex.SelectedLeg] = (short)pgm_read_word(&cInitPosZ[Hex.SelectedLeg]) + Hex.SLLegZ;
    }
  } else {  //All legs to init position
    if (!AllDown) {
      for (LegNr = 0; LegNr < NUMBER_OF_LEGS; ++LegNr) {
        LegPosX[LegNr] = (short)pgm_read_word(&cInitPosX[LegNr]);
        LegPosY[LegNr] = (short)pgm_read_word(&cInitPosY[LegNr]);
        LegPosZ[LegNr] = (short)pgm_read_word(&cInitPosZ[LegNr]);
      }
    }
    if (PrevSelectedLeg != 255)
      PrevSelectedLeg = 255;
  }
}
/******BodyIK***********************************************************************
 BodyIK()
 Calculates necessary foot position (leg space) to acheive commanded body rotations, translations, and gait inputs
 ***********************************************************************************************************/
void BodyFK(short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg) {

  short SinA4;  //Sin buffer for BodyRotX calculations
  short CosA4;  //Cos buffer for BodyRotX calculations
  short SinB4;  //Sin buffer for BodyRotX calculations
  short CosB4;  //Cos buffer for BodyRotX calculations
  short SinG4;  //Sin buffer for BodyRotY calculations
  short CosG4;  //Cos buffer for BodyRotY calculations
  short CPR_X;  //Total X distance between the center of the body and the feet
  short CPR_Z;  //Total Z distance between the center of the body and the feet
  short CPR_Y;  //Total Y distance between the center of the body and the feet
  double bodyBalance[3];
  pid_update(bodyBalance, 3);

#ifdef DEBUG
  Serial.print(F("angle BodyIK"));
  Serial.print(F("\t"));
  Serial.print(bodyBalance[0]);
  Serial.print(F("\t"));
  Serial.print(bodyBalance[1]);
  Serial.print(F("\t"));
  Serial.println(bodyBalance[2]);
#endif  // DEBUG

  CPR_X = (short)pgm_read_word(&cOffsetX[BodyIKLeg]) + PosX + Hex.BodyRotOffsetX;
  CPR_Y = PosY + Hex.BodyRotOffsetY;
  CPR_Z = (short)pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ + Hex.BodyRotOffsetZ;

  GetSinCos(Hex.BodyRotX + TotalXBal1 + bodyBalance[0]);
  SinG4 = sin4;
  CosG4 = cos4;

  GetSinCos(Hex.BodyRotZ + TotalZBal1 + bodyBalance[1]);
  SinB4 = sin4;
  CosB4 = cos4;


  GetSinCos(Hex.BodyRotY + (RotationY * c1DEC) + TotalYBal1 + bodyBalance[2]);

  SinA4 = sin4;
  CosA4 = cos4;

  BodyFKPosX = ((long)CPR_X * c2DEC - ((long)CPR_X * c2DEC * CosA4 / c4DEC * CosB4 / c4DEC - (long)CPR_Z * c2DEC * CosB4 / c4DEC * SinA4 / c4DEC + (long)CPR_Y * c2DEC * SinB4 / c4DEC)) / c2DEC;
  BodyFKPosZ = ((long)CPR_Z * c2DEC - ((long)CPR_X * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC + (long)CPR_X * c2DEC * CosA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC + (long)CPR_Z * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC - (long)CPR_Z * c2DEC * SinA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC - (long)CPR_Y * c2DEC * CosB4 / c4DEC * SinG4 / c4DEC)) / c2DEC;
  BodyFKPosY = ((long)CPR_Y * c2DEC - ((long)CPR_X * c2DEC * SinA4 / c4DEC * SinG4 / c4DEC - (long)CPR_X * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC * SinB4 / c4DEC + (long)CPR_Z * c2DEC * CosA4 / c4DEC * SinG4 / c4DEC + (long)CPR_Z * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC * SinB4 / c4DEC + (long)CPR_Y * c2DEC * CosB4 / c4DEC * CosG4 / c4DEC)) / c2DEC;
}
//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Z
//IKFeetPosZ            - Input Position of the Feet Y
//IKSolution            - Output 1 if the solution is possible
//IKSolutionWarning     - Output 1 if the solution is NEARLY possible
//IKSolutionError    - Output 1 if the solution is NOT possible
//FemurAngle           - Output Angle of Femur in degrees
//TibiaAngle           - Output Angle of Tibia in degrees
//CoxaAngle            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
void LegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr) {
  unsigned long IKSW2;  //Length between Shoulder and Wrist, decimals = 2
  unsigned long IKA14;  //Angle of the line S>W with respect to the ground in radians, decimals = 4
  unsigned long IKA24;  //Angle of the line S>W with respect to the femur in radians, decimals = 4
  short IKFeetPosXZ;    //Diagonal direction from Input X and Z

  long Temp1;
  long Temp2;
  long T3;

  GetATan2(IKFeetPosX, IKFeetPosZ);
  CoxaAngle[LegIKLegNr] = (((long)Atan4 * 180) / 3141) + (short)pgm_read_word(&cCoxaAngle1[LegIKLegNr]);

  IKFeetPosXZ = XYhyp2 / c2DEC;

  IKA14 = GetATan2(IKFeetPosY, IKFeetPosXZ - (byte)pgm_read_byte(&cCoxaLength[LegIKLegNr]));

  IKSW2 = XYhyp2;

  Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[LegIKLegNr]) * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr])) - ((long)(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]) * (byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]))) * c4DEC + ((long)IKSW2 * IKSW2));
  Temp2 = (long)(2 * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr])) * c2DEC * (unsigned long)IKSW2;
  T3 = Temp1 / (Temp2 / c4DEC);
  IKA24 = GetArcCos(T3);
  //IKFemurAngle

  FemurAngle[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900;


  Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[LegIKLegNr]) * (byte)pgm_read_byte(&cFemurLength[LegIKLegNr])) + ((long)(byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]) * (byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]))) * c4DEC - ((long)IKSW2 * IKSW2));
  Temp2 = 2 * ((long)((byte)pgm_read_byte(&cFemurLength[LegIKLegNr]))) * (long)((byte)pgm_read_byte(&cTibiaLength[LegIKLegNr]));
  GetArcCos(Temp1 / Temp2);

  TibiaAngle[LegIKLegNr] = -(900 - (long)AngleRad4 * 180 / 3141);


  //Set the Solution quality
  //  if(IKSW2 < ((uint16_t)((byte)cFemurLength[LegIKLegNr]+(byte)cTibiaLength[LegIKLegNr]-30)*c2DEC))
  //    IKSolution = 1;
  //  else
  //  {
  //    if(IKSW2 < ((uint16_t)((byte)cFemurLength[LegIKLegNr]+(byte)cTibiaLength[LegIKLegNr])*c2DEC))
  //      IKSolutionWarning = 1;
  //    else
  //      IKSolutionError = 1    ;
  //  }
}
//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void) {
  int LegNr;
  for (LegNr = 0; LegNr < NUMBER_OF_LEGS; ++LegNr) {
    CoxaAngle[LegNr] = min(max(CoxaAngle[LegNr], (short)pgm_read_word(&cCoxaMin1[LegNr])),
                           (short)pgm_read_word(&cCoxaMax1[LegNr]))
                       + (short)pgm_read_word(&CoxaAngleOffset[LegNr]);
    FemurAngle[LegNr] = min(max(FemurAngle[LegNr], (short)pgm_read_word(&cFemurMin1[LegNr])),
                            (short)pgm_read_word(&cFemurMax1[LegNr]))
                        + (short)pgm_read_word(&FemurAngleOffset[LegNr]);
    TibiaAngle[LegNr] = min(max(TibiaAngle[LegNr], (short)pgm_read_word(&cTibiaMin1[LegNr])),
                            (short)pgm_read_word(&cTibiaMax1[LegNr]))
                        + (short)pgm_read_word(&TibiaAngleOffset[LegNr]);
  }
}
void Initialize() {
  int LegNr;

  for (LegNr = 0; LegNr < NUMBER_OF_LEGS; ++LegNr) {
    LegPosX[LegNr] = (short)pgm_read_word(&cInitPosX[LegNr]);  //Set start positions for each leg
    LegPosZ[LegNr] = (short)pgm_read_word(&cInitPosZ[LegNr]);
    LegPosY[LegNr] = (short)pgm_read_word(&cInitPosY[LegNr]);
  }

  //Single leg control. Make sure no leg is selected
  Hex.SelectedLeg = 255;  // No Leg selected
  PrevSelectedLeg = 255;

  //Body Positions
  Hex.BodyPosX = 0;
  Hex.BodyPosZ = 0;
  Hex.BodyPosY = 0;

  //Body Rotations
  Hex.BodyRotX = 0;
  Hex.BodyRotZ = 0;
  Hex.BodyRotY = 0;

  Hex.BodyRotOffsetX = 0;
  Hex.BodyRotOffsetZ = 0;  //Input Z offset value to adjust centerpoint of rotation
  Hex.BodyRotOffsetY = 0;
  Hex.TravelLengthX = 0;
  Hex.TravelLengthZ = 0;
  Hex.TravelRotationY = 0;

  Hex.GaitType = 1;
  Hex.bBalanceMode = 0;
  Hex.LegLiftHeight = 50;
  Hex.ForceGaitStepCnt = 0;  // added to try to adjust starting positions depending on height...
  Hex.GaitStep = 1;
  GaitSelect();

  // Input controller initialization
  //    Hex.bAdjustLegPositions = 0;
  Hex.OperationMode = MODE_WALK;
  Hex.GripPosition = 800;
  Hex.bDblTravel = 0;
  Hex.bDoubleHeight = 0;

  Hex.SpeedControl = 100;

  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  ServoMoveTime = 100;
  dataReady = 0;
  Hex.bHexON = 0;
}

//--------------------------------------------------------------------
// SmoothControl (From Zenta) -  This function makes the body
//            rotation and translation much smoother
//--------------------------------------------------------------------
short SmoothControl(short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider) {

  if (CtrlMoveOut < (CtrlMoveInp - 4))
    return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp) / CtrlDivider);
  else if (CtrlMoveOut > (CtrlMoveInp + 4))
    return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp) / CtrlDivider);

  return CtrlMoveInp;
}

//==============================================================================
// AdjustLegPositions() - Will adjust the init leg positions to the width passed in.
//==============================================================================
#ifndef MIN_XZ_LEG_ADJUST
#define MIN_XZ_LEG_ADJUST cXXCoxaLength
#endif

#ifndef MAX_XZ_LEG_ADJUST
#define MAX_XZ_LEG_ADJUST (cXXCoxaLength + cXXTibiaLength + cXXFemurLength / 4)
#endif

void AdjustLegPositions(word XZLength1) {
  if (XZLength1 > MAX_XZ_LEG_ADJUST)
    XZLength1 = MAX_XZ_LEG_ADJUST;
  if (XZLength1 < MIN_XZ_LEG_ADJUST)
    XZLength1 = MIN_XZ_LEG_ADJUST;

  for (byte LegNr = 0; LegNr < NUMBER_OF_LEGS; ++LegNr) {
    GetSinCos((short)pgm_read_word(&cCoxaAngle1[LegNr]));
    LegPosX[LegNr] = ((long)((long)cos4 * XZLength1)) / c4DEC;
    LegPosZ[LegNr] = -((long)((long)sin4 * XZLength1)) / c4DEC;
  }
  // Make sure we cycle through one gait to have the legs all move into their new locations...
  Hex.ForceGaitStepCnt = StepsInGait;
}

//--------------------------------------------------------------------
// AdjustLegPositionsToBodyHeight() - Will try to adjust the position of the legs
//     to be appropriate for the current y location of the body...
//--------------------------------------------------------------------

byte g_iLegInitIndex = 0x00;  // remember which index we are currently using...

void AdjustLegPositionsToBodyHeight(void) {
#ifdef CNT_HEX_INITS
  // Lets see which of our units we should use...
  // Note: We will also limit our body height here...
  if (Hex.BodyPosY > (short)pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS - 1]))
    Hex.BodyPosY = (short)pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS - 1]);

  byte i;
  word XZLength1 = pgm_read_byte(&g_abHexIntXZ[CNT_HEX_INITS - 1]);
  for (i = 0; i < (CNT_HEX_INITS - 1); i++) {  // Don't need to look at last entry as we already init to assume this one...
    if (Hex.BodyPosY <= (short)pgm_read_byte(&g_abHexMaxBodyY[i])) {
      XZLength1 = pgm_read_byte(&g_abHexIntXZ[i]);
      break;
    }
  }
  if (i != g_iLegInitIndex) {
    g_iLegInitIndex = i;  // remember the current index...
    AdjustLegPositions(XZLength1);
  }
#endif  // CNT_HEX_INITS
}

void StepControlServo(uint16_t slave_addr, byte channel, byte ServoNr, int ServoAngle) {
  if (R_DRIVER == slave_addr)
    pwmRight.setPWM(channel, 0, map(ServoAngle, -900, 900, MIN_PULSE, MAX_PULSE));
  else if (L_DRIVER == slave_addr)
    pwmLeft.setPWM(channel, 0, map(ServoAngle, -900, 900, MIN_PULSE, MAX_PULSE));
}

void ControlServo() {
  int LegNr;
#ifndef DEBUG
  for (LegNr = 0; LegNr < 6; ++LegNr) {
    switch (LegNr) {
      case 0:  //RR
        StepControlServo(R_DRIVER, 6, 0, CoxaAngle[LegNr]);
        StepControlServo(R_DRIVER, 7, 1, -FemurAngle[LegNr]);
        StepControlServo(R_DRIVER, 8, 2, -TibiaAngle[LegNr]);
        break;
      case 1:  //RM
        StepControlServo(R_DRIVER, 3, 3, CoxaAngle[LegNr]);
        StepControlServo(R_DRIVER, 4, 4, -FemurAngle[LegNr]);
        StepControlServo(R_DRIVER, 5, 5, -TibiaAngle[LegNr]);
        break;
      case 2:  //RF
        StepControlServo(R_DRIVER, 0, 6, CoxaAngle[LegNr]);
        StepControlServo(R_DRIVER, 1, 7, -FemurAngle[LegNr]);
        StepControlServo(R_DRIVER, 2, 8, -TibiaAngle[LegNr]);
        break;
      case 3:  //LR
        StepControlServo(L_DRIVER, 0, 9, -CoxaAngle[LegNr]);
        StepControlServo(L_DRIVER, 1, 10, -FemurAngle[LegNr]);
        StepControlServo(L_DRIVER, 2, 11, -TibiaAngle[LegNr]);
        break;
      case 4:  //LM
        StepControlServo(L_DRIVER, 3, 12, -CoxaAngle[LegNr]);
        StepControlServo(L_DRIVER, 4, 13, -FemurAngle[LegNr]);
        StepControlServo(L_DRIVER, 5, 14, -TibiaAngle[LegNr]);
        break;
      case 5:  //LF
        StepControlServo(L_DRIVER, 8, 15, -CoxaAngle[LegNr]);
        StepControlServo(L_DRIVER, 7, 16, -FemurAngle[LegNr]);
        StepControlServo(L_DRIVER, 6, 17, -TibiaAngle[LegNr]);
        break;
      default:
        break;
    }
  }
#else  //DEBUG
  for (LegNr = 0; LegNr < 6; ++LegNr) {
    switch (LegNr) {
      case 0:  //RR
        Serial.print(F("RR"));
        Serial.print(F("\t"));
        Serial.print(CoxaAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.print(-FemurAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.println(-TibiaAngle[LegNr]);
        break;
      case 1:  //RM
        Serial.print(F("RM"));
        Serial.print(F("\t"));
        Serial.print(CoxaAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.print(-FemurAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.println(-TibiaAngle[LegNr]);
        break;
      case 2:  //RF
        Serial.print(F("RF"));
        Serial.print(F("\t"));
        Serial.print(CoxaAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.print(-FemurAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.println(-TibiaAngle[LegNr]);
        break;
      case 3:  //LR
        Serial.print(F("LR"));
        Serial.print(F("\t"));
        Serial.print(-CoxaAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.print(-FemurAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.println(-TibiaAngle[LegNr]);
        break;
      case 4:  //LM
        Serial.print(F("LM"));
        Serial.print(F("\t"));
        Serial.print(-CoxaAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.print(-FemurAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.println(-TibiaAngle[LegNr]);
        break;
      case 5:  //LF
        Serial.print(F("LF"));
        Serial.print(F("\t"));
        Serial.print(-CoxaAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.print(-FemurAngle[LegNr]);
        Serial.print(F("\t"));
        Serial.println(-TibiaAngle[LegNr]);
        break;
      default:
        break;
    }
  }
#endif
}

void GripHand(int GripPos) {
  // StepControlServo(R_DRIVER, 15, 18, GripPos);
}
void HexOFF(void) {
  //Turn off
  // Hex.BodyPosX = 0;
  // Hex.BodyPosY = 0;
  // Hex.BodyPosZ = 0;
  // Hex.BodyRotX = 0;
  // Hex.BodyRotY = 0;
  // Hex.BodyRotZ = 0;
  // Hex.TravelLengthX = 0;
  // Hex.TravelLengthZ = 0;
  // Hex.TravelRotationY = 0;
  // g_BodyYOffset = 0;
  // g_BodyYShift = 0;
  // Hex.SelectedLeg = 255;
  // Hex.bHexON = 0;
  // AdjustLegPositionsToBodyHeight();  // Put main workings into main program file
  Initialize();
  delay(10);
  CheckAngles();
  ControlServo();
  delay(500);
  pwmRight.sleep();
  pwmLeft.sleep();
}
void CommandInput() {
  uint16_t checksum;
  Hex.bAdjustLegPositions = 0;
  short dwButton;
  static byte nMode = 0;
  static byte cError = 0;

  // Lets check the checksum...
  checksum = reData[1] + reData[2] + reData[3] + reData[4] + reData[5] + reData[6];
  checksum = (checksum >> 8) + (checksum & 0xFF);
  //	UARTprintf("crc = %x\n",checksum);
  if (reData[0] == checksum) {
    cError = 0;
    dwButton = (reData[1] << 8) | reData[2];
    byte lx = (byte)reData[3];
    byte ly = (byte)reData[4];
    byte rx = (byte)reData[5];
    byte ry = (byte)reData[6];
    // Turn ON/OFF
    if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_ON_OFF)) {
      if (Hex.bHexON) {
        HexOFF();
      } else {
        Hex.bHexON = 1;
        Hex.bAdjustLegPositions = 1;
      }
    }
    if (Hex.bHexON) {
      // Switch operation mode
      if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_MODE)) {
        nMode += 1;
        if (nMode > CONTROL_MODE - 1)
          nMode = 0;
        switch (nMode) {
          case 0:
            Hex.OperationMode = MODE_WALK;
            break;
          case 1:
            Hex.OperationMode = MODE_TRANSLATE;
            break;
          case 2:
            Hex.OperationMode = MODE_ROTATE;
            break;
          case 3:
            Hex.OperationMode = MODE_SINGLE_LEG;
            break;
          default:
            break;
        }
      }
      // if (BUTTON_PRESSED(dwButton, INPUT_BODY_BALANCE))
      // {
      //   Hex.bBodyBalanceMode = !Hex.bBodyBalanceMode;
      // }

      if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_BALANCE))
        Hex.bBalanceMode = !Hex.bBalanceMode;

      if (BUTTON_PRESSED(dwButton, INPUT_TOGGLE_BODY_HEIGHT)) {
        if (g_BodyYOffset > 0)
          g_BodyYOffset = 0;
        else
          g_BodyYOffset = 40;
        Hex.bAdjustLegPositions = 1;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BODY_UP)) {
        g_BodyYOffset += 10;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
        Hex.bAdjustLegPositions = 1;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BODY_DOWN) && g_BodyYOffset) {
        g_BodyYOffset -= 10;
        if (g_BodyYOffset < 0) {
          g_BodyYOffset = 0;
        }
        Hex.bAdjustLegPositions = 1;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_SPEED_UP)) {
        Hex.SpeedControl -= 50;
        if (Hex.SpeedControl < 0)
          Hex.SpeedControl = 0;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_SPEED_DOWN)) {
        Hex.SpeedControl += 50;
        if (Hex.SpeedControl > 500) {
          Hex.SpeedControl = 500;
        }
      }

      if (BUTTON_PRESSED(dwButton, INPUT_GRIP_CLOSE)) {
        Hex.GripPosition = Hex.GripPosition - 100;
        if (Hex.GripPosition < -800) {
          Hex.GripPosition = -800;
        }
        GripHand(Hex.GripPosition);
      }

      if (BUTTON_PRESSED(dwButton, INPUT_GRIP_OPEN)) {
        Hex.GripPosition = Hex.GripPosition + 100;
        if (Hex.GripPosition > 800) {
          Hex.GripPosition = 800;
        }
        GripHand(Hex.GripPosition);
      }

      if (Hex.OperationMode == MODE_WALK) {
        //Switch gait
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL)) {
          if (abs(Hex.TravelLengthX) < CONFIG_TRAVEL_DEAD_ZONE && abs(Hex.TravelLengthZ) < CONFIG_TRAVEL_DEAD_ZONE && abs(Hex.TravelRotationY * 2) < CONFIG_TRAVEL_DEAD_ZONE) {
            Hex.GaitType += 1;
            if (Hex.GaitType > GAIT_TYPE - 1)
              Hex.GaitType = 0;
          }
          GaitSelect();
        }
        // Double leg lift height
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R1)) {
          Hex.bDoubleHeight = !Hex.bDoubleHeight;
          if (Hex.bDoubleHeight)
            Hex.LegLiftHeight = 80;
          else
            Hex.LegLiftHeight = 50;
        }
        // Double Travel Length
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2)) {
          Hex.bDblTravel = !Hex.bDblTravel;
        }

        Hex.TravelLengthX = -(lx - 128);
        Hex.TravelLengthZ = (ly - 128);

        if (!Hex.bDblTravel) {
          Hex.TravelLengthX = Hex.TravelLengthX / 2;
          Hex.TravelLengthZ = Hex.TravelLengthZ / 2;
        }

        Hex.TravelRotationY = -(rx - 128) / 4;  //Right Stick Left/Right
      }

      // Translate
      //            g_BodyYShift = 0;
      if (Hex.OperationMode == MODE_TRANSLATE) {
        //                Hex.BodyPosX = SmoothControl((lx - 128)/2, Hex.BodyPosX, SmDiv);
        //                Hex.BodyPosZ = SmoothControl(-(ly - 128)/3, Hex.BodyPosZ, SmDiv);
        //                Hex.BodyRotY = SmoothControl((rx - 128)*2, Hex.BodyRotY, SmDiv);
        Hex.BodyPosX = (lx - 128) / 2;
        Hex.BodyPosZ = -(ly - 128) / 3;
        Hex.BodyRotY = (rx - 128) * 2;
        g_BodyYShift = (-(ry - 128) / 2);
      }
      if (Hex.OperationMode == MODE_ROTATE) {
        //                Hex.BodyRotX = SmoothControl((ly - 128), Hex.BodyRotX, SmDiv);
        //                Hex.BodyRotY = SmoothControl((rx - 128)*2, Hex.BodyRotY, SmDiv);
        //                Hex.BodyRotZ = SmoothControl((lx - 128), Hex.BodyRotZ, SmDiv);
        //                g_BodyYShift = SmoothControl((-(ry - 128)/2),g_BodyYShift, SmDiv);
        Hex.BodyRotX = (ly - 128);
        Hex.BodyRotY = (rx - 128) * 2;
        Hex.BodyRotZ = (lx - 128);
        g_BodyYShift = (-(ry - 128) / 2);
      }
      if (Hex.OperationMode == MODE_SINGLE_LEG) {
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL)) {  // Select Button Test
          if (Hex.SelectedLeg < 5)
            Hex.SelectedLeg = Hex.SelectedLeg + 1;
          else
            Hex.SelectedLeg = 0;
        }
        Hex.SLLegX = (lx - 128) / 2;
        Hex.SLLegZ = (ly - 128) / 2;
        Hex.SLLegY = (ry - 128) / 10;

        // Hold single leg in place
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2)) {
          Hex.bSLHold = !Hex.bSLHold;
        }
      }
      Hex.InputTimeDelay = 128 - max(max(abs(lx - 128), abs(ly - 128)), abs(rx - 128));
    }
    Hex.BodyPosY = min(max(g_BodyYOffset + g_BodyYShift, 0), MAX_BODY_Y);
    if (Hex.bAdjustLegPositions) {
      AdjustLegPositionsToBodyHeight();
    }
  } else {
    cError++;
    if (cError == 3) {
      HexOFF();
    }
  }
}
//********************************


void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  PMWServoDriver_init();
  bluetooth_init(false);
  Initialize();
  delay(10);
  CheckAngles();
  ControlServo();
  delay(200);
  pid_init();
  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 5; i > 0; --i) {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED off by making the voltage LOW
    delay(100);                       // wait for a second
  }
}


void loop() {
  //Start time
  uint32_t lTimeWaitEnd;
  lTimerStart = millis();
  dataReady = bluetooth_prepareData(reData);
  if (dataReady) {
    CommandInput();
    dataReady = 0;
  }

  SingleLegControl();

  //Gait
  GaitSeq();

  //Balance calculations
  TotalTransX = 0;  //reset values used for calculation of balance
  TotalTransY = 0;
  TotalTransZ = 0;
  TotalXBal1 = 0;
  TotalZBal1 = 0;
  TotalYBal1 = 0;

  if (Hex.bBalanceMode) {
    for (int LegNr = 0; LegNr < (NUMBER_OF_LEGS / 2); ++LegNr) {  // balance calculations for all Right legs

      //              DoBackgroundProcess();
      BalCalcOneLeg(-LegPosX[LegNr] + GaitPosX[LegNr],
                    LegPosZ[LegNr] + GaitPosZ[LegNr],
                    (LegPosY[LegNr] - (short)pgm_read_word(&cInitPosY[LegNr])) + GaitPosY[LegNr], LegNr);
    }

    for (int LegNr = (NUMBER_OF_LEGS / 2); LegNr < NUMBER_OF_LEGS; ++LegNr) {  // balance calculations for all Right legs
      //              DoBackgroundProcess();
      BalCalcOneLeg(LegPosX[LegNr] + GaitPosX[LegNr],
                    LegPosZ[LegNr] + GaitPosZ[LegNr],
                    (LegPosY[LegNr] - (short)pgm_read_word(&cInitPosY[LegNr])) + GaitPosY[LegNr], LegNr);
    }
    BalanceBody();
  }

  //Do IK for all Right legs
  for (int LegNr = 0; LegNr < (NUMBER_OF_LEGS / 2); ++LegNr) {
    //            DoBackgroundProcess();
    BodyFK(-LegPosX[LegNr] + Hex.BodyPosX + GaitPosX[LegNr] - TotalTransX,
           LegPosZ[LegNr] + Hex.BodyPosZ + GaitPosZ[LegNr] - TotalTransZ,
           LegPosY[LegNr] + Hex.BodyPosY + GaitPosY[LegNr] - TotalTransY,
           GaitRotY[LegNr], LegNr);
    LegIK(LegPosX[LegNr] - Hex.BodyPosX + BodyFKPosX - (GaitPosX[LegNr] - TotalTransX),
          LegPosY[LegNr] + Hex.BodyPosY - BodyFKPosY + GaitPosY[LegNr] - TotalTransY,
          LegPosZ[LegNr] + Hex.BodyPosZ - BodyFKPosZ + GaitPosZ[LegNr] - TotalTransZ, LegNr);
  }

  //Do IK for all Left legs
  for (int LegNr = (NUMBER_OF_LEGS / 2); LegNr < NUMBER_OF_LEGS; ++LegNr) {
    //            DoBackgroundProcess();
    BodyFK(LegPosX[LegNr] - Hex.BodyPosX + GaitPosX[LegNr] - TotalTransX,
           LegPosZ[LegNr] + Hex.BodyPosZ + GaitPosZ[LegNr] - TotalTransZ,
           LegPosY[LegNr] + Hex.BodyPosY + GaitPosY[LegNr] - TotalTransY,
           GaitRotY[LegNr], LegNr);
    LegIK(LegPosX[LegNr] + Hex.BodyPosX - BodyFKPosX + GaitPosX[LegNr] - TotalTransX,
          LegPosY[LegNr] + Hex.BodyPosY - BodyFKPosY + GaitPosY[LegNr] - TotalTransY,
          LegPosZ[LegNr] + Hex.BodyPosZ - BodyFKPosY + GaitPosZ[LegNr] - TotalTransZ, LegNr);
  }
  //Check mechanical limits
  CheckAngles();

  if (Hex.bHexON) {
    //Drive Servos
    //Calculate Servo Move time
    if ((abs(Hex.TravelLengthX) > cTravelDeadZone) || (abs(Hex.TravelLengthZ) > cTravelDeadZone) || (abs(Hex.TravelRotationY * 2) > cTravelDeadZone)) {
      ServoMoveTime = NomGaitSpeed + (Hex.InputTimeDelay * 2) + Hex.SpeedControl;
      //Add additional delay when Balance mode is on
      if (Hex.bBalanceMode)
        ServoMoveTime = ServoMoveTime + BALANCE_DELAY;
    } else {  //Movement speed excl. Walking
      ServoMoveTime = 100 + Hex.SpeedControl;
    }

    ControlServo();
    // See if we need to sync our processor with the servo driver while walking to ensure the prev is completed
    // before sending the next one

    // Finding any incident of GaitPos/Rot <>0:
    for (int LegNr = 0; LegNr < NUMBER_OF_LEGS; ++LegNr) {
      if ((GaitPosX[LegNr] > cGPlimit) || (GaitPosX[LegNr] < -cGPlimit)
          || (GaitPosZ[LegNr] > cGPlimit) || (GaitPosZ[LegNr] < -cGPlimit)
          || (GaitRotY[LegNr] > cGPlimit) || (GaitRotY[LegNr] < -cGPlimit)) {

        bExtraCycle = NrLiftedPos + 1;  //For making sure that we are using timed move until all legs are down
        break;
      }
    }

    if (bExtraCycle > 0) {
      bExtraCycle--;
      fWalking = !(bExtraCycle == 0);

      //Get endtime and calculate wait time
      lTimeWaitEnd = lTimerStart + PrevServoMoveTime;

      do {
        // Wait the appropriate time, call any background process while waiting...
      } while (millis() < lTimeWaitEnd);
    }
  }

  PrevServoMoveTime = ServoMoveTime;
}
