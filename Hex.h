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

#include "drvPwm.h"
#include "HexCfg.h"
#include "HexInput.h"
#include "bluetooth.h"
#include "pid.h"
#include "setup.h"
#include "mqtt.h"
#include "http.h"
#include "camera.h"
#include "myWifi.h"
#include "footSensor.h"
#include "helperFunc.h"
#include "servo.h"

#ifndef BALANCE_DELAY
#define BALANCE_DELAY 50
#endif

#ifdef DEBUG_LEG
char legName[6][5] = {"RR", "RM", "RF", "LR", "LM", "LF"};
#endif

// #ifdef FOOT_SENSOR_ENABLED
// enum footSensor
// {
//   GROUND,
//   LIFTING
// };
// static byte *savedCommandData;
// #endif                             // FOOT_SENSOR_ENABLED
word ServoMoveTime;                // Time for servo updates
word PrevServoMoveTime;            // Previous time for the servo updates
unsigned long controlLegTimeStart; // Start time of the calculation cycles
// unsigned long controlLegTimeCurrent;   // End time of the calculation cycles
//  Time
unsigned long lTimerStart; // Start time of the calculation cycles
unsigned long lTimerEnd;   // End time of the calculation cycles

// Parameters for smooth function
unsigned long previousMillis = 0;
byte smoothBaseDiv = 4;
byte smoothThreshold = 4;
float timeStep = 0;

// Min / imax values
const short cCoxaMin1[] PROGMEM = {
    cRRCoxaMin1, cRMCoxaMin1, cRFCoxaMin1, cLRCoxaMin1, cLMCoxaMin1, cLFCoxaMin1};
const short cCoxaMax1[] PROGMEM = {
    cRRCoxaMax1, cRMCoxaMax1, cRFCoxaMax1, cLRCoxaMax1, cLMCoxaMax1, cLFCoxaMax1};
const short cFemurMin1[] PROGMEM = {
    cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1};
const short cFemurMax1[] PROGMEM = {
    cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1};
const short cTibiaMin1[] PROGMEM = {
    cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1};
const short cTibiaMax1[] PROGMEM = {
    cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1};

// Start positions for the leg
const short cInitPosX[] PROGMEM = {
    cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
const short cInitPosZ[] PROGMEM = {
    cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};
const short cInitPosY[] PROGMEM = {
    cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};

// Default leg angle
const short cCoxaAngle1[] PROGMEM = {
    cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1};

// Leg Lengths
const byte cCoxaLength[] PROGMEM = {
    cRRCoxaLength, cRMCoxaLength, cRFCoxaLength, cLRCoxaLength, cLMCoxaLength, cLFCoxaLength};
const byte cFemurLength[] PROGMEM = {
    cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength};
const byte cTibiaLength[] PROGMEM = {
    cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength};

// Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = {
    cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX};
const short cOffsetZ[] PROGMEM = {
    cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ};

// Angles
short coxaAngle[NUM_OF_LEGS]; // Actual Angle of the Leg
short femurAngle[NUM_OF_LEGS];
short tibiaAngle[NUM_OF_LEGS];

short coxaAngleTemp[NUM_OF_LEGS]; // Actual Angle of the Leg
short femurAngleTemp[NUM_OF_LEGS];
short tibiaAngleTemp[NUM_OF_LEGS];

// Actual positions, messured in mm from the coxa offset to the tips of the feet
short legPosX[NUM_OF_LEGS]; // Actual Position of the Leg
short legPosZ[NUM_OF_LEGS];
short legPosY[NUM_OF_LEGS];

// Body Inverse Kinematics
short PosX;
short PosZ;
short PosY;
long BodyFKPosX; // Output Position X of feet with Rotation
long BodyFKPosY; // Output Position Y of feet with Rotation
long BodyFKPosZ; // Output Position Z of feet with Rotation

// Leg Inverse Kinematics
long IKFemurAngle; // Output Angle of Femur in degrees
long IKTibiaAngle; // Output Angle of Tibia in degrees
long IKCoxaAngle;  // Output Angle of Coxa in degrees

// Time
static unsigned long previousRunTime; // previous time of the calculation runTime cycles
static unsigned long runTimeInterval; // runTime interval for servo updates

//[Balance]
long TotalTransX;
long TotalTransY;
long TotalTransZ;
long TotalXBal1;
long TotalYBal1;
long TotalZBal1;

// Single leg control
byte PrevSelectedLeg;
bool AllDown;

// Gait
short NomGaitSpeed;
byte TLDivFactor;
byte NrLiftedPos;
byte LiftDivFactor; // Nr of steps that a leg is on the floor while waling
byte FrontDownPos;
byte HalfLiftHeight; // does the step sequence has half lifted legs
byte TravelRequest;  // Temp to check if the gait is in motion
byte StepsInGait;    // Nr of steps in gait

byte LastLeg;

byte GaitLegNr[NUM_OF_LEGS];
byte GaitLegNrIn;

// Relative position corresponding to the Gait
long GaitPosX[NUM_OF_LEGS];
long GaitPosY[NUM_OF_LEGS];
long GaitPosZ[NUM_OF_LEGS];
long GaitRotY[NUM_OF_LEGS]; // Relative rotation corresponding to the Gait

static short g_BodyYShift;
static short g_BodyYOffset;

const byte BalanceDivFactor = 6;
bool fWalking;    // 1 if the robot are walking
byte bExtraCycle; // Forcing some extra timed cycles for avoiding

struct RobotStatus
{
  byte bHexON;
  bool bHexInit;
  // Body position
  long BodyPosX; // Global Input for the position of the body
  long BodyPosY; // Global Input for the position of the body
  long BodyPosZ; // Global Input for the position of the body
  long BodyRotOffsetX;
  long BodyRotOffsetY;
  long BodyRotOffsetZ;

  // Body Inverse Kinematics
  long BodyRotX; // Global Input pitch of the body
  long BodyRotY; // Global Input rotation of the body
  long BodyRotZ; // Global Input roll of the body

  byte GaitType;
  // Init position of the leg
  short LegLiftHeight;  // Current Travel height
  long TravelLengthX;   // Current Travel length X (sideways)
  long TravelLengthZ;   // Current Travel length Z (front - back)
  long TravelRotationY; // Current Travel Rotation Y

  long TipSensorTravelLengthX; // Saved Travel length X
  long TipSensorTravelLengthZ; // Saved Travel length Y
  bool bTipSensorSavedTravel;  // flat to indicate that travels are saved
  bool bFeetOnTheGround;

  // Single Leg Control
  byte SelectedLeg;
  long SLLegX;
  long SLLegY;
  long SLLegZ;
  bool bSLHold; // Single leg control mode

  // Grip
  long GripPosition;

  // Balance
  bool bBalanceMode;
  bool bBodyBalanceMode;
  // #ifdef FOOT_SENSOR_ENABLED
  //   bool bSenseGround;
  // #endif
  // Camera
  bool bCameraStart;

  // Timing
  byte InputTimeDelay;   // Delay that depends on the input to get the "sneaking" effect
  word SpeedControl;     // Adjustible Delay
  byte ForceGaitStepCnt; // new to allow us to force a step even when not moving
  byte OperationMode;

  byte bDblTravel;
  byte bDoubleHeight;
  byte bAdjustLegPositions;
  byte GaitStep; // Global Input Gait step

  // #ifdef FOOT_SENSOR_ENABLED
  //   enum footSensor legState;
  // #endif // FOOT_SENSOR_ENABLED
} Hex;

long OffsetX[NUM_OF_LEGS]; // Distance X from center of the body to the coxa
long OffsetZ[NUM_OF_LEGS]; // Distance Z from center of the body to the coxa

//***************************************************

byte readyData = 0;
const byte cTravelDeadZone = 5;

//*************************************************************************
// int _min(int a, int b) {
//   return a <= b ? a : b;
// }
// int _max(int a, int b) {
//   return a >= b ? a : b;
// }
//****************************************************************************

/**
 * @brief Select how the robot move
 *
 */
void selectGait()
{
  // Hex Gaits
  //  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground,
  //                      { RR, <RM> RF, LR, <LM>, LF}
  switch (Hex.GaitType)
  {
  case 0:
    // Tripod 8
    GaitLegNr[LR] = 5;
    GaitLegNr[RF] = 1;
    GaitLegNr[LM] = 1;
    GaitLegNr[RR] = 1;
    GaitLegNr[LF] = 5;
    GaitLegNr[RM] = 5;

    StepsInGait = 8;
    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    TLDivFactor = 4;
    HalfLiftHeight = 3;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 1:
    // Ripple Gait 12 steps (one leg at a time alt. between sides)
    GaitLegNr[LR] = 1;
    GaitLegNr[RF] = 3;
    GaitLegNr[LM] = 5;
    GaitLegNr[RR] = 7;
    GaitLegNr[LF] = 9;
    GaitLegNr[RM] = 11;

    StepsInGait = 12;
    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    TLDivFactor = 8;
    HalfLiftHeight = 3;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 2: // Triple Tripod 12 steps
    GaitLegNr[RF] = 3;
    GaitLegNr[LM] = 4;
    GaitLegNr[RR] = 5;
    GaitLegNr[LF] = 9;
    GaitLegNr[RM] = 10;
    GaitLegNr[LR] = 11;

    StepsInGait = 12;
    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    TLDivFactor = 8;
    HalfLiftHeight = 3;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  case 3: /// Triple Tripod 16 steps, use 5 lifted positions
    GaitLegNr[RF] = 4;
    GaitLegNr[LM] = 5;
    GaitLegNr[RR] = 6;
    GaitLegNr[LF] = 12;
    GaitLegNr[RM] = 13;
    GaitLegNr[LR] = 14;

    StepsInGait = 16;
    NrLiftedPos = 5;
    FrontDownPos = 3;
    LiftDivFactor = 4;
    TLDivFactor = 10;
    HalfLiftHeight = 1;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  case 4: // Wave 24 steps
    GaitLegNr[LR] = 1;
    GaitLegNr[RF] = 21;
    GaitLegNr[LM] = 5;
    GaitLegNr[RR] = 13;
    GaitLegNr[LF] = 9;
    GaitLegNr[RM] = 17;

    StepsInGait = 24;
    NrLiftedPos = 3;
    FrontDownPos = 2;
    LiftDivFactor = 2;
    TLDivFactor = 20;
    HalfLiftHeight = 3;
    NomGaitSpeed = DEFAULT_SLOW_GAIT;
    break;
  case 5: // Tripod 6 steps
    GaitLegNr[LR] = 4;
    GaitLegNr[RF] = 1;
    GaitLegNr[LM] = 1;
    GaitLegNr[RR] = 1;
    GaitLegNr[LF] = 4;
    GaitLegNr[RM] = 4;

    StepsInGait = 6;
    NrLiftedPos = 2;
    FrontDownPos = 1;
    LiftDivFactor = 2;
    TLDivFactor = 4;
    HalfLiftHeight = 1;
    NomGaitSpeed = DEFAULT_GAIT_SPEED;
    break;
  default:
    break;
  }
}

/**
 * @brief Calculate gait of the robot
 *
 * @param legNr  Leg number
 */
void calculateGait(byte legNr)
{

  if ((TravelRequest && (NrLiftedPos == 1 || NrLiftedPos == 3 || NrLiftedPos == 5) && Hex.GaitStep == GaitLegNr[legNr]) || (!TravelRequest && Hex.GaitStep == GaitLegNr[legNr] && ((abs(GaitPosX[legNr]) > 2) || (abs(GaitPosZ[legNr]) > 2) || (abs(GaitRotY[legNr]) > 2))))
  { // Up
    GaitPosX[legNr] = 0;
    GaitPosY[legNr] = -Hex.LegLiftHeight;
    GaitPosZ[legNr] = 0;
    GaitRotY[legNr] = 0;
  }
  // Optional Half height Rear (2, 3, 5 lifted positions)
  else if (((NrLiftedPos == 2 && Hex.GaitStep == GaitLegNr[legNr]) || (NrLiftedPos >= 3 && (Hex.GaitStep == GaitLegNr[legNr] - 1 || Hex.GaitStep == GaitLegNr[legNr] + (StepsInGait - 1)))) && TravelRequest)
  {
    GaitPosX[legNr] = -Hex.TravelLengthX / LiftDivFactor;
    GaitPosY[legNr] = -3 * Hex.LegLiftHeight / (3 + HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[legNr] = -Hex.TravelLengthZ / LiftDivFactor;
    GaitRotY[legNr] = -Hex.TravelRotationY / LiftDivFactor;
  }
  // _A_
  // Optional Half height front (2, 3, 5 lifted positions)
  else if ((NrLiftedPos >= 2) && (Hex.GaitStep == GaitLegNr[legNr] + 1 || Hex.GaitStep == GaitLegNr[legNr] - (StepsInGait - 1)) && TravelRequest)
  {
    GaitPosX[legNr] = Hex.TravelLengthX / LiftDivFactor;
    GaitPosY[legNr] = -3 * Hex.LegLiftHeight / (3 + HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[legNr] = Hex.TravelLengthZ / LiftDivFactor;
    GaitRotY[legNr] = Hex.TravelRotationY / LiftDivFactor;
  }

  // Optional Half height Rear 5 LiftedPos (5 lifted positions)
  else if (((NrLiftedPos == 5 && (Hex.GaitStep == GaitLegNr[legNr] - 2))) && TravelRequest)
  {
    GaitPosX[legNr] = -Hex.TravelLengthX / 2;
    GaitPosY[legNr] = -Hex.LegLiftHeight / 2;
    GaitPosZ[legNr] = -Hex.TravelLengthZ / 2;
    GaitRotY[legNr] = -Hex.TravelRotationY / 2;
  }

  // Optional Half height Front 5 LiftedPos (5 lifted positions)
  else if ((NrLiftedPos == 5) && (Hex.GaitStep == GaitLegNr[legNr] + 2 || Hex.GaitStep == GaitLegNr[legNr] - (StepsInGait - 2)) && TravelRequest)
  {
    GaitPosX[legNr] = Hex.TravelLengthX / 2;
    GaitPosY[legNr] = -Hex.LegLiftHeight / 2;
    GaitPosZ[legNr] = Hex.TravelLengthZ / 2;
    GaitRotY[legNr] = Hex.TravelRotationY / 2;
  }
  //_B_
  // Leg front down position //bug here?  From _A_ to _B_ there should only be one Hex.GaitStep, not 2!
  // For example, where is the case of Hex.GaitStep==GaitLegNr[legNr]+2 executed when NRLiftedPos=3?
  else if ((Hex.GaitStep == GaitLegNr[legNr] + FrontDownPos || Hex.GaitStep == GaitLegNr[legNr] - (StepsInGait - FrontDownPos)) && GaitPosZ[legNr] < 0)
  {
    GaitPosX[legNr] = Hex.TravelLengthX / 2;
    GaitPosZ[legNr] = Hex.TravelLengthZ / 2;
    GaitPosY[legNr] = 0;
    GaitRotY[legNr] = Hex.TravelRotationY / 2;
  }

  // Move body forward
  else
  {
    GaitPosX[legNr] = GaitPosX[legNr] - (Hex.TravelLengthX / TLDivFactor);
    GaitPosY[legNr] = 0;
    GaitPosZ[legNr] = GaitPosZ[legNr] - (Hex.TravelLengthZ / TLDivFactor);
    GaitRotY[legNr] = GaitRotY[legNr] - (Hex.TravelRotationY / TLDivFactor);
  }
}

/**
 * @brief Calculate gait sequence
 *
 */
void calculateGaitSeq(void)
{
  // Check if the Gait is in motion
  if (fWalking || (Hex.ForceGaitStepCnt != 0))
    TravelRequest = 1; // Is walking or was walking...
  else
    TravelRequest = ((abs(Hex.TravelLengthX) > cTravelDeadZone) || (abs(Hex.TravelLengthZ) > cTravelDeadZone) || (abs(Hex.TravelRotationY) > cTravelDeadZone) || (Hex.ForceGaitStepCnt != 0) || fWalking);

  // Clear values under the cTravelDeadZone
  if (!TravelRequest)
  {
    Hex.TravelLengthX = 0;
    Hex.TravelLengthZ = 0;
    Hex.TravelRotationY = 0; // Gait NOT in motion, return to home position
  }

  // Calculate Gait sequence
  LastLeg = 0;
  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {                 // for all legs
    if (legNr == 5) // last leg
      LastLeg = 1;
    calculateGait(legNr);
  } // next leg

  // Advance to the next step
  if (LastLeg)
  { // The last leg in this step
    Hex.GaitStep++;
    if (Hex.GaitStep > StepsInGait)
      Hex.GaitStep = 1;
  }

  // If we have a force count decrement it now...
  if (Hex.ForceGaitStepCnt)
    Hex.ForceGaitStepCnt--;
}

/**
 * @brief Calculate leg angle for one leg to balance the robot
 *
 * @param PosX  position X
 * @param PosZ  position Z
 * @param PosY  position Y
 * @param legNr leg number
 */
void calculateBalOneLeg(long PosX, long PosZ, long PosY, byte legNr)
{
  long CPR_X; // Final X value for centerpoint of rotation
  long CPR_Y; // Final Z value for centerpoint of rotation
  long CPR_Z; // Final Y value for centerpoint of rotation
  long *lAtan;

  // Calculating totals from center of the body to the feet
  CPR_Z = (short)pgm_read_word(&cOffsetZ[legNr]) + PosZ;
  CPR_X = (short)pgm_read_word(&cOffsetX[legNr]) + PosX;
  CPR_Y = 150 + PosY; // using the value 150 to lower the centerpoint of rotation 'Hex.BodyPosY +

  TotalTransY += (long)PosY;
  TotalTransZ += (long)CPR_Z;
  TotalTransX += (long)CPR_X;

  lAtan = helperFunc_getATan2(CPR_X, CPR_Z);
  TotalYBal1 += (lAtan[0] * 1800) / 31415;

  lAtan = helperFunc_getATan2(CPR_X, CPR_Y);
  TotalZBal1 += ((lAtan[0] * 1800) / 31415) - 900; // Rotate balance circle 90 deg

  lAtan = helperFunc_getATan2(CPR_Z, CPR_Y);
  TotalXBal1 += ((lAtan[0] * 1800) / 31415) - 900; // Rotate balance circle 90 deg
}

/**
 * @brief Balance the body of robot
 *
 */
void balanceBody(void)
{
  TotalTransZ = TotalTransZ / BalanceDivFactor;
  TotalTransX = TotalTransX / BalanceDivFactor;
  TotalTransY = TotalTransY / BalanceDivFactor;

  if (TotalYBal1 > 0) // Rotate balance circle by +/- 180 deg
    TotalYBal1 -= 1800;
  else
    TotalYBal1 += 1800;

  if (TotalZBal1 < -1800) // Compensate for extreme balance positions that causes owerflow
    TotalZBal1 += 3600;

  if (TotalXBal1 < -1800) // Compensate for extreme balance positions that causes owerflow
    TotalXBal1 += 3600;

  // Balance rotation
  TotalYBal1 = -TotalYBal1 / BalanceDivFactor;
  TotalXBal1 = -TotalXBal1 / BalanceDivFactor;
  TotalZBal1 = TotalZBal1 / BalanceDivFactor;
}

/**
 * @brief Control one leg of the robot
 *
 */
void controlSingleLeg(void)
{
  // Check if all legs are down
  if ((legPosY[RF] == (short)pgm_read_word(&cInitPosY[RF])) && (legPosY[RM] == (short)pgm_read_word(&cInitPosY[RM])) && (legPosY[RR] == (short)pgm_read_word(&cInitPosY[RR])) && (legPosY[LR] == (short)pgm_read_word(&cInitPosY[LR])) && (legPosY[LM] == (short)pgm_read_word(&cInitPosY[LM])) && (legPosY[LF] == (short)pgm_read_word(&cInitPosY[LF])))
  {
    AllDown = 1;
  }

  if (Hex.SelectedLeg < NUM_OF_LEGS)
  {
    if (Hex.SelectedLeg != PrevSelectedLeg)
    {
      if (AllDown)
      { // Lift leg a bit when it got selected
        legPosY[Hex.SelectedLeg] = (short)pgm_read_word(&cInitPosY[Hex.SelectedLeg]) - 20;

        // Store current status
        PrevSelectedLeg = Hex.SelectedLeg;
      }
      else
      { // Return prev leg back to the init position
        legPosX[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosX[PrevSelectedLeg]);
        legPosY[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosY[PrevSelectedLeg]);
        legPosZ[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosZ[PrevSelectedLeg]);
      }
    }
    else if (!Hex.bSLHold)
    {
      legPosY[Hex.SelectedLeg] = legPosY[Hex.SelectedLeg] + Hex.SLLegY;
      // legPosY[Hex.SelectedLeg] = (short)pgm_read_word(&cInitPosY[Hex.SelectedLeg]) + Hex.SLLegY;  // Using DIY remote Zenta prefer it this way
      legPosX[Hex.SelectedLeg] = (short)pgm_read_word(&cInitPosX[Hex.SelectedLeg]) + Hex.SLLegX;
      legPosZ[Hex.SelectedLeg] = (short)pgm_read_word(&cInitPosZ[Hex.SelectedLeg]) + Hex.SLLegZ;
    }
  }
  else
  { // All legs to init position
    if (!AllDown)
    {
      for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
      {
        legPosX[legNr] = (short)pgm_read_word(&cInitPosX[legNr]);
        legPosY[legNr] = (short)pgm_read_word(&cInitPosY[legNr]);
        legPosZ[legNr] = (short)pgm_read_word(&cInitPosZ[legNr]);
      }
    }
    PrevSelectedLeg = 255;
  }
}

/**
 * @brief Calculates necessary foot position (leg space) to acheive commanded body rotations, translations, and gait inputs
 *
 * @param PosX  X position of body
 * @param PosZ  Z position of body
 * @param PosY  Y position of body
 * @param RotationY Y rotaion degree of body
 * @param BodyIKLeg inverse kinetic of body
 */
void doBodyFK(short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg)
{
  short *sinCos4; // return sin4 and cos4
  short SinA4;    // Sin buffer for BodyRotX calculations
  short CosA4;    // Cos buffer for BodyRotX calculations
  short SinB4;    // Sin buffer for BodyRotZ calculations
  short CosB4;    // Cos buffer for BodyRotZ calculations
  short SinG4;    // Sin buffer for BodyRotY calculations
  short CosG4;    // Cos buffer for BodyRotY calculations
  short CPR_X;    // Total X distance between the center of the body and the feet
  short CPR_Z;    // Total Z distance between the center of the body and the feet
  short CPR_Y;    // Total Y distance between the center of the body and the feet
  float bodyBalance[3];
  float tempBodyBalance[3];
  pid_update(tempBodyBalance, 3);
#ifdef MPU_ENABLED
  if (Hex.bBodyBalanceMode)
  {
    memcpy(bodyBalance, tempBodyBalance, 3 * sizeof(tempBodyBalance[0]));
  }
#endif // MPU_ENABLED

#ifdef DEBUG_MPU
  Serial.print(F("angle BodyIK"));
  Serial.print(F("\t"));
  Serial.print(bodyBalance[0]);
  Serial.print(F("\t"));
  Serial.print(bodyBalance[1]);
  Serial.print(F("\t"));
  Serial.println(bodyBalance[2]);
#endif // DEBUG_MPU

  CPR_X = (short)pgm_read_word(&cOffsetX[BodyIKLeg]) + PosX + Hex.BodyRotOffsetX;
  CPR_Y = PosY + Hex.BodyRotOffsetY;
  CPR_Z = (short)pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ + Hex.BodyRotOffsetZ;

  sinCos4 = helperFunc_getSinCos(Hex.BodyRotX + TotalXBal1 + bodyBalance[0]);
  SinG4 = sinCos4[0];
  CosG4 = sinCos4[1];

  sinCos4 = helperFunc_getSinCos(Hex.BodyRotZ + TotalZBal1 + bodyBalance[1]);
  SinB4 = sinCos4[0];
  CosB4 = sinCos4[1];

  sinCos4 = helperFunc_getSinCos(Hex.BodyRotY + (RotationY * c1DEC) + TotalYBal1);

  SinA4 = sinCos4[0];
  CosA4 = sinCos4[1];

  BodyFKPosX = ((long)CPR_X * c2DEC - ((long)CPR_X * c2DEC * CosA4 / c4DEC * CosB4 / c4DEC - (long)CPR_Z * c2DEC * CosB4 / c4DEC * SinA4 / c4DEC + (long)CPR_Y * c2DEC * SinB4 / c4DEC)) / c2DEC;
  BodyFKPosZ = ((long)CPR_Z * c2DEC - ((long)CPR_X * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC + (long)CPR_X * c2DEC * CosA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC + (long)CPR_Z * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC - (long)CPR_Z * c2DEC * SinA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC - (long)CPR_Y * c2DEC * CosB4 / c4DEC * SinG4 / c4DEC)) / c2DEC;
  BodyFKPosY = ((long)CPR_Y * c2DEC - ((long)CPR_X * c2DEC * SinA4 / c4DEC * SinG4 / c4DEC - (long)CPR_X * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC * SinB4 / c4DEC + (long)CPR_Z * c2DEC * CosA4 / c4DEC * SinG4 / c4DEC + (long)CPR_Z * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC * SinB4 / c4DEC + (long)CPR_Y * c2DEC * CosB4 / c4DEC * CosG4 / c4DEC)) / c2DEC;
}

/**
 * @brief Calculates the angles of the coxa, femur and tibia for the given position of the feet
 *
 * @param IKFeetPosX  X postion of the leg
 * @param IKFeetPosY  Y postion of the leg
 * @param IKFeetPosZ  Z postion of the leg
 * @param legNr  leg number
 */
void doLegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte legNr)
{
  unsigned long IKSW2; // Length between Shoulder and Wrist, decimals = 2
  long *IKA14;         // Angle of the line S>W with respect to the ground in radians, decimals = 4
  unsigned long IKA24; // Angle of the line S>W with respect to the femur in radians, decimals = 4
  short IKFeetPosXZ;   // Diagonal direction from Input X and Z
  short AngleRad4;
  long *atan4;

  long Temp1;
  long Temp2;
  long T3;

  atan4 = helperFunc_getATan2(IKFeetPosX, IKFeetPosZ);
  coxaAngle[legNr] = (((long)atan4[0] * 180) / 3141) + (short)pgm_read_word(&cCoxaAngle1[legNr]);

  IKFeetPosXZ = atan4[1] / c2DEC;

  IKA14 = helperFunc_getATan2(IKFeetPosY, IKFeetPosXZ - (byte)pgm_read_byte(&cCoxaLength[legNr]));

  IKSW2 = atan4[1];

  Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[legNr]) * (byte)pgm_read_byte(&cFemurLength[legNr])) - ((long)(byte)pgm_read_byte(&cTibiaLength[legNr]) * (byte)pgm_read_byte(&cTibiaLength[legNr]))) * c4DEC + ((long)IKSW2 * IKSW2));
  Temp2 = (long)(2 * (byte)pgm_read_byte(&cFemurLength[legNr])) * c2DEC * (unsigned long)IKSW2;
  T3 = Temp1 / (Temp2 / c4DEC);
  IKA24 = helperFunc_getArcCos(T3);
  // IKFemurAngle

  femurAngle[legNr] = -(long)(IKA14[0] + IKA24) * 180 / 3141 + 900;

  Temp1 = ((((long)(byte)pgm_read_byte(&cFemurLength[legNr]) * (byte)pgm_read_byte(&cFemurLength[legNr])) + ((long)(byte)pgm_read_byte(&cTibiaLength[legNr]) * (byte)pgm_read_byte(&cTibiaLength[legNr]))) * c4DEC - ((long)IKSW2 * IKSW2));
  Temp2 = 2 * ((long)((byte)pgm_read_byte(&cFemurLength[legNr]))) * (long)((byte)pgm_read_byte(&cTibiaLength[legNr]));
  AngleRad4 = helperFunc_getArcCos(Temp1 / Temp2);

  tibiaAngle[legNr] = -(900 - (long)AngleRad4 * 180 / 3141);

  // Set the Solution quality
  //   if(IKSW2 < ((uint16_t)((byte)cFemurLength[legNr]+(byte)cTibiaLength[legNr]-30)*c2DEC))
  //     IKSolution = 1;
  //   else
  //   {
  //     if(IKSW2 < ((uint16_t)((byte)cFemurLength[legNr]+(byte)cTibiaLength[legNr])*c2DEC))
  //       IKSolutionWarning = 1;
  //     else
  //       IKSolutionError = 1    ;
  //   }
}

/**
 * @brief Checks the mechanical limits of the servos
 *
 */
void limitAngles(void)
{
  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
    coxaAngle[legNr] = _min(_max(coxaAngle[legNr], (short)pgm_read_word(&cCoxaMin1[legNr])),
                            (short)pgm_read_word(&cCoxaMax1[legNr])) +
                       (short)pgm_read_word(&CoxaAngleOffset[legNr]);
    femurAngle[legNr] = _min(_max(femurAngle[legNr], (short)pgm_read_word(&cFemurMin1[legNr])),
                             (short)pgm_read_word(&cFemurMax1[legNr])) +
                        (short)pgm_read_word(&FemurAngleOffset[legNr]);
    tibiaAngle[legNr] = _min(_max(tibiaAngle[legNr], (short)pgm_read_word(&cTibiaMin1[legNr])),
                             (short)pgm_read_word(&cTibiaMax1[legNr])) +
                        (short)pgm_read_word(&TibiaAngleOffset[legNr]);
  }
}

/**
 * @brief Initializate parameters for the robot
 *
 */
void initialize()
{
  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
    legPosX[legNr] = (short)pgm_read_word(&cInitPosX[legNr]); // Set start positions for each leg
    legPosZ[legNr] = (short)pgm_read_word(&cInitPosZ[legNr]);
    legPosY[legNr] = (short)pgm_read_word(&cInitPosY[legNr]);
  }

  // Single leg control. Make sure no leg is selected
  Hex.SelectedLeg = 255; // No Leg selected
  PrevSelectedLeg = 255;

  // Body Positions
  Hex.BodyPosX = 0;
  Hex.BodyPosZ = 0;
  Hex.BodyPosY = 0;

  // Body Rotations
  Hex.BodyRotX = 0;
  Hex.BodyRotZ = 0;
  Hex.BodyRotY = 0;

  Hex.BodyRotOffsetX = 0;
  Hex.BodyRotOffsetZ = 0; // Input Z offset value to adjust centerpoint of rotation
  Hex.BodyRotOffsetY = 0;
  Hex.TravelLengthX = 0;
  Hex.TravelLengthZ = 0;
  Hex.TravelRotationY = 0;

  Hex.GaitType = 0;
  Hex.bBalanceMode = false;
  Hex.bBodyBalanceMode = false;
  // #ifdef FOOT_SENSOR_ENABLED
  //   Hex.bSenseGround = false;
  // #endif
  Hex.bCameraStart = false;
  Hex.LegLiftHeight = 60;
  Hex.ForceGaitStepCnt = 0; // added to try to adjust starting positions depending on height...
  Hex.GaitStep = 1;
  selectGait();

  // Input controller initialization
  //    Hex.bAdjustLegPositions = false;
  Hex.OperationMode = MODE_WALK;
  Hex.GripPosition = 800;
  Hex.bDblTravel = false;
  Hex.bDoubleHeight = false;

  Hex.SpeedControl = 50;
  Hex.bFeetOnTheGround = false;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  runTimeInterval = 100;
  Hex.bHexON = false;
  Hex.bHexInit = false;
}

#ifndef MIN_XZ_LEG_ADJUST
#define MIN_XZ_LEG_ADJUST cXXCoxaLength
#endif

#ifndef MAX_XZ_LEG_ADJUST
#define MAX_XZ_LEG_ADJUST (cXXCoxaLength + cXXTibiaLength + cXXFemurLength / 4)
#endif

/**
 * @brief adjust the init leg positions to the width passed in.
 *
 * @param XZLength1   Distance from the center to the tip of the leg
 */
void adjustLegPos(word XZLength1)
{
  short *sinCos4;
  if (XZLength1 > MAX_XZ_LEG_ADJUST)
    XZLength1 = MAX_XZ_LEG_ADJUST;
  if (XZLength1 < MIN_XZ_LEG_ADJUST)
    XZLength1 = MIN_XZ_LEG_ADJUST;

  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
    sinCos4 = helperFunc_getSinCos((short)pgm_read_word(&cCoxaAngle1[legNr]));
    legPosX[legNr] = ((long)((long)sinCos4[1] * XZLength1)) / c4DEC;
    legPosZ[legNr] = -((long)((long)sinCos4[0] * XZLength1)) / c4DEC;
  }
  // Make sure we cycle through one gait to have the legs all move into their new locations...
  Hex.ForceGaitStepCnt = StepsInGait;
}

byte g_iLegInitIndex = 0x00; // remember which index we are currently using...
/**
 * @brief Adjust the position of the legs to be appropriate for the current y location of the body
 *
 */
void adjustLegPosToBodyHeight(void)
{
#ifdef CNT_HEX_INITS
  // Lets see which of our units we should use...
  // Note: We will also limit our body height here...
  if (Hex.BodyPosY > (short)pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS - 1]))
    Hex.BodyPosY = (short)pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS - 1]);

  byte i;
  word XZLength1 = pgm_read_byte(&g_abHexIntXZ[CNT_HEX_INITS - 1]);
  for (i = 0; i < (CNT_HEX_INITS - 1); i++)
  { // Don't need to look at last entry as we already init to assume this one...
    if (Hex.BodyPosY <= (short)pgm_read_byte(&g_abHexMaxBodyY[i]))
    {
      XZLength1 = pgm_read_byte(&g_abHexIntXZ[i]);
      break;
    }
  }
  if (i != g_iLegInitIndex)
  {
    g_iLegInitIndex = i; // remember the current index...
    adjustLegPos(XZLength1);
  }
#endif // CNT_HEX_INITS
}

void updateServo()
{
  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
    coxaAngleTemp[legNr] = helperFunc_smoothBasedVal(coxaAngleTemp[legNr], coxaAngle[legNr], smoothBaseDiv, smoothThreshold);
    femurAngleTemp[legNr] = helperFunc_smoothBasedVal(femurAngleTemp[legNr], femurAngle[legNr], smoothBaseDiv, smoothThreshold);
    tibiaAngleTemp[legNr] = helperFunc_smoothBasedVal(tibiaAngleTemp[legNr], tibiaAngle[legNr], smoothBaseDiv, smoothThreshold);
  }
}

/**
 * @brief Move servos to expected angle
 *
 */
void controlLegs()
{
#ifndef DEBUG_LEG
  updateServo();
  for (byte legNr = 0; legNr < NUM_LEGS_PER_SIDE; ++legNr)
  {
    servo_setAngle(legNr, COXA, -coxaAngleTemp[legNr]);
    servo_setAngle(legNr, FERMUR, femurAngleTemp[legNr]);
    servo_setAngle(legNr, TIBIA, -tibiaAngleTemp[legNr]);
  }
  for (byte legNr = NUM_LEGS_PER_SIDE; legNr < NUM_OF_LEGS; ++legNr)
  {
    servo_setAngle(legNr, COXA, coxaAngleTemp[legNr]);
    servo_setAngle(legNr, FERMUR, -femurAngleTemp[legNr]);
    servo_setAngle(legNr, TIBIA, tibiaAngleTemp[legNr]);
  }
#else  // DEBUG_LEG
  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
    Serial.print(legName[legNr][5]);
    Serial.print(F("\t"));
    switch (legNr)
    {
    case 0: // RR
    case 1: // RM
    case 2: // RF
      Serial.print(-coxaAngle[legNr]);
      Serial.print(F("\t"));
      Serial.print(femurAngle[legNr]);
      Serial.print(F("\t"));
      Serial.println(-tibiaAngle[legNr]);
      break;
    case 3: // LR
    case 4: // LM
    case 5: // LF
      Serial.print(coxaAngle[legNr]);
      Serial.print(F("\t"));
      Serial.print(-femurAngle[legNr]);
      Serial.print(F("\t"));
      Serial.println(tibiaAngle[legNr]);
      break;
    default:
      break;
    }
  }
#endif // DEBUG_LEG
}

/**
 * @brief Control hand grip
 *
 * @param angle
 */
void controlGripHand(int angle)
{
  // servo_setAngle(R_DRIVER, 15, 18, angle);
}

/**
 * @brief Turn off the robot
 *
 */
void offHex(void)
{
  // Turn off
  drvPwm_goSleep();
}

/**
 * @brief Execute input command
 *
 * @param cmd   input command string
 */
void doCommand(byte *cmd)
{
  uint16_t checksum;

  Hex.bAdjustLegPositions = false;
  short dwButton;
  static byte nMode = 0;
  static byte cError = 0;

  // Lets check the checksum...
  checksum = cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5] + cmd[6];
  checksum = (checksum >> 8) + (checksum & 0xFF);
  //	UARTprintf("crc = %x\n",checksum);
  if (cmd[0] == checksum
      // #ifdef FOOT_SENSOR_ENABLED

      // #endif
  )
  {
    cError = 0;
    dwButton = (cmd[2] << 8) | cmd[1];
    byte lx = (byte)cmd[3];
    byte ly = (byte)cmd[4];
    byte rx = (byte)cmd[5];
    byte ry = (byte)cmd[6];

#ifdef REVERSE_CMD_ENABLED
    helperFunc_reverseCmd(&lx, &ly, &rx, &ry);
#endif

    // #ifdef FOOT_SENSOR_ENABLED
    // if (footSensorTimeOut && savedCommandData != NULL && Hex.OperationMode == MODE_WALK)
    // {
    //   dwButton = 0;
    //   lx = (byte)(255 - savedCommandData[3]);
    //   ly = (byte)(255 - savedCommandData[4]);
    //   rx = (byte)(255 - savedCommandData[5]);
    //   ry = (byte)(255 - savedCommandData[6]);
    //   savedCommandData = NULL;
    //   Serial.print(F("saved data: "));
    //   Serial.print(dwButton);
    //   Serial.print(F(" "));
    //   Serial.print(lx);
    //   Serial.print(F(" "));
    //   Serial.print(ly);
    //   Serial.print(F(" "));
    //   Serial.print(rx);
    //   Serial.print(F(" "));
    //   Serial.println(ry);
    // }
    // else
    // #endif
    // {
    //   Serial.print(F("normal data: "));
    //   Serial.print(dwButton);
    //   Serial.print(F(" "));
    //   Serial.print(lx);
    //   Serial.print(F(" "));
    //   Serial.print(ly);
    //   Serial.print(F(" "));
    //   Serial.print(rx);
    //   Serial.print(F(" "));
    //   Serial.println(ry);
    // }
    // Turn ON/OFF
    if (BUTTON_PRESSED(dwButton, INPUT_HEX_TOGGLE))
    {
      if (Hex.bHexON)
      {
        offHex();
      }
      else
      {
        Hex.bHexON = true;
        Hex.bAdjustLegPositions = true;
      }
    }
    if (Hex.bHexON)
    {
      // Switch operation mode
      if (BUTTON_PRESSED(dwButton, INPUT_MODE_SELECT))
      {
        nMode += 1;
        if (nMode > CONTROL_MODE - 1)
          nMode = 0;
        switch (nMode)
        {
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
      if (BUTTON_PRESSED(dwButton, INPUT_BODY_BALANCE))
      {
        Hex.bBodyBalanceMode = !Hex.bBodyBalanceMode;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BALANCE_TOGGLE))
      {
        if (!Hex.bBalanceMode
            // #ifdef FOOT_SENSOR_ENABLED
            //         && !Hex.bSenseGround
            // #endif
        )
        {
          Hex.bBalanceMode = true;
        }
        else
        {
          Hex.bBalanceMode = false;
        }
      }
      // #ifdef FOOT_SENSOR_ENABLED
      //       if (BUTTON_PRESSED(dwButton, INPUT_TIP_SENSOR))
      //       {
      //         if (!Hex.bBalanceMode && !Hex.bSenseGround)
      //         {
      //           Hex.bSenseGround = true;
      //         }
      //         else
      //         {
      //           Hex.bSenseGround = false;
      //         }
      //       }
      // #endif
      if (BUTTON_PRESSED(dwButton, INPUT_BODY_HEIGHT_TOGGLE))
      {
        if (g_BodyYOffset > 0)
          g_BodyYOffset = 0;
        else
          g_BodyYOffset = 40;
        Hex.bAdjustLegPositions = true;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BODY_UP))
      {
        g_BodyYOffset += 10;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
        Hex.bAdjustLegPositions = true;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BODY_DOWN) && g_BodyYOffset)
      {
        g_BodyYOffset -= 10;
        if (g_BodyYOffset < 0)
        {
          g_BodyYOffset = 0;
        }
        Hex.bAdjustLegPositions = true;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_SPEED_UP))
      {
        Hex.SpeedControl -= 50;
        if (Hex.SpeedControl < 50)
          Hex.SpeedControl = 50;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_SPEED_DOWN))
      {
        Hex.SpeedControl += 50;
        // if (Hex.SpeedControl > 200)
        // {
        //   Hex.SpeedControl = 200;
        // }
      }

      if (BUTTON_PRESSED(dwButton, INPUT_GRIP_CLOSE))
      {
        Hex.GripPosition = Hex.GripPosition - 100;
        if (Hex.GripPosition < -800)
        {
          Hex.GripPosition = -800;
        }
        controlGripHand(Hex.GripPosition);
      }

      if (BUTTON_PRESSED(dwButton, INPUT_GRIP_OPEN))
      {
        Hex.GripPosition = Hex.GripPosition + 100;
        if (Hex.GripPosition > 800)
        {
          Hex.GripPosition = 800;
        }
        controlGripHand(Hex.GripPosition);
      }

      if (BUTTON_PRESSED(dwButton, INPUT_CAMERA_TOGGLE))
      {
        Hex.bCameraStart = !Hex.bCameraStart;
        if (Hex.bCameraStart && camera_isInitDone())
        {
          // camera_start();
        }
        else
        {
          // camera_stop();
        }
      }

      if (Hex.OperationMode == MODE_WALK)
      {
        // Switch gait
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL))
        {
          if (abs(Hex.TravelLengthX) < CONFIG_TRAVEL_DEAD_ZONE && abs(Hex.TravelLengthZ) < CONFIG_TRAVEL_DEAD_ZONE && abs(Hex.TravelRotationY * 2) < CONFIG_TRAVEL_DEAD_ZONE)
          {
            Hex.GaitType += 1;
            if (Hex.GaitType >= GAIT_TYPE)
              Hex.GaitType = 0;
          }
          selectGait();
        }
        // Double leg lift height
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R1))
        {
          Hex.bDoubleHeight = !Hex.bDoubleHeight;
          if (Hex.bDoubleHeight)
            Hex.LegLiftHeight = 80;
          else
            Hex.LegLiftHeight = 50;
        }
        // Double Travel Length
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2))
        {
          Hex.bDblTravel = !Hex.bDblTravel;
        }

        Hex.TravelLengthX = -(lx - 128);
        Hex.TravelLengthZ = (ly - 128);

        if (!Hex.bDblTravel)
        {
          Hex.TravelLengthX = Hex.TravelLengthX / 2;
          Hex.TravelLengthZ = Hex.TravelLengthZ / 2;
        }
        // #ifdef FOOT_SENSOR_ENABLED
        // Hex.TipSensorTravelLengthX = Hex.TravelLengthX;
        // Hex.TipSensorTravelLengthZ = Hex.TravelLengthZ;
        // Hex.bTipSensorSavedTravel = true;
        //         if (Hex.bSenseGround && Hex.bTipSensorSavedTravel)
        //         {
        //           Hex.TravelLengthX = Hex.TipSensorTravelLengthX;
        //           Hex.TravelLengthZ = Hex.TipSensorTravelLengthZ;
        //           Hex.bTipSensorSavedTravel = false;
        //         }
        // #endif
        Hex.TravelRotationY = -(rx - 128) / 4; // Right Stick Left/Right
      }

      // Translate
      // g_BodyYShift = 0;
      if (Hex.OperationMode == MODE_TRANSLATE)
      {
        Hex.BodyPosX = (lx - 128) / 2;
        Hex.BodyPosZ = -(ly - 128) / 3;
        Hex.BodyRotY = (rx - 128) * 2;
        g_BodyYShift = (-(ry - 128) / 2);
      }
      if (Hex.OperationMode == MODE_ROTATE)
      {
        Hex.BodyRotX = (ly - 128);
        Hex.BodyRotY = (rx - 128) * 2;
        Hex.BodyRotZ = (lx - 128);
        g_BodyYShift = (-(ry - 128) / 2);
      }
      if (Hex.OperationMode == MODE_SINGLE_LEG)
      {
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL))
        { // Select Button Test
          if (Hex.SelectedLeg < 5)
            Hex.SelectedLeg = Hex.SelectedLeg + 1;
          else
            Hex.SelectedLeg = 0;
        }
        Hex.SLLegX = (lx - 128) / 2;
        Hex.SLLegZ = (ly - 128) / 2;
        Hex.SLLegY = (ry - 128) / 2;

        // Hold single leg in place
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2))
        {
          Hex.bSLHold = !Hex.bSLHold;
        }
      }
      Hex.InputTimeDelay = 128 - _max(_max(abs(lx - 128), abs(ly - 128)), abs(rx - 128));
    }
    Hex.BodyPosY = _min(_max(g_BodyYOffset + g_BodyYShift, 0), MAX_BODY_Y);
    if (Hex.bAdjustLegPositions)
    {
      adjustLegPosToBodyHeight();
    }
  }
  else
  {
    cError++;
    if (cError == 3)
    {
      offHex();
    }
  }
}

/**
 * @brief Execute balance feature
 *
 */
void doBalance()
{
  if (Hex.bBalanceMode)
  {
    for (byte legNr = 0; legNr < NUM_LEGS_PER_SIDE; ++legNr)
    { // balance calculations for all Right legs

      //              DoBackgroundProcess();
      calculateBalOneLeg(-legPosX[legNr] + GaitPosX[legNr],
                         legPosZ[legNr] + GaitPosZ[legNr],
                         (legPosY[legNr] - (short)pgm_read_word(&cInitPosY[legNr])) + GaitPosY[legNr], legNr);
    }

    for (byte legNr = NUM_LEGS_PER_SIDE; legNr < NUM_OF_LEGS; ++legNr)
    { // balance calculations for all Right legs
      //              DoBackgroundProcess();
      calculateBalOneLeg(legPosX[legNr] + GaitPosX[legNr],
                         legPosZ[legNr] + GaitPosZ[legNr],
                         (legPosY[legNr] - (short)pgm_read_word(&cInitPosY[legNr])) + GaitPosY[legNr], legNr);
    }
    balanceBody();
  }
}

/**
 * @brief Execuate kinetic caculation to get servo angles
 *
 */
void doKinetic()
{
  // Do IK for all Right legs
  for (byte legNr = 0; legNr < NUM_LEGS_PER_SIDE; ++legNr)
  {
    //            DoBackgroundProcess();
    doBodyFK(-legPosX[legNr] + Hex.BodyPosX + GaitPosX[legNr] - TotalTransX,
             legPosZ[legNr] + Hex.BodyPosZ + GaitPosZ[legNr] - TotalTransZ,
             legPosY[legNr] + Hex.BodyPosY + GaitPosY[legNr] - TotalTransY,
             GaitRotY[legNr], legNr);
    doLegIK(legPosX[legNr] - Hex.BodyPosX + BodyFKPosX - (GaitPosX[legNr] - TotalTransX),
            legPosY[legNr] + Hex.BodyPosY - BodyFKPosY + GaitPosY[legNr] - TotalTransY,
            legPosZ[legNr] + Hex.BodyPosZ - BodyFKPosZ + GaitPosZ[legNr] - TotalTransZ, legNr);
  }

  // Do IK for all Left legs
  for (byte legNr = NUM_LEGS_PER_SIDE; legNr < NUM_OF_LEGS; ++legNr)
  {
    //            DoBackgroundProcess();
    doBodyFK(legPosX[legNr] - Hex.BodyPosX + GaitPosX[legNr] - TotalTransX,
             legPosZ[legNr] + Hex.BodyPosZ + GaitPosZ[legNr] - TotalTransZ,
             legPosY[legNr] + Hex.BodyPosY + GaitPosY[legNr] - TotalTransY,
             GaitRotY[legNr], legNr);
    doLegIK(legPosX[legNr] + Hex.BodyPosX - BodyFKPosX + GaitPosX[legNr] - TotalTransX,
            legPosY[legNr] + Hex.BodyPosY - BodyFKPosY + GaitPosY[legNr] - TotalTransY,
            legPosZ[legNr] + Hex.BodyPosZ - BodyFKPosZ + GaitPosZ[legNr] - TotalTransZ, legNr);
  }
}

/**
 * @brief Get the command
 *
 * @param cmd   command data
 * @return true if command data is available, false if not
 */
bool getCmd(byte *cmd)
{
  bool ret = false;
#ifdef BLUETOOTH_ENABLED
  ret = bluetooth_prepareData(cmd);
#elif defined(MQTT_ENABLED)
  mqtt_update();
  ret = mqtt_dataReady();
  if (ret)
  {
    byte *mqttData = mqtt_inputData();
    memcpy(cmd, mqttData, 7 * sizeof(byte));
  }

#elif defined(HTTP_ENABLED)
  ret = http_dataReady();
  if (ret)
  {
    byte *httpData = http_inputData();
    memcpy(cmd, httpData, 7 * sizeof(byte));
  }
#else
  ret = false;
  cmd = NULL;
#endif

#ifdef DEBUG_INPUT_DATA
  Serial.println(ret);
  Serial.print(F("inputData: "));
  for (byte i = 0; i < 7; i++)
  {
    Serial.print(cmd[i]);
    Serial.print(F(" "));
  }
  Serial.println();
#endif // DEBUG_INPUT_DATA
  return ret;
}

/**
 * @brief Arduino setup function
 *
 */
void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif

  drvPwm_init();
  delay(10);

#ifdef BLUETOOTH_ENABLED
  bluetooth_init(false);
  delay(10);
#endif // BLUETOOTH_ENABLED

#if defined(MQTT_ENABLED) || defined(HTTP_ENABLED) || defined(CAMERA_ENABLED)
  wifi_init();
  delay(10);
#endif

#ifdef MQTT_ENABLED
  mqtt_init();
  delay(10);
#endif // MQTT_ENABLED

#ifdef HTTP_ENABLED
  http_init();
  delay(10);
#endif // HTTP_ENABLED

#ifdef MPU_ENABLED
  pid_init();
  delay(10);
#endif // MPU_ENABLED

  initialize();
  limitAngles();
  controlLegs();
  delay(200);
  // after motor init to calibrate the sensor

#ifdef MPU_ENABLED
  pid_init();
  delay(10);
#endif // MPU_ENABLED

  // #ifdef FOOT_SENSOR_ENABLED
  //   footSensor_init();
  //   delay(10);
  // #endif // FOOT_SENSOR_ENABLED

  pinMode(LED_RED_GPIO, OUTPUT);
  for (byte i = 5; i > 0; --i)
  {
    digitalWrite(LED_RED_GPIO, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_RED_GPIO, LOW);  // turn the LED off by making the voltage LOW
    delay(100);                       // wait for a second
  }
}

/**
 * @brief Arduino loop function
 *
 */
void loop()
{
  byte commandData[7];
  bool cmdDataReady = getCmd(commandData);

  uint32_t lTimeWaitEnd;
  lTimerStart = millis();

  // #ifdef FOOT_SENSOR_ENABLED
  //   if (footSensorTimeOut && savedCommandData != NULL)
  //   {
  //     commandData = savedCommandData;
  //     cmdDataReady = true;
  //   }
  //   else
  //   {
  //     savedCommandData = commandData;
  //   }
  // #endif // FOOT_SENSOR_ENABLED

  if (cmdDataReady)
  {
    cmdDataReady = false;
#ifdef MQTT_ENABLED
    mqtt_setDataReady(false);
#elif defined(HTTP_ENABLED)
    http_setDataReady(false);
#endif
    doCommand(commandData);
  }
  controlSingleLeg();

  // Gait
  calculateGaitSeq();

  // Balance calculations
  TotalTransX = 0; // reset values used for calculation of balance
  TotalTransY = 0;
  TotalTransZ = 0;
  TotalXBal1 = 0;
  TotalZBal1 = 0;
  TotalYBal1 = 0;

  doBalance();
  doKinetic();

  // Check mechanical limits
  limitAngles();

#ifndef DEBUG_ON
  if (Hex.bHexON
      // || !Hex.bHexInit
  )
#else
  if (1)
#endif // DEBUG_ON
  {
    // Drive Servos
    // Calculate Servo Move time
    if ((abs(Hex.TravelLengthX) > cTravelDeadZone) || (abs(Hex.TravelLengthZ) > cTravelDeadZone) || (abs(Hex.TravelRotationY * 2) > cTravelDeadZone))
    {
      ServoMoveTime = NomGaitSpeed + (Hex.InputTimeDelay * 2) + Hex.SpeedControl;
      // Add additional delay when Balance mode is on
      if (Hex.bBalanceMode)
        ServoMoveTime = ServoMoveTime + BALANCE_DELAY;
    }
    else
    { // Movement speed excl. Walking
      ServoMoveTime = 50 + Hex.SpeedControl;
    }

    controlLegs();
    // See if we need to sync our processor with the servo driver while walking to ensure the prev is completed
    // before sending the next one

    // Finding any incident of GaitPos/Rot <>0:
    for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
    {
      if ((GaitPosX[legNr] > cGPlimit) || (GaitPosX[legNr] < -cGPlimit) || (GaitPosZ[legNr] > cGPlimit) || (GaitPosZ[legNr] < -cGPlimit) || (GaitRotY[legNr] > cGPlimit) || (GaitRotY[legNr] < -cGPlimit))
      {

        bExtraCycle = NrLiftedPos + 1; // For making sure that we are using timed move until all legs are down
        break;
      }
    }
    Hex.bFeetOnTheGround = true;
    // This code below is used to reduce the speed of hexapod, it will be fast without these lines below
    if (bExtraCycle > 0)
    {
      bExtraCycle--;
      fWalking = !(bExtraCycle == 0);

      // Get endtime and calculate wait time
      lTimeWaitEnd = lTimerStart + PrevServoMoveTime;

      do
      {
        controlLegs();
        // Wait the appropriate time, call any background process while waiting...
      } while (millis() < lTimeWaitEnd);
      // #ifdef FOOT_SENSOR_ENABLED
      //       if (footSensor_checkAllSensor())
      //       {
      //         footSensorTimeOut = false;
      //       }
      //       else
      //       {
      //         footSensorTimeOut = true;
      //       }
      // #endif // FOOT_SENSOR_ENABLED
    }
  }
  // run 1 time to setup the robot
  Hex.bHexInit = true;
  PrevServoMoveTime = ServoMoveTime;
}