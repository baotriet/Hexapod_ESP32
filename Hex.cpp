#include "Hex.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <Wire.h>

#include "HexCfg.h"
#include "HexInput.h"
#include "pid.h"
#include "setup.h"
#include "http.h"
#include "camera.h"
#include "wifey.h"
#include "footSensor.h"
#include "footSensorTest.h"
#include "footLanding.h"
#include "webControl.h"
#include <WiFi.h>

#if defined(FOOT_SENSOR_WALK_ENABLED) && !defined(FOOT_SENSOR_ENABLED)
#error "Contact-aware walking requires FOOT_SENSOR_ENABLED"
#endif
#include "helperFunc.h"
#include "servo.h"
#include "servoMotion.h"
#include "battMan.h"
#include "driver/buzzer.h"
#include "driver/ads1115.h"
#include "driver/pcf8575.h"

// Runtime state and per-leg geometry

#ifndef BALANCE_DELAY
#define BALANCE_DELAY 50
#endif

#ifdef DEBUG_LEG
char legName[6][5] = {"RF", "RM", "RR", "LF", "LM", "LR"};
#endif

#ifdef CALIBRATION_ENABLED
int servoOffsets[NUM_OF_LEGS][NUM_SERVOS_PER_LEG] = {0}; // Store offsets for each servo
#endif

word servoMoveTime;                // Time for servo updates
uint32_t lastServoChange;
unsigned long controlLegTimeStart; // Start time of the calculation cycles
// unsigned long controlLegTimeCurrent;   // End time of the calculation cycles
//  Time
unsigned long lTimerEnd;   // End time of the calculation cycles

// Parameters for smooth function
unsigned long previousMillis = 0;
byte smoothBaseDiv = 4;
byte smoothThreshold = 4;
ServoMotion servoMotion[NUM_OF_LEGS];
uint32_t servoOutputTime;
float timeStep = 0;

// Per-leg tables follow the IDs, servo channels, and sensors: RF, RM, RR, LF, LM, LR.
const short cCoxaMin1[] PROGMEM = {cRFCoxaMin1, cRMCoxaMin1, cRRCoxaMin1,
                                 cLFCoxaMin1, cLMCoxaMin1, cLRCoxaMin1};
const short cCoxaMax1[] PROGMEM = {cRFCoxaMax1, cRMCoxaMax1, cRRCoxaMax1,
                                 cLFCoxaMax1, cLMCoxaMax1, cLRCoxaMax1};
const short cFemurMin1[] PROGMEM = {cRFFemurMin1, cRMFemurMin1, cRRFemurMin1,
                                  cLFFemurMin1, cLMFemurMin1, cLRFemurMin1};
const short cFemurMax1[] PROGMEM = {cRFFemurMax1, cRMFemurMax1, cRRFemurMax1,
                                  cLFFemurMax1, cLMFemurMax1, cLRFemurMax1};
const short cTibiaMin1[] PROGMEM = {cRFTibiaMin1, cRMTibiaMin1, cRRTibiaMin1,
                                  cLFTibiaMin1, cLMTibiaMin1, cLRTibiaMin1};
const short cTibiaMax1[] PROGMEM = {cRFTibiaMax1, cRMTibiaMax1, cRRTibiaMax1,
                                  cLFTibiaMax1, cLMTibiaMax1, cLRTibiaMax1};

// Start positions for the leg
const short cInitPosX[] PROGMEM = {cRFInitPosX, cRMInitPosX, cRRInitPosX,
                                 cLFInitPosX, cLMInitPosX, cLRInitPosX};
const short cInitPosZ[] PROGMEM = {cRFInitPosZ, cRMInitPosZ, cRRInitPosZ,
                                 cLFInitPosZ, cLMInitPosZ, cLRInitPosZ};
const short cInitPosY[] PROGMEM = {cRFInitPosY, cRMInitPosY, cRRInitPosY,
                                 cLFInitPosY, cLMInitPosY, cLRInitPosY};

// Default leg angle
const short cCoxaAngle1[] PROGMEM = {cRFCoxaAngle1, cRMCoxaAngle1, cRRCoxaAngle1,
                                   cLFCoxaAngle1, cLMCoxaAngle1, cLRCoxaAngle1};

// Leg Lengths
const byte cCoxaLength[] PROGMEM = {cRFCoxaLength, cRMCoxaLength, cRRCoxaLength,
                                  cLFCoxaLength, cLMCoxaLength, cLRCoxaLength};
const byte cFemurLength[] PROGMEM = {cRFFemurLength, cRMFemurLength, cRRFemurLength,
                                   cLFFemurLength, cLMFemurLength, cLRFemurLength};
const byte cTibiaLength[] PROGMEM = {cRFTibiaLength, cRMTibiaLength, cRRTibiaLength,
                                   cLFTibiaLength, cLMTibiaLength, cLRTibiaLength};

// Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = {cRFOffsetX, cRMOffsetX, cRROffsetX,
                                cLFOffsetX, cLMOffsetX, cLROffsetX};
const short cOffsetZ[] PROGMEM = {cRFOffsetZ, cRMOffsetZ, cRROffsetZ,
                                cLFOffsetZ, cLMOffsetZ, cLROffsetZ};

// Time
static unsigned long previousRunTime; // previous time of the calculation runTime cycles
static unsigned long runTimeInterval; // runTime interval for servo updates

bool fWalking;    // 1 if the robot are walking
byte bExtraCycle; // Forcing some extra timed cycles for avoiding

typedef enum
{
  STATE_IDLE,
  STATE_STOPPED,
  STATE_GET_CMD,
  STATE_PROCESS_CMD,
  STATE_CALCULATE,
  STATE_MOVE_ALL_LEGS,
  STATE_WAIT_SYNC,
} hexState_t;

byte bHexON;

// Body position
long bodyPosX; // Global Input for the position of the body
long bodyPosY; // Global Input for the position of the body
long bodyPosZ; // Global Input for the position of the body
long bodyRotOffsetX;
long bodyRotOffsetY;
long bodyRotOffsetZ;

// Body Inverse Kinematics
long BodyRotX; // Global Input pitch of the body
long BodyRotY; // Global Input rotation of the body
long BodyRotZ; // Global Input roll of the body

// Balance
long TotalTransX;
long TotalTransY;
long TotalTransZ;
long TotalXBal1;
long TotalYBal1;
long TotalZBal1;
const byte BalanceDivFactor = 6;

// Gait
byte GaitType;
byte GaitLegNr[NUM_OF_LEGS];
byte GaitLegNrIn;
short NomGaitSpeed;
byte TLDivFactor;
byte NrLiftedPos;
byte LiftDivFactor; // Nr of steps that a leg is on the floor while waling
byte FrontDownPos;
byte HalfLiftHeight; // does the step sequence has half lifted legs
byte TravelRequest;  // Temp to check if the gait is in motion
byte StepsInGait;    // Nr of steps in gait
byte LastLeg;
// Relative position corresponding to the Gait
long GaitPosX[NUM_OF_LEGS];
long GaitPosY[NUM_OF_LEGS];
long GaitPosZ[NUM_OF_LEGS];
long GaitRotY[NUM_OF_LEGS]; // Relative rotation corresponding to the Gait

// Init position of the leg
short LegLiftHeight;  // Current Travel height
long TravelLengthX;   // Current Travel length X (sideways)
long TravelLengthZ;   // Current Travel length Z (front - back)
long TravelRotationY; // Current Travel Rotation Y

short coxaAngle[NUM_OF_LEGS]; // Actual Angle of the Leg
short femurAngle[NUM_OF_LEGS];
short tibiaAngle[NUM_OF_LEGS];
// Temp Angles
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

// Single Leg Control
byte SelectedLeg;
long SLLegX;
long SLLegY;
long SLLegZ;
// Each leg owns its hold state; selecting another leg must not inherit the previous leg's hold.
bool bSLHold[NUM_OF_LEGS];
byte PrevSelectedLeg;
// Remember each leg's actual stance, not the folded startup coordinates, for explicit returns.
long singleLegOrigin[NUM_OF_LEGS][3];
bool singleLegMoved[NUM_OF_LEGS];
byte singleLegPreviousMode = MODE_WALK;

// Grip
long GripPosition;

// Balance
bool bBalanceMode;
bool bBodyBalanceMode;

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

static hexState_t currentState;

long offsetX[NUM_OF_LEGS]; // Distance X from center of the body to the coxa
long offsetZ[NUM_OF_LEGS]; // Distance Z from center of the body to the coxa
static short g_BodyYShift;
static short g_BodyYOffset;
bool creepActive;
// Preserve nominal gait state separately from contact-corrected output so searches do not distort
// the gait.
bool walkingFootTargets;
float walkingBodyBalance[3];
bool footWalkingEnabled;
enum FootRecovery
{
  RECOVERY_IDLE,
  RECOVERY_RETRACE,
  RECOVERY_REVERSE,
  RECOVERY_SETTLE,
  RECOVERY_WAIT_CENTER,
  RECOVERY_WAIT_INPUT,
  RECOVERY_HOLD,
  // A support foot lost ground with the gait at rest: lift it and set it down again at home.
  RECOVERY_REPLACE
};
FootRecovery footRecovery;
byte replaceLeg = NUM_OF_LEGS;
bool replaceLifted;
// A failed re-place already caused one retreat; a second failure holds instead of looping.
bool replaceRetreated;
bool webArmed;
bool webStopHold;
bool webDisablePending;
long recoveryTravelX, recoveryTravelZ, recoveryRotation;
unsigned recoverySteps;
bool footFallback, footSupportWaiting;
// Joint-limit pause: last valid pose held, landing checks frozen until the operator changes input.
bool walkingPaused;
char walkingPauseReason[40];
byte footRequestedGait;
short footRequestedLift;
bool footSensorBypass, footBypassAwaitInput;
// Retain only successfully issued targets for sensor exit; a failed IK proposal must never become
// home.
long footBypassOffset[NUM_OF_LEGS];
long walkingIKX[NUM_OF_LEGS], walkingIKY[NUM_OF_LEGS], walkingIKZ[NUM_OF_LEGS];
// First unreachable leg in the current walking IK pass; NUM_OF_LEGS when all targets are reachable.
byte walkingUnreachableLeg = NUM_OF_LEGS;
bool walkingUnreachableSearch; // Reachable without the landing search depth.
long walkingSearchDepth[NUM_OF_LEGS];
long issuedX[NUM_OF_LEGS], issuedY[NUM_OF_LEGS], issuedZ[NUM_OF_LEGS], issuedRotation[NUM_OF_LEGS];
long issuedPlanY[NUM_OF_LEGS];
byte issuedStep, issuedForce;
bool walkingFrameIssued;

//***************************************************

byte readyData = 0;
const byte cTravelDeadZone = 5;

// Gait selection and foot trajectories

/**
 * @brief Select the current gait mode based on input or configuration.
 *
 * Updates internal gait parameters used by the gait controller.
 */
void selectGait()
{
  // Hex Gaits
  //  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground,
  //                      { RR, <RM> RF, LR, <LM>, LF}
  switch (GaitType)
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
 * @brief Calculate gait for a single leg index.
 *
 * Computes target positions/angles for leg `legNr` based on gait phase
 * and global movement parameters.
 *
 * @param legNr Index of the leg to calculate (0..NUM_OF_LEGS-1)
 */
void calculateGait(byte legNr)
{

  if ((TravelRequest && (NrLiftedPos == 1 || NrLiftedPos == 3 || NrLiftedPos == 5) &&
       GaitStep == GaitLegNr[legNr]) ||
      (!TravelRequest && GaitStep == GaitLegNr[legNr] &&
       ((abs(GaitPosX[legNr]) > 2) || (abs(GaitPosZ[legNr]) > 2) || (abs(GaitRotY[legNr]) > 2))))
  { // Up
    GaitPosX[legNr] = 0;
    GaitPosY[legNr] = -LegLiftHeight;
    GaitPosZ[legNr] = 0;
    GaitRotY[legNr] = 0;
  }
  // Optional Half height Rear (2, 3, 5 lifted positions)
  else if (((NrLiftedPos == 2 && GaitStep == GaitLegNr[legNr]) ||
            (NrLiftedPos >= 3 && (GaitStep == GaitLegNr[legNr] - 1 ||
                                  GaitStep == GaitLegNr[legNr] + (StepsInGait - 1)))) &&
           TravelRequest)
  {
    GaitPosX[legNr] = -TravelLengthX / LiftDivFactor;
    GaitPosY[legNr] =
        -3 * LegLiftHeight /
        (3 + HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[legNr] = -TravelLengthZ / LiftDivFactor;
    GaitRotY[legNr] = -TravelRotationY / LiftDivFactor;
  }
  // _A_
  // Two-position swings land at +1; a front half-lift there would swallow Tripod 6 touchdown.
  else if ((NrLiftedPos >= 3) &&
           (GaitStep == GaitLegNr[legNr] + 1 || GaitStep == GaitLegNr[legNr] - (StepsInGait - 1)) &&
           TravelRequest)
  {
    GaitPosX[legNr] = TravelLengthX / LiftDivFactor;
    GaitPosY[legNr] =
        -3 * LegLiftHeight /
        (3 + HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
    GaitPosZ[legNr] = TravelLengthZ / LiftDivFactor;
    GaitRotY[legNr] = TravelRotationY / LiftDivFactor;
  }

  // Optional Half height Rear 5 LiftedPos (5 lifted positions)
  else if (((NrLiftedPos == 5 && (GaitStep == GaitLegNr[legNr] - 2))) && TravelRequest)
  {
    GaitPosX[legNr] = -TravelLengthX / 2;
    GaitPosY[legNr] = -LegLiftHeight / 2;
    GaitPosZ[legNr] = -TravelLengthZ / 2;
    GaitRotY[legNr] = -TravelRotationY / 2;
  }

  // Optional Half height Front 5 LiftedPos (5 lifted positions)
  else if ((NrLiftedPos == 5) &&
           (GaitStep == GaitLegNr[legNr] + 2 || GaitStep == GaitLegNr[legNr] - (StepsInGait - 2)) &&
           TravelRequest)
  {
    GaitPosX[legNr] = TravelLengthX / 2;
    GaitPosY[legNr] = -LegLiftHeight / 2;
    GaitPosZ[legNr] = TravelLengthZ / 2;
    GaitRotY[legNr] = TravelRotationY / 2;
  }
  // Land a lifted foot regardless of travel direction; checking Z skips forward/sideways touchdown.
  else if ((GaitStep == GaitLegNr[legNr] + FrontDownPos ||
            GaitStep == GaitLegNr[legNr] - (StepsInGait - FrontDownPos)) &&
           GaitPosY[legNr] < 0)
  {
    GaitPosX[legNr] = TravelLengthX / 2;
    GaitPosZ[legNr] = TravelLengthZ / 2;
    GaitPosY[legNr] = 0;
    GaitRotY[legNr] = TravelRotationY / 2;
  }

  // Move body forward
  else
  {
    GaitPosX[legNr] = GaitPosX[legNr] - (TravelLengthX / TLDivFactor);
    GaitPosY[legNr] = 0;
    GaitPosZ[legNr] = GaitPosZ[legNr] - (TravelLengthZ / TLDivFactor);
    GaitRotY[legNr] = GaitRotY[legNr] - (TravelRotationY / TLDivFactor);
  }
}

/**
 * @brief Calculate the entire gait sequence for the robot.
 *
 * Updates internal GaitPosX/Y/Z arrays for the next cycle.
 */
void calculateGaitSeq(void)
{
  // Settling keeps fWalking true, but must not request fresh lifts after travel/stance steps end.
  TravelRequest =
      ((abs(TravelLengthX) > cTravelDeadZone) || (abs(TravelLengthZ) > cTravelDeadZone) ||
       (abs(TravelRotationY) > cTravelDeadZone) || (ForceGaitStepCnt != 0));

  // Clear values under the cTravelDeadZone
  if (!TravelRequest)
  {
    TravelLengthX = 0;
    TravelLengthZ = 0;
    TravelRotationY = 0; // Gait NOT in motion, return to home position
  }

  // Calculate Gait sequence
  LastLeg = 0;
  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {                 // for all legs
    if (legNr == 5) // last leg
    {
      LastLeg = 1;
    }
    calculateGait(legNr);
  } // next leg

  // Advance to the next step
  if (LastLeg)
  { // The last leg in this step
    GaitStep++;
    if (GaitStep > StepsInGait)
    {
      GaitStep = 1;
    }
  }

  // If we have a force count decrement it now...
  if (ForceGaitStepCnt)
  {
    ForceGaitStepCnt--;
  }
}

// Balance and inverse kinematics

/**
 * @brief Calculate balance adjustments for a single leg.
 *
 * @param PosX X offset used for balance calculations (mm)
 * @param PosZ Z offset used for balance calculations (mm)
 * @param PosY Y offset used for balance calculations (mm)
 * @param legNr Leg index to calculate
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
  CPR_Y = 150 + PosY; // using the value 150 to lower the centerpoint of rotation 'bodyPosY +

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
 * @brief Balance the robot's body by adjusting leg positions.
 *
 * Runs body-level balance algorithm using sensor inputs and leg positions.
 */
void balanceBody(void)
{
  TotalTransZ = TotalTransZ / BalanceDivFactor;
  TotalTransX = TotalTransX / BalanceDivFactor;
  TotalTransY = TotalTransY / BalanceDivFactor;

  if (TotalYBal1 > 0) // Rotate balance circle by +/- 180 deg
  {
    TotalYBal1 -= 1800;
  }
  else
  {
    TotalYBal1 += 1800;
  }

  if (TotalZBal1 < -1800) // Compensate for extreme balance positions that causes owerflow
  {
    TotalZBal1 += 3600;
  }

  if (TotalXBal1 < -1800) // Compensate for extreme balance positions that causes owerflow
  {
    TotalXBal1 += 3600;
  }

  // Balance rotation
  TotalYBal1 = -TotalYBal1 / BalanceDivFactor;
  TotalXBal1 = -TotalXBal1 / BalanceDivFactor;
  TotalZBal1 = TotalZBal1 / BalanceDivFactor;
}

/**
 * @brief Control a single leg when operating in single-leg mode.
 */
void controlSingleLeg(void)
{
  bool leaving = singleLegPreviousMode == MODE_SINGLE_LEG && OperationMode != MODE_SINGLE_LEG;
  bool changing = OperationMode == MODE_SINGLE_LEG && SelectedLeg != PrevSelectedLeg;
  if (leaving || changing)
  {
    // Return only unheld legs; other legs' saved holds survive selection and mode changes.
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      if (singleLegMoved[leg] && !bSLHold[leg])
      {
        legPosX[leg] = singleLegOrigin[leg][0];
        legPosY[leg] = singleLegOrigin[leg][1];
        legPosZ[leg] = singleLegOrigin[leg][2];
        singleLegMoved[leg] = false;
      }
    }
    // Never transfer a previous leg's joystick request to a newly selected leg.
    SLLegX = SLLegY = SLLegZ = 0;
    SelectedLeg = leaving ? 255 : SelectedLeg;
    PrevSelectedLeg = SelectedLeg;
  }
  singleLegPreviousMode = OperationMode;
  if (OperationMode == MODE_SINGLE_LEG && SelectedLeg < NUM_OF_LEGS && !bSLHold[SelectedLeg])
  {
    byte leg = SelectedLeg;
    if (!singleLegMoved[leg] && (SLLegX || SLLegY || SLLegZ))
    {
      singleLegOrigin[leg][0] = legPosX[leg];
      singleLegOrigin[leg][1] = legPosY[leg];
      singleLegOrigin[leg][2] = legPosZ[leg];
      singleLegMoved[leg] = true;
    }
    // Center means released: retain each axis independently. Repeated packets cannot accumulate Y.
    if (SLLegX)
    {
      legPosX[leg] = singleLegOrigin[leg][0] + SLLegX;
    }
    if (SLLegY)
    {
      legPosY[leg] = singleLegOrigin[leg][1] + SLLegY;
    }
    if (SLLegZ)
    {
      legPosZ[leg] = singleLegOrigin[leg][2] + SLLegZ;
    }
  }
}

/**
 * @brief Perform forward kinematics for the body to compute foot targets.
 *
 * @param PosX Body translation X
 * @param PosZ Body translation Z
 * @param PosY Body translation Y
 * @param RotationY Body rotation around Y axis
 * @param BodyIKLeg Leg index for focused IK or 255 for all
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
  // With IMU balance disabled these values still enter FK; initialize them to avoid arbitrary foot
  // targets.
  float bodyBalance[3] = {0.0f, 0.0f, 0.0f};
  float tempBodyBalance[3];
#ifdef FOOT_SENSOR_WALK_ENABLED
  if (walkingFootTargets)
  {
    // Reuse one frozen IMU correction for all feet so a contact transaction does not move the body
    // beneath them.
    memcpy(tempBodyBalance, walkingBodyBalance, sizeof(tempBodyBalance));
  }
  else
#endif
  {
    pid_update(tempBodyBalance, 3);
  }
#ifdef MPU_ENABLED
  if (bBodyBalanceMode)
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

  CPR_X = (short)pgm_read_word(&cOffsetX[BodyIKLeg]) + PosX + bodyRotOffsetX;
  CPR_Y = PosY + bodyRotOffsetY;
  CPR_Z = (short)pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ + bodyRotOffsetZ;

  sinCos4 = helperFunc_getSinCos(BodyRotX + TotalXBal1 + bodyBalance[0]);
  SinG4 = sinCos4[0];
  CosG4 = sinCos4[1];

  sinCos4 = helperFunc_getSinCos(BodyRotZ + TotalZBal1 + bodyBalance[1]);
  SinB4 = sinCos4[0];
  CosB4 = sinCos4[1];

  sinCos4 = helperFunc_getSinCos(BodyRotY + (RotationY * c1DEC) + TotalYBal1);

  SinA4 = sinCos4[0];
  CosA4 = sinCos4[1];

  BodyFKPosX = ((long)CPR_X * c2DEC - ((long)CPR_X * c2DEC * CosA4 / c4DEC * CosB4 / c4DEC -
                                       (long)CPR_Z * c2DEC * CosB4 / c4DEC * SinA4 / c4DEC +
                                       (long)CPR_Y * c2DEC * SinB4 / c4DEC)) /
               c2DEC;
  BodyFKPosZ =
      ((long)CPR_Z * c2DEC - ((long)CPR_X * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC +
                              (long)CPR_X * c2DEC * CosA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC +
                              (long)CPR_Z * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC -
                              (long)CPR_Z * c2DEC * SinA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC -
                              (long)CPR_Y * c2DEC * CosB4 / c4DEC * SinG4 / c4DEC)) /
      c2DEC;
  BodyFKPosY =
      ((long)CPR_Y * c2DEC - ((long)CPR_X * c2DEC * SinA4 / c4DEC * SinG4 / c4DEC -
                              (long)CPR_X * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC * SinB4 / c4DEC +
                              (long)CPR_Z * c2DEC * CosA4 / c4DEC * SinG4 / c4DEC +
                              (long)CPR_Z * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC * SinB4 / c4DEC +
                              (long)CPR_Y * c2DEC * CosB4 / c4DEC * CosG4 / c4DEC)) /
      c2DEC;
}

/**
 * @brief Compute inverse kinematics for a single leg.
 *
 * @param IKFeetPosX Target X position for the foot (relative to coxa)
 * @param IKFeetPosY Target Y position for the foot
 * @param IKFeetPosZ Target Z position for the foot
 * @param legNr Leg index (0..NUM_OF_LEGS-1)
 */
void doLegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte legNr)
{
#ifdef FOOT_SENSOR_WALK_ENABLED
  if (walkingFootTargets)
  {
    // Reject impossible triangles before the legacy IK math can turn an unreachable search into
    // PWM.
    walkingIKX[legNr] = IKFeetPosX;
    walkingIKY[legNr] = IKFeetPosY;
    walkingIKZ[legNr] = IKFeetPosZ;
    float horizontal = hypotf(IKFeetPosX, IKFeetPosZ);
    int femur = pgm_read_byte(&cFemurLength[legNr]);
    int tibia = pgm_read_byte(&cTibiaLength[legNr]);
    float coxa = pgm_read_byte(&cCoxaLength[legNr]);
    auto reachable = [&](long y)
    {
      float distance = hypotf(horizontal - coxa, y);
      return y > 0 && horizontal >= 1 && distance > abs(femur - tibia) && distance < femur + tibia;
    };
    if (!reachable(IKFeetPosY))
    {
      // No PWM for this target; contactWalking decides between missing-ground recovery and pause.
      if (walkingUnreachableLeg == NUM_OF_LEGS)
      {
        walkingUnreachableLeg = legNr;
        walkingUnreachableSearch =
            walkingSearchDepth[legNr] > 0 && reachable(IKFeetPosY - walkingSearchDepth[legNr]);
      }
      return;
    }
  }
#endif
  unsigned long IKSW2; // Length between Shoulder and Wrist, decimals = 2
  long *IKA14;         // Angle of the line S>W with respect to the ground in radians, decimals = 4
  unsigned long IKA24; // Angle of the line S>W with respect to the femur in radians, decimals = 4
  short IKFeetPosXZ;   // Diagonal direction from Input X and Z
  short AngleRad4;
  long *atan4;

  long Temp1;
  long Temp2;
  long T3;

  byte coxaLen = (byte)pgm_read_byte(&cCoxaLength[legNr]);
  byte femurLen = (byte)pgm_read_byte(&cFemurLength[legNr]);
  byte tibiaLen = (byte)pgm_read_byte(&cTibiaLength[legNr]);

  atan4 = helperFunc_getATan2(IKFeetPosX, IKFeetPosZ);
  coxaAngle[legNr] = (((long)atan4[0] * 180) / 3141) + (short)pgm_read_word(&cCoxaAngle1[legNr]);

  IKFeetPosXZ = atan4[1] / c2DEC;

  IKA14 = helperFunc_getATan2(IKFeetPosY, IKFeetPosXZ - coxaLen);

  IKSW2 = atan4[1];

  Temp1 =
      ((((long)femurLen * femurLen) - ((long)tibiaLen * tibiaLen)) * c4DEC + ((long)IKSW2 * IKSW2));
  Temp2 = (long)(2 * femurLen) * c2DEC * (unsigned long)IKSW2;
  T3 = Temp1 / (Temp2 / c4DEC);
  IKA24 = helperFunc_getArcCos(T3);
  // IKFemurAngle

  femurAngle[legNr] = -(long)(IKA14[0] + IKA24) * 180 / 3141 + 900;

  Temp1 =
      ((((long)femurLen * femurLen) + ((long)tibiaLen * tibiaLen)) * c4DEC - ((long)IKSW2 * IKSW2));
  Temp2 = 2 * ((long)(femurLen)) * (long)(tibiaLen);
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
 * @brief Clamp servo angles to the configured mechanical limits.
 */
void limitAngles(void)
{
  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
#ifndef CALIBRATION_ENABLED
    coxaAngle[legNr] = _min(_max(coxaAngle[legNr], (short)pgm_read_word(&cCoxaMin1[legNr])),
                            (short)pgm_read_word(&cCoxaMax1[legNr])) +
                       (short)pgm_read_word(&CoxaAngleOffset[legNr]);
    femurAngle[legNr] = _min(_max(femurAngle[legNr], (short)pgm_read_word(&cFemurMin1[legNr])),
                             (short)pgm_read_word(&cFemurMax1[legNr])) +
                        (short)pgm_read_word(&FemurAngleOffset[legNr]);
    tibiaAngle[legNr] = _min(_max(tibiaAngle[legNr], (short)pgm_read_word(&cTibiaMin1[legNr])),
                             (short)pgm_read_word(&cTibiaMax1[legNr])) +
                        (short)pgm_read_word(&TibiaAngleOffset[legNr]);
#else
    coxaAngle[legNr] = servoOffsets[legNr][COXA];
    femurAngle[legNr] = servoOffsets[legNr][FEMUR];
    tibiaAngle[legNr] = servoOffsets[legNr][TIBIA];
#endif
  }
}

/**
 * @brief Initialize leg positions, offsets, and internal state.
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
  SelectedLeg = 255; // No Leg selected
  PrevSelectedLeg = 255;

  // Body Positions
  bodyPosX = 0;
  bodyPosZ = 0;
  bodyPosY = 0;

  // Body Rotations
  BodyRotX = 0;
  BodyRotZ = 0;
  BodyRotY = 0;

  bodyRotOffsetX = 0;
  bodyRotOffsetZ = 0; // Input Z offset value to adjust centerpoint of rotation
  bodyRotOffsetY = 0;
  TravelLengthX = 0;
  TravelLengthZ = 0;
  TravelRotationY = 0;

  GaitType = 0;
  bBalanceMode = false;
  bBodyBalanceMode = false;
  bCameraStart = false;
  LegLiftHeight = DEFAULT_LEG_LIFT_MM;
  ForceGaitStepCnt = 0; // added to try to adjust starting positions depending on height...
  GaitStep = 1;
  selectGait();

  // Input controller initialization
  //    bAdjustLegPositions = false;
  OperationMode = MODE_WALK;
  GripPosition = 800;
  bDblTravel = false;
  bDoubleHeight = false;

  SpeedControl = 50;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  runTimeInterval = 100;
  bHexON = false;
}

#ifndef MIN_XZ_LEG_ADJUST
#define MIN_XZ_LEG_ADJUST cXXCoxaLength
#endif

#ifndef MAX_XZ_LEG_ADJUST
#define MAX_XZ_LEG_ADJUST (cXXCoxaLength + cXXTibiaLength + cXXFemurLength / 4)
#endif

/**
 * @brief Adjust the leg positions based on a supplied XZ length.
 *
 * @param XZLength1 Length parameter in internal units
 */
void adjustLegPos(word XZLength1)
{
  short *sinCos4;
  if (XZLength1 > MAX_XZ_LEG_ADJUST)
  {
    XZLength1 = MAX_XZ_LEG_ADJUST;
  }
  if (XZLength1 < MIN_XZ_LEG_ADJUST)
  {
    XZLength1 = MIN_XZ_LEG_ADJUST;
  }

  for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
    sinCos4 = helperFunc_getSinCos((short)pgm_read_word(&cCoxaAngle1[legNr]));
    legPosX[legNr] = ((long)((long)sinCos4[1] * XZLength1)) / c4DEC;
    legPosZ[legNr] = -((long)((long)sinCos4[0] * XZLength1)) / c4DEC;
  }
  // Make sure we cycle through one gait to have the legs all move into their new locations...
  ForceGaitStepCnt = StepsInGait;
}

/**
 * @brief Adjust legs to match the current body height setting.
 */
void adjustLegPosToBodyHeight(void)
{
#ifdef CNT_HEX_INITS
  static byte lastIndex = 255; // Apply the first requested stance even at index zero.
  // Lets see which of our units we should use...
  // Note: We will also limit our body height here...
  if (bodyPosY > MAX_BODY_Y_OFFSET)
  {
    bodyPosY = MAX_BODY_Y_OFFSET;
  }

  byte i;
  word XZLength1 = pgm_read_byte(&g_abHexIntXZ[CNT_HEX_INITS - 1]);
  for (i = 0; i < (CNT_HEX_INITS - 1); i++)
  { // Don't need to look at last entry as we already init to assume this one...
    // Stance thresholds are absolute heights; comparing only the offset selects the wrong width.
    if (CHexInitY + bodyPosY <= (short)pgm_read_byte(&g_abHexMaxBodyY[i]))
    {
      XZLength1 = pgm_read_byte(&g_abHexIntXZ[i]);
      break;
    }
  }
  if (i != lastIndex)
  {
    lastIndex = i; // remember the current index...
    adjustLegPos(XZLength1);
  }
#endif // CNT_HEX_INITS
}

void beginServoMove()
{
  uint32_t now = millis();
  uint32_t frame = (1000U + PWM_FREQ - 1) / PWM_FREQ;
  uint32_t duration = servoMoveTime > frame ? servoMoveTime - frame : 1;
  // Cubic easing peaks at 1.5 times average velocity; respect the configured nominal servo rate.
  const short *current[] = {coxaAngleTemp, femurAngleTemp, tibiaAngleTemp};
  const short *target[] = {coxaAngle, femurAngle, tibiaAngle};
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
    {
      uint32_t minimum = uint32_t(ceilf(abs(target[joint][leg] - current[joint][leg]) *
                                     0.15f * SERVO_TIME_6V * 1000 / SERVO_ANGLE_6V));
      duration = _max(duration, minimum);
    }
  }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    short from[] = {coxaAngleTemp[leg], femurAngleTemp[leg], tibiaAngleTemp[leg]};
    short to[] = {coxaAngle[leg], femurAngle[leg], tibiaAngle[leg]};
    servoMotion[leg].begin(from, to, now, duration);
  }
}

void updateServoAngle(byte legNr)
{
#ifdef FOOT_SENSOR_WALK_ENABLED
  if (walkingFootTargets && footLanding_direct(legNr))
  {
    // Remove software smoothing lag during contact placement so stopping Y also stops new downward
    // PWM.
    coxaAngleTemp[legNr] = coxaAngle[legNr];
    femurAngleTemp[legNr] = femurAngle[legNr];
    tibiaAngleTemp[legNr] = tibiaAngle[legNr];
    servoMotion[legNr].cancel();
    return;
  }
#endif
  short target[] = {coxaAngle[legNr], femurAngle[legNr], tibiaAngle[legNr]};
  short output[NUM_SERVOS_PER_LEG];
  if (servoMotion[legNr].sample(output, target, servoOutputTime))
  {
    coxaAngleTemp[legNr] = output[COXA];
    femurAngleTemp[legNr] = output[FEMUR];
    tibiaAngleTemp[legNr] = output[TIBIA];
    return;
  }
  coxaAngleTemp[legNr] = helperFunc_smoothBasedVal(coxaAngleTemp[legNr], coxaAngle[legNr],
                                                   smoothBaseDiv, smoothThreshold);
  femurAngleTemp[legNr] = helperFunc_smoothBasedVal(femurAngleTemp[legNr], femurAngle[legNr],
                                                    smoothBaseDiv, smoothThreshold);
  tibiaAngleTemp[legNr] = helperFunc_smoothBasedVal(tibiaAngleTemp[legNr], tibiaAngle[legNr],
                                                    smoothBaseDiv, smoothThreshold);
}

/**
 * @brief Update and command all servos based on computed leg positions.
 *
 * This is the main servo update routine called periodically by the control loop.
 */
void controlLegs()
{
  servoOutputTime = millis();
  // Keep the loop outside DEBUG_LEG branches so diagnostic builds still have a valid in-range leg
  // index.
  for (byte legNr = 0; legNr < NUM_OF_LEGS; legNr++)
  {
#ifndef DEBUG_LEG
    short previousCoxa = coxaAngleTemp[legNr];
    short previousFemur = femurAngleTemp[legNr];
    short previousTibia = tibiaAngleTemp[legNr];
    updateServoAngle(legNr);
    // Cache values to local variables for efficiency
    short coxa = coxaAngleTemp[legNr];
    short femur = femurAngleTemp[legNr];
    short tibia = tibiaAngleTemp[legNr];

    if (legNr < NUM_LEGS_PER_SIDE)
    {
      // Right legs: invert coxa and tibia
      servo_setAngle(legNr, COXA, -coxa);
      servo_setAngle(legNr, FEMUR, -femur);
      servo_setAngle(legNr, TIBIA, -tibia);
    }
    else
    {
      // Left legs: invert femur only
      servo_setAngle(legNr, COXA, coxa);
      servo_setAngle(legNr, FEMUR, femur);
      servo_setAngle(legNr, TIBIA, tibia);
    }
    if (coxa != previousCoxa || femur != previousFemur || tibia != previousTibia)
    {
      lastServoChange = millis();
    }

#else  // DEBUG_LEG
    // Print this leg's name, not an out-of-range array entry after the normal output loop.
    Serial.print(legName[legNr]);
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
#endif // DEBUG_LEG
  }
}

bool servoTargetsReady()
{
#ifdef DEBUG_LEG
  return true;
#else
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (coxaAngleTemp[leg] != coxaAngle[leg] || femurAngleTemp[leg] != femurAngle[leg] ||
        tibiaAngleTemp[leg] != tibiaAngle[leg])
    {
      return false;
    }
  }
  // Let the PCA9685 emit at least one complete PWM frame at the final commanded pose.
  return uint32_t(millis() - lastServoChange) >= (1000U + PWM_FREQ - 1) / PWM_FREQ;
#endif
}

/**
 * @brief Control the grip mechanism to the given angle.
 * @param angle Target grip servo angle
 */
void controlGripHand(int angle)
{
  // servo_setAngle(R_DRIVER, 15, 18, angle);
}

bool gaitAtRest()
{
  // Centered sticks alone are not settled: pending landings and displaced feet still need
  // completion.
#ifdef FOOT_SENSOR_WALK_ENABLED
  if (footLanding_busy() || footLanding_fault())
  {
    return false;
  }
#endif
  if (TravelLengthX || TravelLengthZ || TravelRotationY || fWalking || ForceGaitStepCnt)
  {
    return false;
  }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    if (abs(GaitPosX[leg]) > cGPlimit || GaitPosY[leg] || abs(GaitPosZ[leg]) > cGPlimit ||
        abs(GaitRotY[leg]) > cGPlimit)
    {
      return false;
    }
  }
  return true;
}

void updateCreepProfile()
{
  // Save normal settings once on entry so leaving the low-height profile restores the user's
  // choices.
  static byte savedGait;
  static short savedLift;
  static bool savedBalance, savedBodyBalance;
  bool requested = OperationMode == MODE_WALK && CHexInitY + g_BodyYOffset == CREEP_BODY_Y_MM;
  if (requested != creepActive)
  {
    if (requested)
    {
      savedGait = GaitType;
      savedLift = LegLiftHeight;
      savedBalance = bBalanceMode;
      savedBodyBalance = bBodyBalanceMode;
      GaitType = 0; // Tripod 8.
      GaitStep = 1;
      selectGait();
      Serial.println(F("Creep enabled: 45 mm, tripod gait, no turning/body corrections."));
    }
    else
    {
      GaitType = savedGait;
      LegLiftHeight = savedLift;
      bBalanceMode = savedBalance;
      bBodyBalanceMode = savedBodyBalance;
      GaitStep = 1;
      selectGait();
      Serial.println(F("Creep disabled; normal gait/lift/balance settings restored."));
    }
    creepActive = requested;
  }
  if (creepActive)
  {
    // Low body clearance leaves little IK margin; suppress turns/body corrections as well as large
    // strides.
    LegLiftHeight = CREEP_LEG_LIFT_MM;
    bBalanceMode = bBodyBalanceMode = false;
    bodyPosX = bodyPosZ = 0;
    BodyRotX = BodyRotY = BodyRotZ = 0;
    bodyRotOffsetX = bodyRotOffsetY = bodyRotOffsetZ = 0;
    g_BodyYShift = 0;
    TravelRotationY = 0;
  }
}

void walkingSupportGeometry(bool fromIK)
{
  // Mirror right-side leg-local X into body coordinates; prefer the last IK footprint after body
  // corrections.
  long x[NUM_OF_LEGS], z[NUM_OF_LEGS];
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    long localX = fromIK
                      ? walkingIKX[leg]
                      : legPosX[leg] + (leg < NUM_LEGS_PER_SIDE ? -GaitPosX[leg] : GaitPosX[leg]);
    x[leg] = (short)pgm_read_word(&cOffsetX[leg]) + (leg < NUM_LEGS_PER_SIDE ? -localX : localX);
    z[leg] = (short)pgm_read_word(&cOffsetZ[leg]) +
             (fromIK ? walkingIKZ[leg] : legPosZ[leg] + GaitPosZ[leg]);
  }
  footLanding_supportGeometry(x, z);
}

void walkingRestoreGait()
{
  // Fallback is temporary; call only once stationary so restoring the selected gait cannot split a
  // swing.
  if (!footFallback)
  {
    return;
  }
  GaitType = footRequestedGait;
  LegLiftHeight = footRequestedLift;
  GaitStep = 1;
  selectGait();
  footFallback = false;
  footSupportWaiting = false;
}

bool walkingEnableSensors()
{
  // Accept only a usable current support pattern; never arm later automatically after a rejected
  // request.
  if (footLanding_hardFault() || !footSensor_mappingValid())
  {
    return false;
  }
  if (!footLanding_engaged())
  {
    walkingSupportGeometry(false);
  }
  if (!footLanding_supportReady())
  {
    return false;
  }
  if (footSensorBypass)
  {
    footLanding_resume(GaitPosY, footBypassOffset);
  }
  else
  {
    footLanding_init();
  }
  footSensorBypass = footBypassAwaitInput = false;
  footWalkingEnabled = true;
  footRecovery = RECOVERY_IDLE;
  replaceRetreated = false;
  return true;
}

bool walkingCancelSensors()
{
  // Cancel sensor recovery without waiving workspace/joint/power faults or cutting holding torque.
  if (!footWalkingEnabled || footLanding_hardFault())
  {
    return false;
  }
  TravelLengthX = TravelLengthZ = TravelRotationY = 0;
  SLLegX = SLLegY = SLLegZ = 0;
  if (walkingFrameIssued)
  {
    // Restore the last output anchor, not the most recent proposed step, which may have failed
    // validation.
    GaitStep = issuedStep;
    ForceGaitStepCnt = issuedForce;
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      GaitPosX[leg] = issuedX[leg];
      GaitPosY[leg] = issuedPlanY[leg];
      GaitPosZ[leg] = issuedZ[leg];
      GaitRotY[leg] = issuedRotation[leg];
      footBypassOffset[leg] = issuedY[leg] - issuedPlanY[leg];
    }
  }
  else
  {
    memset(footBypassOffset, 0, sizeof(footBypassOffset));
  }
  footWalkingEnabled = false;
  footLanding_init();
  footRecovery = RECOVERY_IDLE;
  replaceRetreated = false;
  recoveryTravelX = recoveryTravelZ = recoveryRotation = 0;
  recoverySteps = 0;
  footSupportWaiting = false;
  walkingPaused = false;
  walkingPauseReason[0] = 0;
  footSensorBypass = footBypassAwaitInput = true;
  // Re-enable alone must not move a stranded leg; require deliberate fresh input in the guarded
  // WALK path.
  OperationMode = MODE_WALK;
  bHexON = webArmed = webDisablePending = false;
  webStopHold = true;
  Serial.println(F(
      "Foot sensor disabled; pose retained. Explicit Control enable and fresh movement required."));
  return true;
}

bool walkingPlanStep()
{
  // Preview every proposed lift and roll back all scheduler state if the remaining feet cannot
  // support it.
  long x[NUM_OF_LEGS], y[NUM_OF_LEGS], z[NUM_OF_LEGS], rotation[NUM_OF_LEGS];
  memcpy(x, GaitPosX, sizeof(x));
  memcpy(y, GaitPosY, sizeof(y));
  memcpy(z, GaitPosZ, sizeof(z));
  memcpy(rotation, GaitRotY, sizeof(rotation));
  byte step = GaitStep, forced = ForceGaitStepCnt;
  byte requested = TravelRequest, last = LastLeg;
  calculateGaitSeq();
  if (footSensorBypass || footLanding_liftReady(GaitPosY))
  {
    footSupportWaiting = false;
    return true;
  }
  memcpy(GaitPosX, x, sizeof(x));
  memcpy(GaitPosY, y, sizeof(y));
  memcpy(GaitPosZ, z, sizeof(z));
  memcpy(GaitRotY, rotation, sizeof(rotation));
  GaitStep = step;
  ForceGaitStepCnt = forced;
  TravelRequest = requested;
  LastLeg = last;
  bool airborne = false;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    airborne |= y[leg] < 0;
  }
  if (airborne)
  {
    // Finish the current group before changing gait or lifting another group.
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      if (y[leg] < 0)
      {
        GaitPosY[leg] = 0;
      }
    }
    footLanding_plan(y, GaitPosY, servoMoveTime);
  }
  else if (!footFallback)
  {
    // Switch only between swings; one-foot wave lifts can retain support that a tripod lift would
    // remove.
    footRequestedGait = GaitType;
    footRequestedLift = LegLiftHeight;
    footFallback = true;
    GaitType = 4; // Wave 24: one swing leg at a time.
    LegLiftHeight = _min(LegLiftHeight, FALLBACK_LEG_LIFT_MM);
    GaitStep = 1;
    selectGait();
    // A foot that lost ground (table edge) swings first: the remaining feet still support it,
    // while other lifts may not until it is back on the surface.
    byte lost = footLanding_lostMask();
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      if (lost & (1 << leg))
      {
        GaitStep = GaitLegNr[leg] > 1 ? GaitLegNr[leg] - 1 : StepsInGait;
        break;
      }
    }
    Serial.println(F("Support-aware fallback: slow wave, small translation, no turning."));
  }
  if (!footSupportWaiting)
  {
    Serial.println(
        F("Waiting for supporting feet; no new lift. Missing support feet will search down."));
  }
  footSupportWaiting = true;
  return false;
}

void walkingStartReplace()
{
  byte lost = footLanding_lostMask();
  replaceLeg = NUM_OF_LEGS;
  for (byte leg = 0; leg < NUM_OF_LEGS && replaceLeg == NUM_OF_LEGS; ++leg)
  {
    if (lost & (1 << leg))
    {
      replaceLeg = leg;
    }
  }
  replaceLifted = false;
  footRecovery = RECOVERY_REPLACE;
  TravelLengthX = TravelLengthZ = TravelRotationY = 0;
  if (replaceLeg < NUM_OF_LEGS)
  {
    Serial.println(F("Recovery: lifting the lost foot and setting it down again."));
  }
}

/**
 * @brief Process an incoming command packet and act on it.
 *
 * @param cmd Pointer to the 7-byte command buffer
 */
void doCommand(byte *cmd)
{
  uint16_t checksum;

  bAdjustLegPositions = false;
  short dwButton;
  static byte nMode = 0;
  static byte cError = 0;

  // Lets check the checksum...
  checksum = cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5] + cmd[6];
  checksum = (checksum >> 8) + (checksum & 0xFF);
  //	UARTprintf("crc = %x\n",checksum);
  if (cmd[0] == checksum)
  {
    cError = 0;
    dwButton = (cmd[2] << 8) | cmd[1];
    byte lx = (byte)cmd[3];
    byte ly = (byte)cmd[4];
    byte rx = (byte)cmd[5];
    byte ry = (byte)cmd[6];

    bool centered = abs(int(lx) - 128) <= CONFIG_TRAVEL_DEAD_ZONE &&
                    abs(int(ly) - 128) <= CONFIG_TRAVEL_DEAD_ZONE &&
                    abs(int(rx) - 128) <= CONFIG_TRAVEL_DEAD_ZONE &&
                    abs(int(ry) - 128) <= CONFIG_TRAVEL_DEAD_ZONE;
#ifdef FOOT_SENSOR_WALK_ENABLED
    static bool previousSensorButton = false;
    bool sensorButton = BUTTON_PRESSED(dwButton, INPUT_TIP_SENSOR);
    // App packets repeat held bits; activate/cancel once per new press rather than oscillating
    // every packet.
    bool sensorEdge = sensorButton && !previousSensorButton;
    previousSensorButton = sensorButton;
    if (sensorEdge)
    {
      if (centered && footWalkingEnabled)
      {
        if (!walkingCancelSensors())
        {
          Serial.println(F("Foot sensor exit rejected: battery/geometry fault remains latched."));
        }
        return;
      }
      if (!centered || !gaitAtRest() ||
          (footRecovery != RECOVERY_IDLE && footRecovery != RECOVERY_WAIT_INPUT))
      {
        Serial.println(
            F("Foot sensor toggle rejected: stop and settle first; recovery cannot be disabled."));
      }
      else if (!footWalkingEnabled &&
               (SelectedLeg < NUM_OF_LEGS || OperationMode != MODE_WALK ||
                CHexInitY + bodyPosY <= CHexInitY || !walkingEnableSensors()))
      {
        Serial.println(F("Foot sensor toggle rejected: check support geometry, mapping, standing "
                         "WALK and no selected leg."));
      }
      else
      {
        Serial.println(footWalkingEnabled ? F("Contact-aware walking enabled.")
                                          : F("Contact-aware walking disabled."));
      }
    }
    if (footWalkingEnabled && footRecovery != RECOVERY_IDLE)
    {
      // A held joystick must not resume driving at the end of autonomous retreat.
      if (footRecovery == RECOVERY_WAIT_CENTER && centered)
      {
        footRecovery = RECOVERY_WAIT_INPUT;
        Serial.println(F("Recovery complete. Fresh joystick movement required."));
      }
      else if (footRecovery == RECOVERY_WAIT_INPUT && !centered)
      {
        footRecovery = RECOVERY_IDLE;
      }
      else
      {
        return;
      }
    }
    if (footWalkingEnabled && (footLanding_fault() || footLanding_busy()))
    {
      if (centered)
      {
        TravelLengthX = TravelLengthZ = TravelRotationY = 0;
      }
      return; // Do not alter body, gait, or mode beneath a pending landing.
    }
#endif
    if (footSensorBypass)
    {
      // Mode/height changes would replace the retained pose before contact sensing has been
      // restored.
      if (dwButton & ~(INPUT_HEX_TOGGLE | INPUT_SPEED_UP | INPUT_SPEED_DOWN))
      {
        Serial.println(F("Pose-preserving sensor bypass: only WALK and speed changes allowed until "
                         "sensors are re-enabled."));
      }
      dwButton &= INPUT_HEX_TOGGLE | INPUT_SPEED_UP | INPUT_SPEED_DOWN;
    }
    if (!gaitAtRest() &&
        BUTTON_PRESSED(dwButton, INPUT_BODY_HEIGHT_TOGGLE | INPUT_BODY_UP | INPUT_BODY_DOWN))
    {
      Serial.println(F("Height change rejected: center sticks and wait for the gait to settle."));
    }
    if (creepActive && !gaitAtRest() && BUTTON_PRESSED(dwButton, INPUT_MODE_SELECT))
    {
      Serial.println(F("Mode change rejected: stop creeping and let the gait settle first."));
    }

#ifdef REVERSE_CMD_ENABLED
    helperFunc_reverseCmd(&lx, &ly, &rx, &ry);
#endif

    // Turn ON/OFF
    if (BUTTON_PRESSED(dwButton, INPUT_HEX_TOGGLE))
    {
      if (bHexON)
      {
        // offHex();
      }
      else
      {
        if (footSensorBypass && (!centered || footLanding_hardFault()))
        {
          Serial.println(
              F("Sensor bypass enable rejected: center inputs and resolve hard faults."));
          return;
        }
        bHexON = true;
        bAdjustLegPositions = !footSensorBypass;
      }
    }
    if (bHexON)
    {
      if (footSensorBypass)
      {
        webStopHold = false;
        if (!centered)
        {
          footBypassAwaitInput = false;
        }
      }
      // Switch operation mode
      if (BUTTON_PRESSED(dwButton, INPUT_MODE_SELECT) && ((!creepActive
#ifdef FOOT_SENSOR_WALK_ENABLED
                                                           && !footLanding_engaged()
#endif
                                                               ) ||
                                                          gaitAtRest()))
      {
        nMode += 1;
        if (nMode > CONTROL_MODE - 1)
        {
          nMode = 0;
        }
        switch (nMode)
        {
        case 0:
          OperationMode = MODE_WALK;
          break;
        case 1:
          OperationMode = MODE_TRANSLATE;
          break;
        case 2:
          OperationMode = MODE_ROTATE;
          break;
        case 3:
          OperationMode = MODE_SINGLE_LEG;
          break;
        default:
          break;
        }
      }
      if (BUTTON_PRESSED(dwButton, INPUT_BODY_BALANCE) && !creepActive)
      {
        bBodyBalanceMode = !bBodyBalanceMode;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BALANCE_TOGGLE) && !creepActive)
      {
        if (!bBalanceMode)
        {
          bBalanceMode = true;
        }
        else
        {
          bBalanceMode = false;
        }
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BODY_HEIGHT_TOGGLE) && gaitAtRest())
      {
        static byte curCnt;
        // The old XZ-width table is not a height table; presets must become offsets from folded
        // home.
        g_BodyYOffset = (short)pgm_read_byte(&g_abHexMaxBodyY[curCnt]) - CHexInitY;
        curCnt++;
        if (curCnt >= CNT_HEX_INITS)
        {
          curCnt = 0;
        }
        bAdjustLegPositions = true;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BODY_UP) && gaitAtRest())
      {
        g_BodyYOffset += 10;
        if (g_BodyYOffset > MAX_BODY_Y_OFFSET)
        {
          g_BodyYOffset = MAX_BODY_Y_OFFSET;
        }
        bAdjustLegPositions = true;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_BODY_DOWN) && g_BodyYOffset && gaitAtRest())
      {
        g_BodyYOffset -= 10;
        if (g_BodyYOffset < 0)
        {
          g_BodyYOffset = 0;
        }
        bAdjustLegPositions = true;
      }

      if (BUTTON_PRESSED(dwButton, INPUT_SPEED_UP))
      {
        SpeedControl -= 10;
        if (SpeedControl <= 0)
        {
          SpeedControl = 0;
        }
      }

      if (BUTTON_PRESSED(dwButton, INPUT_SPEED_DOWN))
      {
        SpeedControl += 10;
        if (SpeedControl >= 100)
        {
          SpeedControl = 100;
        }
      }

      if (BUTTON_PRESSED(dwButton, INPUT_GRIP_CLOSE))
      {
        GripPosition = GripPosition - 100;
        if (GripPosition < -800)
        {
          GripPosition = -800;
        }
        controlGripHand(GripPosition);
      }

      if (BUTTON_PRESSED(dwButton, INPUT_GRIP_OPEN))
      {
        GripPosition = GripPosition + 100;
        if (GripPosition > 800)
        {
          GripPosition = 800;
        }
        controlGripHand(GripPosition);
      }

      if (BUTTON_PRESSED(dwButton, INPUT_CAMERA_TOGGLE))
      {
        bCameraStart = !bCameraStart;
        if (bCameraStart && camera_isInitDone())
        {
          // camera_start();
        }
        else
        {
          // camera_stop();
        }
      }

      updateCreepProfile();
      if (OperationMode == MODE_WALK)
      {
        // Switch gait
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL) && !creepActive)
        {
          if (abs(TravelLengthX) < CONFIG_TRAVEL_DEAD_ZONE &&
              abs(TravelLengthZ) < CONFIG_TRAVEL_DEAD_ZONE &&
              abs(TravelRotationY * 2) < CONFIG_TRAVEL_DEAD_ZONE)
          {
            GaitType += 1;
            if (GaitType >= GAIT_TYPE)
            {
              GaitType = 0;
            }
          }
          selectGait();
        }
        // Double leg lift height
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R1) && !creepActive)
        {
          bDoubleHeight = !bDoubleHeight;
          if (bDoubleHeight)
          {
            LegLiftHeight = HIGH_LEG_LIFT_MM;
          }
          else
          {
            LegLiftHeight = DEFAULT_LEG_LIFT_MM;
          }
        }
        // Double Travel Length
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2) && !creepActive)
        {
          bDblTravel = !bDblTravel;
        }

        TravelLengthX = -(lx - 128);
        TravelLengthZ = (ly - 128);

        if (!bDblTravel)
        {
          TravelLengthX = TravelLengthX / 2;
          TravelLengthZ = TravelLengthZ / 2;
        }
        TravelRotationY = -(rx - 128) / 4; // Right Stick Left/Right
        if (creepActive)
        {
          // Scale the app axes into the checked small-translation envelope, independent of double
          // travel.
          TravelLengthX = _min(_max(-(int(lx) - 128) / 8, -CREEP_TRAVEL_MM), CREEP_TRAVEL_MM);
          TravelLengthZ = _min(_max((int(ly) - 128) / 8, -CREEP_TRAVEL_MM), CREEP_TRAVEL_MM);
          TravelRotationY = 0;
        }
      }

      // Translate
      // g_BodyYShift = 0;
      if (OperationMode == MODE_TRANSLATE)
      {
        bodyPosX = (lx - 128) / 2;
        bodyPosZ = -(ly - 128) / 3;
        BodyRotY = (rx - 128) * 2;
        g_BodyYShift = (-(ry - 128) / 2);
      }
      if (OperationMode == MODE_ROTATE)
      {
        BodyRotX = (ly - 128);
        BodyRotY = (rx - 128) * 2;
        BodyRotZ = (lx - 128);
        g_BodyYShift = (-(ry - 128) / 2);
      }
      if (OperationMode == MODE_SINGLE_LEG)
      {
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_SEL))
        { // Select Button Test
          if (SelectedLeg < 5)
          {
            SelectedLeg = SelectedLeg + 1;
          }
          else
          {
            SelectedLeg = 0;
          }
        }
        SLLegX = (lx - 128) / 2;
        SLLegZ = (ly - 128) / 2;
        SLLegY = (ry - 128) / 2;

        // Hold single leg in place
        if (BUTTON_PRESSED(dwButton, INPUT_OPT_R2) && SelectedLeg < NUM_OF_LEGS)
        {
          bSLHold[SelectedLeg] = !bSLHold[SelectedLeg];
        }
      }
      InputTimeDelay = 128 - _max(_max(abs(lx - 128), abs(ly - 128)), abs(rx - 128));
    }
    bodyPosY = _min(_max(g_BodyYOffset + g_BodyYShift, 0), MAX_BODY_Y_OFFSET);
    if (bAdjustLegPositions)
    {
      adjustLegPosToBodyHeight();
    }
  }
  else
  {
    cError++;
    if (cError == 3)
    {
      // offHex();
    }
  }
}

/**
 * @brief Perform the balance control loop step.
 */
void doBalance()
{
  if (bBalanceMode)
  {
    for (byte legNr = 0; legNr < NUM_LEGS_PER_SIDE; ++legNr)
    {
      // balance calculations for all Right legs
      calculateBalOneLeg(
          -legPosX[legNr] + GaitPosX[legNr], legPosZ[legNr] + GaitPosZ[legNr],
          (legPosY[legNr] - (short)pgm_read_word(&cInitPosY[legNr])) + GaitPosY[legNr], legNr);
    }

    for (byte legNr = NUM_LEGS_PER_SIDE; legNr < NUM_OF_LEGS; ++legNr)
    {
      // balance calculations for all Right legs
      calculateBalOneLeg(
          legPosX[legNr] + GaitPosX[legNr], legPosZ[legNr] + GaitPosZ[legNr],
          (legPosY[legNr] - (short)pgm_read_word(&cInitPosY[legNr])) + GaitPosY[legNr], legNr);
    }
    balanceBody();
  }
}

/**
 * @brief Perform kinetic calculations for gait dynamics.
 */
void doKinetic()
{
  // Do IK for all Right legs
  for (byte legNr = 0; legNr < NUM_LEGS_PER_SIDE; ++legNr)
  {
    doBodyFK(-legPosX[legNr] + bodyPosX + GaitPosX[legNr] - TotalTransX,
             legPosZ[legNr] + bodyPosZ + GaitPosZ[legNr] - TotalTransZ,
             legPosY[legNr] + bodyPosY + GaitPosY[legNr] - TotalTransY, GaitRotY[legNr], legNr);
    doLegIK(legPosX[legNr] - bodyPosX + BodyFKPosX - (GaitPosX[legNr] - TotalTransX),
            legPosY[legNr] + bodyPosY - BodyFKPosY + GaitPosY[legNr] - TotalTransY,
            legPosZ[legNr] + bodyPosZ - BodyFKPosZ + GaitPosZ[legNr] - TotalTransZ, legNr);
  }

  // Do IK for all Left legs
  for (byte legNr = NUM_LEGS_PER_SIDE; legNr < NUM_OF_LEGS; ++legNr)
  {
    doBodyFK(legPosX[legNr] - bodyPosX + GaitPosX[legNr] - TotalTransX,
             legPosZ[legNr] + bodyPosZ + GaitPosZ[legNr] - TotalTransZ,
             legPosY[legNr] + bodyPosY + GaitPosY[legNr] - TotalTransY, GaitRotY[legNr], legNr);
    doLegIK(legPosX[legNr] + bodyPosX - BodyFKPosX + GaitPosX[legNr] - TotalTransX,
            legPosY[legNr] + bodyPosY - BodyFKPosY + GaitPosY[legNr] - TotalTransY,
            legPosZ[legNr] + bodyPosZ - BodyFKPosZ + GaitPosZ[legNr] - TotalTransZ, legNr);
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
#if defined(HTTP_ENABLED) &&                                                                       \
    (defined(LEGACY_HTTP_COMMAND_ENABLED) || !defined(WEB_CONTROLLER_ENABLED))
  ret = http_takeInput(cmd);
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

#ifdef CALIBRATION_ENABLED
/**
 * @brief Print configured servo offsets over Serial.
 */
void printAllServoOffsets()
{
  Serial.println("Current Servo Offsets:");
  const char *labels[] = {"Coxa", "Femur", "Tibia"};
  for (int i = 0; i < NUM_OF_SERVOS; i++)
  {
    int legIndex = i / 3;
    int jointIndex = i % 3;
    Serial.print("Leg ");
    Serial.print(legIndex);
    Serial.print(" (");
    Serial.print(labels[jointIndex]);
    Serial.print(") offset: ");
    Serial.println(servoOffsets[legIndex][jointIndex]);
  }
}

/**
 * @brief Serial-based servo calibration helper (reads commands from Serial).
 */
void calibrationServo(void)
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("PRINT"))
    {
      printAllServoOffsets();
    }
    else
    {
      int separatorIndex = command.indexOf('|');
      if (separatorIndex != -1)
      {
        int servoNum = command.substring(0, separatorIndex).toInt();
        int offset = command.substring(separatorIndex + 1).toInt();
        int legNr = servoNum / 3;
        int joint = servoNum % 3;

        if (servoNum >= 0 && servoNum < NUM_OF_SERVOS && offset >= -900 && offset <= 900)
        {
          servoOffsets[legNr][joint] = offset;
        }
        else
        {
          Serial.println(
              "Invalid input! Use 'servoID|servoAngle' format, or 'PRINT' to display offsets.");
        }
      }
      else
      {
        Serial.println("Invalid format! Use 'servoID|servoAngle' or 'PRINT'.");
      }
    }
  }
}
#endif

#ifdef FOOT_SENSOR_TEST_ENABLED
bool footTestMove(byte leg, short y)
{
  // Validate the isolated test independently of walking height; never clamp an invalid target into
  // motion.
  short x = (short)pgm_read_word(&cInitPosX[leg]);
  short z = (short)pgm_read_word(&cInitPosZ[leg]);
  float horizontal = sqrtf(float(x) * x + float(z) * z);
  float radial = horizontal - pgm_read_byte(&cCoxaLength[leg]);
  float distance = sqrtf(radial * radial + float(y) * y);
  int femur = pgm_read_byte(&cFemurLength[leg]);
  int tibia = pgm_read_byte(&cTibiaLength[leg]);
  if (horizontal < 1 || distance <= abs(femur - tibia) || distance >= femur + tibia)
  {
    Serial.println(F("Foot test: unreachable target; holding previous command."));
    return false;
  }
  doLegIK(x, y, z, leg);
  short angles[] = {coxaAngle[leg], femurAngle[leg], tibiaAngle[leg]};
  const short *mins[] = {cCoxaMin1, cFemurMin1, cTibiaMin1};
  const short *maxs[] = {cCoxaMax1, cFemurMax1, cTibiaMax1};
  const short *offsets[] = {CoxaAngleOffset, FemurAngleOffset, TibiaAngleOffset};
  for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
  {
    int angle = angles[joint];
    int corrected = angle + (short)pgm_read_word(&offsets[joint][leg]);
    if (angle < (short)pgm_read_word(&mins[joint][leg]) ||
        angle > (short)pgm_read_word(&maxs[joint][leg]) || corrected < -900 || corrected > 900)
    {
      Serial.println(F("Foot test: joint limit exceeded; holding previous command."));
      return false;
    }
    angles[joint] = corrected;
  }
  // Small Cartesian test steps bypass smoothing so contact cannot leave a queued descent.
  for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
  {
    servo_setAngle(leg, joint, leg < NUM_LEGS_PER_SIDE ? -angles[joint] : angles[joint]);
  }
  return true;
}

#endif

/**
 * @brief Arduino setup() function that initializes subsystems.
 */
void setup()
{
#if defined(DEBUG) || defined(CALIBRATION_ENABLED) || defined(FOOT_SENSOR_ENABLED) ||              \
    defined(HTTP_ENABLED)
  Serial.begin(115200);
#endif
#ifdef DEBUG
  Serial.println("START: setup");
#endif
  Wire.begin(SDA_PIN, SCL_PIN);

#if !defined(CALIBRATION_ENABLED) && !defined(FOOT_SENSOR_TEST_ENABLED)
  // Isolated testing must not start networking, IMU control, or unrelated motion-producing
  // services.
#if defined(HTTP_ENABLED) || defined(CAMERA_ENABLED)
  wifi_init();
  delay(10);
#endif

#ifdef HTTP_ENABLED
  http_init();
  delay(10);
#endif // HTTP_ENABLED

#ifdef MPU_ENABLED
  pid_init();
  delay(10);
#endif // MPU_ENABLED
#endif // CALIBRATION_ENABLED
  servo_init();
  delay(10);

#ifndef FOOT_SENSOR_TEST_ENABLED
  // The supported-foot experiment is intentionally independent of battery sensing and ADS startup.
  ads1115_init();
  delay(10);
#endif

  pcf8575_init();
  delay(10);

#ifdef FOOT_SENSOR_ENABLED
  footSensor_init();
#endif
  footLanding_init();
  footWalkingEnabled = false;
  footRecovery = RECOVERY_IDLE;
  initialize();
#ifdef FOOT_SENSOR_TEST_ENABLED
  // Skip normal all-leg startup positioning; the test moves only after an explicit POSITION
  // command.
  footSensorTest_init(footTestMove);
#else
  limitAngles();
  for (int i = 500; i > 0; i--)
  {
    controlLegs();
  }
  delay(200);
  //  after motor init to calibrate the sensor

  currentState = STATE_IDLE;
  buzzer_toggle(3, 200, 100);
#endif
#ifdef DEBUG
  Serial.println("DONE: setup");
#endif
}

/**
 * @brief Arduino main loop(), called repeatedly after setup().
 */
#ifdef FOOT_SENSOR_WALK_ENABLED
void contactWalking()
{
  // Separate timed output/step updates replace the servo busy-wait so HTTP and switches stay
  // serviceable.
  static uint32_t lastStep = 0;
  static uint32_t stepDuration = 0;
  static uint32_t lastOutput = 0;
  static uint32_t lastPowerCheck = 0;
  static bool powerSampled = false;
  static bool powerOk = false;
  static long stepStartX[NUM_OF_LEGS], stepStartZ[NUM_OF_LEGS];
  static long stepStartRotation[NUM_OF_LEGS];
  uint32_t now = millis();
  if (!bHexON)
  {
    return;
  }
  if (!powerSampled || uint32_t(now - lastPowerCheck) >= 100)
  {
    // Rate-limit synchronous ADC work without dropping the power check from walking/bypass motion.
    powerSampled = true;
    lastPowerCheck = now;
    powerOk = battMan_isPowerOk();
  }
  footSensor_update();
  if (!powerOk)
  {
    footLanding_fail("battery check failed", NUM_OF_LEGS, FOOT_FAULT_POWER);
    return;
  }
  if (footSensorBypass && footBypassAwaitInput)
  {
    // Preserve the held pose through centered re-enable packets until the operator deliberately
    // moves.
    return;
  }
  if (SelectedLeg < NUM_OF_LEGS)
  {
    footLanding_fail("deselect single leg before walking", SelectedLeg);
    return;
  }
  if (footRecovery == RECOVERY_HOLD || footRecovery == RECOVERY_WAIT_CENTER ||
      footRecovery == RECOVERY_WAIT_INPUT)
  {
    return;
  }
  if (footLanding_fault())
  {
    // Retreat only for missing ground (landing foot, or support foot past a table edge while
    // walking), with support checks; a new retreat fault stays in HOLD.
    bool walking = TravelLengthX || TravelLengthZ || TravelRotationY || ForceGaitStepCnt;
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      walking |= abs(GaitPosX[leg]) > cGPlimit || GaitPosY[leg] != 0 ||
                 abs(GaitPosZ[leg]) > cGPlimit || abs(GaitRotY[leg]) > cGPlimit;
    }
    if ((footRecovery == RECOVERY_IDLE || footRecovery == RECOVERY_SETTLE) && !walking &&
        footLanding_timedOut() && footLanding_supportLost())
    {
      // Gait at rest: retract the lost foot, then set it down again instead of holding.
      if (footLanding_retry() && footLanding_lostMask())
      {
        walkingStartReplace();
      }
      else
      {
        footRecovery = RECOVERY_HOLD;
        Serial.println(F("Lost foot cannot be re-placed. HOLD; secure robot and restart."));
      }
    }
    else if (footRecovery == RECOVERY_REPLACE && footLanding_timedOut() &&
             !footLanding_supportLost() && !replaceRetreated &&
             (recoveryTravelX || recoveryTravelZ || recoveryRotation))
    {
      // The re-placed foot found no ground either: walk back once, then re-check all feet.
      footLanding_abandon(replaceLeg);
      if (footLanding_retry())
      {
        replaceRetreated = true;
        footRecovery = RECOVERY_REVERSE;
        recoverySteps = 0;
        Serial.println(F("Re-placed foot found no ground; reversing last travel."));
      }
      else
      {
        footRecovery = RECOVERY_HOLD;
        Serial.println(F("Opposite travel blocked: support unavailable. HOLD; secure robot and restart."));
      }
    }
    else if (footRecovery == RECOVERY_IDLE && footLanding_timedOut() &&
        (!footLanding_supportLost() || walking) &&
        (recoveryTravelX || recoveryTravelZ || recoveryRotation))
    {
      if (footLanding_beginRestore())
      {
        footRecovery = RECOVERY_RETRACE;
      }
      else if (footLanding_allSupported())
      {
        footLanding_retry();
        footRecovery = RECOVERY_REVERSE;
        recoverySteps = 0;
      }
      else
      {
        footRecovery = RECOVERY_HOLD;
        Serial.println(
            F("Opposite travel blocked: support unavailable. HOLD; secure robot and restart."));
      }
    }
    else if (footRecovery == RECOVERY_RETRACE && footLanding_allSupported())
    {
      footLanding_retry();
      footRecovery = RECOVERY_REVERSE;
      recoverySteps = 0;
    }
    else
    {
      footRecovery = RECOVERY_HOLD;
      Serial.println(F("Recovery blocked by fault. HOLD; secure robot and restart."));
    }
    if (footLanding_fault())
    {
      return;
    }
  }
  if (footRecovery == RECOVERY_REVERSE)
  {
    TravelLengthX = -recoveryTravelX;
    TravelLengthZ = -recoveryTravelZ;
    TravelRotationY = -recoveryRotation;
  }
  else if (footRecovery == RECOVERY_SETTLE || footRecovery == RECOVERY_REPLACE)
  {
    TravelLengthX = TravelLengthZ = TravelRotationY = 0;
  }
  if (!footLanding_engaged() && CHexInitY + bodyPosY <= CHexInitY &&
      abs(TravelLengthX) <= cTravelDeadZone && abs(TravelLengthZ) <= cTravelDeadZone &&
      abs(TravelRotationY) <= cTravelDeadZone)
  {
    return; // Leave the intentional folded startup untouched until a standing height is chosen.
  }
  if (footFallback)
  {
    // One-leg support fallback trades speed and turning for smaller changes to the support
    // footprint.
    TravelLengthX = _min(_max(TravelLengthX, -CREEP_TRAVEL_MM), CREEP_TRAVEL_MM);
    TravelLengthZ = _min(_max(TravelLengthZ, -CREEP_TRAVEL_MM), CREEP_TRAVEL_MM);
    TravelRotationY = 0;
  }
  // A pending contact search sets fWalking too; finishing seating alone must not start another
  // swing.
  bool gaitDisplaced = false;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    gaitDisplaced |= abs(GaitPosX[leg]) > cGPlimit || GaitPosY[leg] != 0 ||
                     abs(GaitPosZ[leg]) > cGPlimit || abs(GaitRotY[leg]) > cGPlimit;
  }
  bool stepping = abs(TravelLengthX) > cTravelDeadZone || abs(TravelLengthZ) > cTravelDeadZone ||
                  abs(TravelRotationY) > cTravelDeadZone || gaitDisplaced || ForceGaitStepCnt;
  servoMoveTime = stepping ? NomGaitSpeed + InputTimeDelay * 2 + SpeedControl +
                                 (bBalanceMode ? BALANCE_DELAY : 0)
                           : 50 + SpeedControl;
  if (creepActive)
  {
    servoMoveTime = _max(servoMoveTime, CREEP_STEP_MS);
  }
  if (footFallback || footRecovery == RECOVERY_REPLACE)
  {
    servoMoveTime = _max(servoMoveTime, FOOT_WALK_FALLBACK_STEP_MS);
  }

  if (uint32_t(now - lastOutput) < 20)
  {
    return;
  }
  lastOutput = now;
  static uint32_t pauseClock;
  static long pauseInput[11];
  long input[] = {TravelLengthX, TravelLengthZ, TravelRotationY, LegLiftHeight, bodyPosX, bodyPosY,
                  bodyPosZ,      BodyRotX,      BodyRotY,        BodyRotZ,      GaitType};
  if (walkingPaused)
  {
    // Landing time stands still while paused, so no descent, grace, or timeout progresses.
    footLanding_delay(uint32_t(now - pauseClock));
    pauseClock = now;
    if (!memcmp(input, pauseInput, sizeof(input)))
    {
      walkingFootTargets = true;
      controlLegs(); // Keep holding the last accepted pose.
      walkingFootTargets = false;
      return;
    }
  }
  // Snapshot everything a rejected frame could change so a joint-limit pause leaves no trace.
  struct
  {
    long x[NUM_OF_LEGS], y[NUM_OF_LEGS], z[NUM_OF_LEGS], rotation[NUM_OF_LEGS];
    long startX[NUM_OF_LEGS], startZ[NUM_OF_LEGS], startRotation[NUM_OF_LEGS];
    short coxa[NUM_OF_LEGS], femur[NUM_OF_LEGS], tibia[NUM_OF_LEGS];
    long travelX, travelZ, rotationY;
    byte step, force, request, last, gait;
    short lift;
    bool fallback;
    FootRecovery recovery;
    unsigned recoverySteps;
  } saved;
  memcpy(saved.x, GaitPosX, sizeof(saved.x));
  memcpy(saved.y, GaitPosY, sizeof(saved.y));
  memcpy(saved.z, GaitPosZ, sizeof(saved.z));
  memcpy(saved.rotation, GaitRotY, sizeof(saved.rotation));
  memcpy(saved.startX, stepStartX, sizeof(saved.startX));
  memcpy(saved.startZ, stepStartZ, sizeof(saved.startZ));
  memcpy(saved.startRotation, stepStartRotation, sizeof(saved.startRotation));
  memcpy(saved.coxa, coxaAngle, sizeof(saved.coxa));
  memcpy(saved.femur, femurAngle, sizeof(saved.femur));
  memcpy(saved.tibia, tibiaAngle, sizeof(saved.tibia));
  saved.travelX = TravelLengthX;
  saved.travelZ = TravelLengthZ;
  saved.rotationY = TravelRotationY;
  saved.step = GaitStep;
  saved.force = ForceGaitStepCnt;
  saved.request = TravelRequest;
  saved.last = LastLeg;
  saved.gait = GaitType;
  saved.lift = LegLiftHeight;
  saved.fallback = footFallback;
  saved.recovery = footRecovery;
  saved.recoverySteps = recoverySteps;
  footLanding_save();
  bool supportReady = true;
  // Advance landing coordinates only when an output frame will be issued.
  if (!footSensorBypass)
  {
    if (!footLanding_engaged())
    {
      walkingSupportGeometry(false);
      footLanding_plan(GaitPosY, GaitPosY, servoMoveTime);
    }
    footLanding_update();
    byte airborne = 0;
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      if (GaitPosY[leg] < 0)
      {
        airborne |= 1 << leg;
      }
    }
    // Missing switches block new swings, not bounded landing/support searches. Those searches
    // must reach their targets and run their own timeout rather than faulting after release grace.
    supportReady = footLanding_supportReady(airborne);
    footSupportWaiting = !supportReady;
  }
  if (footLanding_fault())
  {
    return;
  }
  bool stepAttempted = false;
  if (footRecovery == RECOVERY_REPLACE && replaceLeg >= NUM_OF_LEGS)
  {
    walkingStartReplace(); // Previous foot is down; continue with the next lost foot, if any.
    if (replaceLeg >= NUM_OF_LEGS)
    {
      footRecovery = RECOVERY_SETTLE; // Nothing lost any more; re-check all feet.
    }
  }
  if (footRecovery == RECOVERY_REPLACE && !footLanding_busy() &&
      uint32_t(now - lastStep) >= stepDuration && servoTargetsReady())
  {
    // Lift the lost foot, then lower it at its current (home) spot with the normal landing search.
    long previousY[NUM_OF_LEGS];
    memcpy(previousY, GaitPosY, sizeof(previousY));
    if (!replaceLifted && supportReady)
    {
      GaitPosY[replaceLeg] = -_min(LegLiftHeight, FALLBACK_LEG_LIFT_MM);
      replaceLifted = stepAttempted = true;
    }
    else if (replaceLifted && GaitPosY[replaceLeg] < 0)
    {
      GaitPosY[replaceLeg] = 0;
      stepAttempted = true;
    }
    else if (replaceLifted)
    {
      replaceLeg = NUM_OF_LEGS; // Landed with contact; a failed search faults instead.
    }
    if (stepAttempted)
    {
      memcpy(stepStartX, GaitPosX, sizeof(stepStartX));
      memcpy(stepStartZ, GaitPosZ, sizeof(stepStartZ));
      memcpy(stepStartRotation, GaitRotY, sizeof(stepStartRotation));
      uint32_t firstFrameMs = (1000U + PWM_FREQ - 1) / PWM_FREQ;
      footLanding_plan(previousY, GaitPosY, servoMoveTime, firstFrameMs);
      footLanding_update();
    }
  }
  if (supportReady && footRecovery != RECOVERY_RETRACE && footRecovery != RECOVERY_REPLACE &&
      !footLanding_busy() && stepping &&
      uint32_t(now - lastStep) >= stepDuration && servoTargetsReady())
  {
    long previousY[NUM_OF_LEGS];
    long previousX[NUM_OF_LEGS], previousZ[NUM_OF_LEGS], previousRotation[NUM_OF_LEGS];
    memcpy(previousY, GaitPosY, sizeof(previousY));
    memcpy(previousX, GaitPosX, sizeof(previousX));
    memcpy(previousZ, GaitPosZ, sizeof(previousZ));
    memcpy(previousRotation, GaitRotY, sizeof(previousRotation));
    if (footRecovery == RECOVERY_IDLE && (TravelLengthX || TravelLengthZ || TravelRotationY))
    {
      recoveryTravelX = TravelLengthX;
      recoveryTravelZ = TravelLengthZ;
      recoveryRotation = TravelRotationY;
    }
    bool advanced = walkingPlanStep();
    if (advanced)
    {
      memcpy(stepStartX, walkingFrameIssued ? issuedX : previousX, sizeof(stepStartX));
      memcpy(stepStartZ, walkingFrameIssued ? issuedZ : previousZ, sizeof(stepStartZ));
      memcpy(stepStartRotation, walkingFrameIssued ? issuedRotation : previousRotation,
             sizeof(stepStartRotation));
    }
    if (advanced && !footSensorBypass)
    {
      uint32_t firstFrameMs = (1000U + PWM_FREQ - 1) / PWM_FREQ;
      footLanding_plan(previousY, GaitPosY, servoMoveTime, firstFrameMs);
      footLanding_update();
    }
    if (advanced && footRecovery == RECOVERY_REVERSE && ++recoverySteps >= 3U * StepsInGait)
    {
      // Count accepted gait steps, not blocked attempts; stop after three requested reverse
      // periods.
      footRecovery = RECOVERY_SETTLE;
      TravelLengthX = TravelLengthZ = TravelRotationY = 0;
      Serial.println(
          F("Three reverse gait cycles requested; finishing pending contact and settling."));
    }
    stepAttempted = true;
  }
  if (footLanding_fault())
  {
    return;
  }
  long plannedY[NUM_OF_LEGS];
  // Feed searched/latched Y into IK, then restore nominal Y so the next gait transition remains
  // meaningful.
  memcpy(plannedY, GaitPosY, sizeof(plannedY));
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    GaitPosY[leg] = footSensorBypass ? plannedY[leg] + footBypassOffset[leg]
                                     : footLanding_y(leg, plannedY[leg]);
  }
  if (footRecovery == RECOVERY_RETRACE)
  {
    footLanding_restoreTargets(GaitPosX, GaitPosY, GaitPosZ, GaitRotY);
    if (footLanding_fault())
    {
      memcpy(GaitPosY, plannedY, sizeof(plannedY));
      return;
    }
    if (footLanding_restored())
    {
      footRecovery = RECOVERY_REVERSE;
      recoverySteps = 0;
      // The restored Y coordinates now live in per-foot support offsets.
      memset(plannedY, 0, sizeof(plannedY));
    }
  }
  // Freeze body corrections during landing/recovery/bypass so they cannot undo a latched contact
  // target.
  if (!footSensorBypass && !footLanding_busy() && footRecovery != RECOVERY_RETRACE &&
      footRecovery != RECOVERY_REVERSE && footRecovery != RECOVERY_SETTLE &&
      footRecovery != RECOVERY_REPLACE)
  {
    TotalTransX = TotalTransY = TotalTransZ = 0;
    TotalXBal1 = TotalYBal1 = TotalZBal1 = 0;
    doBalance();
    pid_update(walkingBodyBalance, 3);
  }
  long plannedX[NUM_OF_LEGS], plannedZ[NUM_OF_LEGS], plannedRotation[NUM_OF_LEGS];
  memcpy(plannedX, GaitPosX, sizeof(plannedX));
  memcpy(plannedZ, GaitPosZ, sizeof(plannedZ));
  memcpy(plannedRotation, GaitRotY, sizeof(plannedRotation));
  if (!footSensorBypass && footRecovery != RECOVERY_RETRACE)
  {
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      GaitPosX[leg] = footLanding_horizontal(leg, stepStartX[leg], plannedX[leg]);
      GaitPosZ[leg] = footLanding_horizontal(leg, stepStartZ[leg], plannedZ[leg]);
      GaitRotY[leg] = footLanding_horizontal(leg, stepStartRotation[leg], plannedRotation[leg]);
    }
  }
  long effectiveX[NUM_OF_LEGS], effectiveZ[NUM_OF_LEGS], effectiveRotation[NUM_OF_LEGS];
  memcpy(effectiveX, GaitPosX, sizeof(effectiveX));
  memcpy(effectiveZ, GaitPosZ, sizeof(effectiveZ));
  memcpy(effectiveRotation, GaitRotY, sizeof(effectiveRotation));
  walkingFootTargets = true;
  walkingUnreachableLeg = NUM_OF_LEGS;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    walkingSearchDepth[leg] = footSensorBypass ? 0 : footLanding_searchDepth(leg);
  }
  doKinetic();
  memcpy(GaitPosX, plannedX, sizeof(plannedX));
  memcpy(GaitPosZ, plannedZ, sizeof(plannedZ));
  memcpy(GaitRotY, plannedRotation, sizeof(plannedRotation));
  memcpy(GaitPosY, plannedY, sizeof(plannedY));
  byte limitLeg = walkingUnreachableLeg;
  const char *limitReason = "unreachable walking target";
  for (byte leg = 0; leg < NUM_OF_LEGS && limitLeg == NUM_OF_LEGS; ++leg)
  {
    // Check both configured joint limits and calibration-adjusted servo range before any PWM is
    // issued.
    short angles[] = {coxaAngle[leg], femurAngle[leg], tibiaAngle[leg]};
    const short *mins[] = {cCoxaMin1, cFemurMin1, cTibiaMin1};
    const short *maxs[] = {cCoxaMax1, cFemurMax1, cTibiaMax1};
    const short *offsets[] = {CoxaAngleOffset, FemurAngleOffset, TibiaAngleOffset};
    for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
    {
      if (limitLeg == NUM_OF_LEGS &&
          (angles[joint] < (short)pgm_read_word(&mins[joint][leg]) ||
           angles[joint] > (short)pgm_read_word(&maxs[joint][leg]) ||
           angles[joint] + (short)pgm_read_word(&offsets[joint][leg]) < -900 ||
           angles[joint] + (short)pgm_read_word(&offsets[joint][leg]) > 900))
      {
        limitLeg = leg;
        limitReason = "walking joint limit";
      }
    }
  }
  if (limitLeg < NUM_OF_LEGS && !footLanding_fault())
  {
    // Reject only this frame: restore the last accepted pose and gait state.
    memcpy(GaitPosX, saved.x, sizeof(saved.x));
    memcpy(GaitPosY, saved.y, sizeof(saved.y));
    memcpy(GaitPosZ, saved.z, sizeof(saved.z));
    memcpy(GaitRotY, saved.rotation, sizeof(saved.rotation));
    memcpy(stepStartX, saved.startX, sizeof(saved.startX));
    memcpy(stepStartZ, saved.startZ, sizeof(saved.startZ));
    memcpy(stepStartRotation, saved.startRotation, sizeof(saved.startRotation));
    memcpy(coxaAngle, saved.coxa, sizeof(saved.coxa));
    memcpy(femurAngle, saved.femur, sizeof(saved.femur));
    memcpy(tibiaAngle, saved.tibia, sizeof(saved.tibia));
    TravelLengthX = saved.travelX;
    TravelLengthZ = saved.travelZ;
    TravelRotationY = saved.rotationY;
    GaitStep = saved.step;
    ForceGaitStepCnt = saved.force;
    TravelRequest = saved.request;
    LastLeg = saved.last;
    LegLiftHeight = saved.lift;
    footFallback = saved.fallback;
    if (GaitType != saved.gait)
    {
      GaitType = saved.gait;
      selectGait();
      GaitStep = saved.step;
    }
    footRecovery = saved.recovery;
    recoverySteps = saved.recoverySteps;
    footLanding_rollback();
    walkingFootTargets = false;
    if (walkingUnreachableLeg < NUM_OF_LEGS && walkingUnreachableSearch)
    {
      // The search went below reach without finding ground: run the missing-ground
      // retrace/reverse recovery from the last issued pose instead of pausing.
      walkingPaused = false;
      walkingPauseReason[0] = 0;
      footLanding_noGround(walkingUnreachableLeg);
      return;
    }
    walkingFootTargets = true;
    controlLegs(); // Hold the last accepted pose.
    walkingFootTargets = false;
    if (!walkingPaused)
    {
      walkingPaused = true;
      static const char *const pauseLeg[] = {"RF", "RM", "RR", "LF", "LM", "LR"};
      snprintf(walkingPauseReason, sizeof(walkingPauseReason), "%s (%s)", limitReason,
               pauseLeg[limitLeg]);
      Serial.print(F("Walking paused: "));
      Serial.println(walkingPauseReason);
    }
    pauseClock = now;
    memcpy(pauseInput, input, sizeof(pauseInput));
    return;
  }
  if (!footLanding_fault())
  {
    if (walkingPaused)
    {
      walkingPaused = false;
      walkingPauseReason[0] = 0;
      Serial.println(F("Walking resumed; landing checks active."));
    }
    long effectiveY[NUM_OF_LEGS];
    for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    {
      effectiveY[leg] = footSensorBypass ? plannedY[leg] + footBypassOffset[leg]
                                         : footLanding_y(leg, plannedY[leg]);
    }
    limitAngles();
    if (stepAttempted)
    {
      beginServoMove();
    }
    controlLegs();
    if (stepAttempted)
    {
      lastStep = millis();
      stepDuration = servoMoveTime;
    }
    walkingSupportGeometry(true);
    // Cache only after output succeeds; sensor cancellation must not adopt an invalid future
    // target.
    memcpy(issuedX, effectiveX, sizeof(issuedX));
    memcpy(issuedY, effectiveY, sizeof(issuedY));
    memcpy(issuedPlanY, plannedY, sizeof(issuedPlanY));
    memcpy(issuedZ, effectiveZ, sizeof(issuedZ));
    memcpy(issuedRotation, effectiveRotation, sizeof(issuedRotation));
    issuedStep = GaitStep;
    issuedForce = ForceGaitStepCnt;
    walkingFrameIssued = true;
    if (!footSensorBypass && footRecovery != RECOVERY_RETRACE)
    {
      footLanding_record(effectiveX, effectiveY, effectiveZ, effectiveRotation);
    }
  }
  walkingFootTargets = false;
  bool displaced = false;
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    displaced |= abs(GaitPosX[leg]) > cGPlimit || GaitPosY[leg] != 0 ||
                 abs(GaitPosZ[leg]) > cGPlimit || abs(GaitRotY[leg]) > cGPlimit;
  }
  fWalking = displaced || footLanding_busy();
  if (footFallback && !fWalking && !TravelLengthX && !TravelLengthZ && !TravelRotationY &&
      !ForceGaitStepCnt)
  {
    walkingRestoreGait();
  }
  if (footRecovery == RECOVERY_SETTLE && !fWalking)
  {
    if (footLanding_lostMask())
    {
      walkingStartReplace();
    }
    else if (footSensor_isAllLegsTriggered())
    {
      // Normal stance with every foot confirmed on the ground.
      footRecovery = RECOVERY_WAIT_CENTER;
      replaceRetreated = false;
      Serial.println(F("Retreat stopped. Center all sticks, then send fresh movement input."));
    }
  }
}
#endif

#ifdef WEB_CONTROLLER_ENABLED
char webResult[128] = "Disabled. Configure network and enable explicitly.";

void webStop(bool disable)
{
  // Stop travel without releasing torque; settle a healthy gait, but freeze a faulted/recovering
  // one.
  TravelLengthX = TravelLengthZ = TravelRotationY = 0;
  SLLegX = SLLegY = SLLegZ = 0;
  if (OperationMode == MODE_SINGLE_LEG && SelectedLeg < NUM_OF_LEGS)
  {
    bSLHold[SelectedLeg] = true;
  }
  if (footLanding_fault() || footRecovery != RECOVERY_IDLE)
  {
    webStopHold = true;
  }
  if (disable)
  {
    webArmed = false;
    webDisablePending = true;
  }
}

void webService()
{
  // Apply HTTP commands on the motion thread, not the server task, so gait and servo state have one
  // writer.
  if (webArmed && webControl_expired())
  {
    webStop(true);
    snprintf(webResult, sizeof(webResult), "Connection timeout: explicit enable required.");
  }
  WebCommand cmd;
  if (webControl_take(cmd))
  {
    if (cmd.ownershipChanged && webArmed)
    {
      webStop(true);
    }
    bool centered = abs(cmd.lx - 128) <= 4 && abs(cmd.ly - 128) <= 4 && abs(cmd.rx - 128) <= 4 &&
                    abs(cmd.ry - 128) <= 4;
    if (!strcmp(cmd.action, "feet") && cmd.value == 0 && footWalkingEnabled)
    {
      // Exit already stops/disarms; a simultaneous STOP must not swallow cancellation while in
      // HOLD. Reject non-neutral clients explicitly instead of silently treating their exit as a
      // generic stop.
      webStop(true);
      if (!centered)
      {
        snprintf(webResult, sizeof(webResult),
                 "Foot sensor exit rejected: center inputs and press Foot sensor again.");
      }
      else
      {
        bool cancelled = walkingCancelSensors();
        snprintf(webResult, sizeof(webResult), "%s",
                 cancelled ? "Foot sensor disabled; pose held. Enable Control, then fresh movement "
                             "to continue "
                             "without sensors."
                           : "Cannot clear battery/geometry fault with Foot sensor. Correct fault "
                             "and restart.");
      }
      Serial.println(webResult);
    }
    else if (cmd.stop || !cmd.enable)
    {
      bool wasActive = webArmed || webDisablePending || cmd.stop;
      webStop(!cmd.enable);
      if (wasActive)
      {
        snprintf(webResult, sizeof(webResult), "Stop requested; torque retained.");
      }
    }
    else if (!strcmp(cmd.action, "enable"))
    {
      // Lease renewal or a movement packet is not consent to resume; require explicit centered
      // enable.
      if (!centered || (webStopHold && !footSensorBypass) || footLanding_fault() ||
          footRecovery == RECOVERY_HOLD)
      {
        snprintf(webResult, sizeof(webResult),
                 "Enable rejected: center inputs / restore support and reset after fault.");
      }
      else if (!webDisablePending)
      {
        webArmed = true;
        bHexON = true;
        if (footSensorBypass)
        {
          webStopHold = false;
        }
        snprintf(webResult, sizeof(webResult), "Control enabled.");
      }
      else
      {
        snprintf(webResult, sizeof(webResult),
                 "Enable rejected: wait for stop/settle to complete.");
      }
    }
    else if (webArmed && !webStopHold && !webDisablePending)
    {
      if (cmd.action[0])
      {
        // Speed changes timing only; geometry/gait settings still require centered, settled feet.
        bool ok = !footLanding_fault() && footRecovery == RECOVERY_IDLE &&
                  (!strcmp(cmd.action, "speed") || (centered && gaitAtRest()));
        const char *rejection = "rejected (center, settle, or incompatible setting)";
        if (!strcmp(cmd.action, "feet") && cmd.value == 1)
        {
          rejection = "not enabled; check sensors, support/fault status, center inputs and settle";
        }
        if (ok && footSensorBypass && strcmp(cmd.action, "feet") && strcmp(cmd.action, "speed"))
        {
          ok = false;
          rejection =
              "pose-preserving sensor bypass; only WALK/speed until Foot sensor is re-enabled";
        }
        if (ok)
        {
          int v = cmd.value;
          if (!strcmp(cmd.action, "mode") && v >= 0 && v < CONTROL_MODE)
          {
            OperationMode = v;
            updateCreepProfile();
          }
          else if (!strcmp(cmd.action, "gait") && !creepActive && v >= 0 && v < GAIT_TYPE)
          {
            GaitType = v;
            GaitStep = 1;
            selectGait();
          }
          else if (!strcmp(cmd.action, "height") && (v == 45 || v == 56 || v == 65 || v == 85))
          {
            g_BodyYOffset = v - CHexInitY;
            g_BodyYShift = 0;
            bodyPosY = g_BodyYOffset;
            updateCreepProfile();
            adjustLegPosToBodyHeight();
          }
          else if (!strcmp(cmd.action, "bodyUp") || !strcmp(cmd.action, "bodyDown"))
          {
            int change = !strcmp(cmd.action, "bodyUp") ? BODY_HEIGHT_STEP_MM : -BODY_HEIGHT_STEP_MM;
            g_BodyYOffset = _min(_max(g_BodyYOffset + change, 0), MAX_BODY_Y_OFFSET);
            g_BodyYShift = 0;
            bodyPosY = g_BodyYOffset;
            updateCreepProfile();
            adjustLegPosToBodyHeight();
          }
          else if (!strcmp(cmd.action, "speed") && v >= 0 && v <= 100)
          {
            SpeedControl = v;
          }
          else if (!strcmp(cmd.action, "lift") && !creepActive && v >= MIN_LEG_LIFT_MM &&
                   v <= MAX_LEG_LIFT_MM && v % LEG_LIFT_STEP_MM == 0)
          {
            LegLiftHeight = v;
            bDoubleHeight = v > DEFAULT_LEG_LIFT_MM;
          }
          else if ((!strcmp(cmd.action, "liftUp") || !strcmp(cmd.action, "liftDown")) &&
                   !creepActive)
          {
            // Relative steps stay correct when taps arrive faster than telemetry updates.
            int change = !strcmp(cmd.action, "liftUp") ? LEG_LIFT_STEP_MM : -LEG_LIFT_STEP_MM;
            LegLiftHeight = _min(_max(LegLiftHeight + change, MIN_LEG_LIFT_MM), MAX_LEG_LIFT_MM);
            bDoubleHeight = LegLiftHeight > DEFAULT_LEG_LIFT_MM;
          }
          else if (!strcmp(cmd.action, "travel") && !creepActive && (v == 0 || v == 1))
          {
            bDblTravel = v;
          }
          else if (!strcmp(cmd.action, "balance") && !creepActive && (v == 0 || v == 1))
          {
            bBalanceMode = v;
          }
          else if (!strcmp(cmd.action, "imu") && !creepActive && (v == 0 || v == 1))
          {
            bBodyBalanceMode = v;
          }
          else if (!strcmp(cmd.action, "feet") && (v == 0 || v == 1))
          {
            if (v && (OperationMode != MODE_WALK || SelectedLeg < NUM_OF_LEGS ||
                      CHexInitY + bodyPosY <= CHexInitY || !walkingEnableSensors()))
            {
              ok = false;
              rejection = "not enabled; check support pattern, sensors, WALK, standing height and "
                          "no selected leg";
            }
            else if (!v)
            {
              footWalkingEnabled = v;
              footLanding_init();
            }
          }
          else if (!strcmp(cmd.action, "leg") && OperationMode == MODE_SINGLE_LEG &&
                   ((v >= 0 && v < NUM_OF_LEGS) || v == 255))
          {
            SelectedLeg = v;
          }
          else if (!strcmp(cmd.action, "leg"))
          {
            ok = false;
            rejection = "select Operation = Single leg first";
          }
          else if (!strcmp(cmd.action, "hold") && OperationMode == MODE_SINGLE_LEG &&
                   SelectedLeg < NUM_OF_LEGS && (v == 0 || v == 1))
          {
            // A centered hold toggle is not a fresh joystick command; do not replay old offsets.
            SLLegX = SLLegY = SLLegZ = 0;
            bSLHold[SelectedLeg] = v;
          }
          else if (!strcmp(cmd.action, "hold"))
          {
            ok = false;
            rejection = "select a leg in Operation = Single leg first";
          }
          else
          {
            ok = false;
          }
        }
        snprintf(webResult, sizeof(webResult), "%s: %s", cmd.action, ok ? "applied" : rejection);
        if (!ok)
        {
          Serial.println(webResult);
        }
      }
      else if (footRecovery == RECOVERY_WAIT_CENTER)
      {
        if (centered)
        {
          footRecovery = RECOVERY_WAIT_INPUT;
        }
      }
      else if (footRecovery == RECOVERY_WAIT_INPUT)
      {
        if (!centered)
        {
          footRecovery = RECOVERY_IDLE;
        }
      }
      else if (footRecovery == RECOVERY_IDLE && !footLanding_busy() && !footLanding_fault())
      {
        if (footSensorBypass && !centered)
        {
          footBypassAwaitInput = false;
        }
        byte packet[7] = {0, 0, 0, byte(cmd.lx), byte(cmd.ly), byte(cmd.rx), byte(cmd.ry)};
        // Reuse the existing seven-byte axis/checksum path; named settings do not emulate held app
        // buttons.
        for (byte i = 3; i < 7; ++i)
        {
          if (abs(int(packet[i]) - 128) <= 4)
          {
            packet[i] = 128;
#ifdef REVERSE_CMD_ENABLED
            if (i != 3)
            {
              // Legacy reversal is 255 - value, so pre-adjust neutral to avoid introducing a
              // one-count drift.
              packet[i] = 127; // Reverse mapping must produce exactly neutral.
            }
#endif
          }
        }
        unsigned checksum = packet[3] + packet[4] + packet[5] + packet[6];
        packet[0] = (checksum >> 8) + (checksum & 255);
        doCommand(packet);
      }
      else if (centered && footRecovery == RECOVERY_IDLE)
      {
        TravelLengthX = TravelLengthZ = TravelRotationY = 0;
      }
    }
    else if (cmd.action[0])
    {
      snprintf(webResult, sizeof(webResult), "%s: rejected; %s", cmd.action,
               webStopHold || footLanding_fault()
                   ? "HOLD; secure robot and correct fault before restart"
                   : "enable Control and wait for settling");
      Serial.println(webResult);
    }
  }
  if (webDisablePending && (webStopHold || gaitAtRest()))
  {
    bHexON = false;
    webDisablePending = false;
  }
  static uint32_t lastStatus;
  static uint32_t lastBattery;
  static float cell1, cell2;
  uint32_t now = millis();
  if (uint32_t(now - lastBattery) >= 1000)
  {
    // Sample I2C on this thread; HTTP handlers read the cache instead of competing for the bus.
    lastBattery = now;
    battMan_getVoltages(&cell1, &cell2);
    http_setVoltages(cell1, cell2);
  }
  if (uint32_t(now - lastStatus) >= 200)
  {
    // Report accepted runtime state and rejection/fault reasons so a blocked robot does not look
    // like a frozen UI.
    lastStatus = now;
    JsonDocument status;
    status["ready"] = true;
    status["enabled"] = bool(bHexON);
    status["armed"] = webArmed;
    status["hold"] = webStopHold || footLanding_fault();
    status["mode"] = OperationMode;
    status["gait"] = GaitType;
    status["height"] = CHexInitY + bodyPosY;
    status["lift"] = LegLiftHeight;
    status["speed"] = SpeedControl;
    status["travel"] = bDblTravel;
    status["balance"] = bBalanceMode;
    status["imu"] = bBodyBalanceMode;
    status["leg"] = SelectedLeg;
    // Report the selected leg, not the last leg whose hold button was pressed.
    status["singleLegHold"] = SelectedLeg < NUM_OF_LEGS && bSLHold[SelectedLeg];
    status["creep"] = creepActive;
    status["fallback"] = footFallback;
    status["requestedGait"] = footFallback ? footRequestedGait : GaitType;
    status["supportWaiting"] = footSupportWaiting;
    status["paused"] = walkingPaused ? walkingPauseReason : "";
    status["sensorBypass"] = footSensorBypass;
    status["awaitMovement"] = footBypassAwaitInput;
    status["hardFault"] = footLanding_hardFault();
    status["feetProtection"] = footWalkingEnabled;
    status["contacts"] = footSensor_getSensorMask();
    status["landing"] = footLanding_busy();
    status["recovery"] = int(footRecovery);
    const char *recoveryNames[] = {"IDLE",        "RETRACE",    "REVERSE", "SETTLE",
                                   "WAIT_CENTER", "WAIT_INPUT", "HOLD",    "REPLACE"};
    status["recoveryState"] = recoveryNames[int(footRecovery)];
    status["fault"] = footLanding_reason();
    status["result"] = webResult;
    if (isfinite(cell1) && isfinite(cell2) && lastBattery)
    {
      status["cell1"] = cell1;
      status["cell2"] = cell2;
    }
    status["station"] =
        wifi_ready() && WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
    status["hotspot"] = wifi_ready() ? WiFi.softAPIP().toString() : "";
#ifdef CAMERA_ENABLED
    status["cameraAvailable"] = true;
#else
    status["cameraAvailable"] = false;
#endif
    char json[2048];
    serializeJson(status, json, sizeof(json));
    webControl_status(json);
  }
}
#endif

void loop()
{
#ifdef FOOT_SENSOR_ENABLED
  footSensor_update();
#endif
#ifdef FOOT_SENSOR_TEST_ENABLED
  // Test mode owns all motion; do not fall through into normal commands, gait, or single-leg
  // updates.
  footSensorTest_update();
  return;
#endif
#ifndef CALIBRATION_ENABLED
#if defined(HTTP_ENABLED) || defined(CAMERA_ENABLED)
  wifi_update(!bHexON && !webArmed);
#endif
#ifdef WEB_CONTROLLER_ENABLED
  webService();
  if (webStopHold)
  {
    // Keep network/status servicing above this return, but issue no new motion targets while HOLD
    // is latched.
    return;
  }
#endif
  byte commandData[7];
  bool cmdDataReady = getCmd(commandData);

  if (cmdDataReady)
  {
    cmdDataReady = false;
#ifdef WEB_CONTROLLER_ENABLED
    if (!webArmed && !webDisablePending)
#endif
    {
      doCommand(commandData);
    }
  }
  // Apply explicit single-leg transitions before selecting the walking scheduler.
  controlSingleLeg();
#ifdef FOOT_SENSOR_WALK_ENABLED
  if (footSensorBypass ||
      (footWalkingEnabled && (OperationMode == MODE_WALK || footLanding_fault())))
  {
    // Bypass must keep the pose-preserving scheduler, not fall back to legacy home/settling logic.
    contactWalking();
    return;
  }
  if (footLanding_engaged())
  {
    footLanding_init();
  }
#endif
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
  if (bHexON && battMan_isPowerOk())
#else
  if (1)
#endif // DEBUG_ON
  {
    // Drive Servos
    // Calculate Servo Move time
    if ((abs(TravelLengthX) > cTravelDeadZone) || (abs(TravelLengthZ) > cTravelDeadZone) ||
        (abs(TravelRotationY * 2) > cTravelDeadZone))
    {
      servoMoveTime = NomGaitSpeed + (InputTimeDelay * 2) + SpeedControl;
      // Add additional delay when Balance mode is on
      if (bBalanceMode)
      {
        servoMoveTime = servoMoveTime + BALANCE_DELAY;
      }
    }
    else
    { // Movement speed excl. Walking
      servoMoveTime = 50 + SpeedControl;
    }
    if (creepActive)
    {
      servoMoveTime = _max(servoMoveTime, CREEP_STEP_MS);
    }

    // Finding any incident of GaitPos/Rot <>0:
    for (byte legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
    {
      if ((GaitPosX[legNr] > cGPlimit) || (GaitPosX[legNr] < -cGPlimit) ||
          GaitPosY[legNr] != 0 ||
          (GaitPosZ[legNr] > cGPlimit) || (GaitPosZ[legNr] < -cGPlimit) ||
          (GaitRotY[legNr] > cGPlimit) || (GaitRotY[legNr] < -cGPlimit))
      {

        bExtraCycle =
            NrLiftedPos + 1; // For making sure that we are using timed move until all legs are down
        break;
      }
    }

    if (bExtraCycle > 0)
    {
      beginServoMove();
    }
    else
    {
      for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
      {
        servoMotion[leg].cancel();
      }
    }
    controlLegs();
    uint32_t servoMoveStart = millis();
    const word stepMoveTime = servoMoveTime;
    // Finish this pose before calculating another gait target; calculation/ADC time is not motion
    // time.
    if (bExtraCycle > 0)
    {
      bExtraCycle--;
      fWalking = !(bExtraCycle == 0);

      do
      {
#ifdef WEB_CONTROLLER_ENABLED
        webService();
        if (webStopHold || !bHexON)
        {
          break;
        }
#endif
#ifdef FOOT_SENSOR_ENABLED
        footSensor_update();
#endif
        controlLegs();
        // Wait the appropriate time, call any background process while waiting...
      } while (uint32_t(millis() - servoMoveStart) < stepMoveTime || !servoTargetsReady());
    }
  }
#else
  calibrationServo();
  limitAngles();
  controlLegs();
#endif // CALIBRATION_ENABLED
}