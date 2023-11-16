#ifndef _SHOOT_TASK_H
#define _SHOOT_TASK_H

#include "includes.h"

#define SHOOT_TASK_PERIOD 1

#include "bsp_CAN.h"
#include "system_identification.h"
#define FRIC_RM3508_LEFT_ID 0x203
#define FRIC_RM3508_RIGHT_ID 0x204
#define FRIC_MOTOR_MIN_RPM 2000
#define FRIC_MOTOR_MAX_RPM 8700

// Trigger motor configuration
#define TRIGGER_MOTOR_LEFT_ID 0x201
// #define TRIGGER_MOTOR_RIGHT_ID 0x202
#define TRIGGER_MOTOR_REDUCTION_RATIO 36
#define BULLETS_PER_ROUND 8

#define SWITCH_MOTOR_ID 0x202
#define FIXED_OUTPUT 1500
#define SLEW_OUTPUT 3000
#define MAX_ROTARY_OUTPUT 10000
#define TIMES_COUNT 110

enum
{
  NoShooting = 0,
  DebugShoot,
  OneShot,
  CounterShot,
  KeepShooting,
  AimAssistShooting,
  DebugSpinning,
};

enum _barrel_mode
{
  Unfixed = 0,
  BarrelNormal,
  Overheated,
  Stallad,
};

enum _shooterid
{
  Shooter1 = 1,
  Shooter2,
};

typedef struct _rotary
{
  int16_t fixed_output;
  int16_t slew_output;
  float Recovery_time;
  float Resume_timing;

  Motor_t RotaryMotor;
  float RotaryMotorVelocity;
  float Rotary_Output;
  int8_t Rotary_Output_Coefficient;
  float Rotary_Output_MAX;
  uint8_t Rotary_reset;

  uint8_t Shooter_id;
  uint8_t Last_Shooterid;
  uint8_t Shooter_state;
  uint16_t Rotarytimes;
  uint8_t Barrel_Debug;

  float Stalladornot;
  float Stallad_Recovery_time;
  int32_t Motor_limit_angle[2];
  uint8_t Rotary_Stallad_times;
  uint8_t Rotary_debug;

  float current_lpf;
  float current_last;
  float current_judge;
  float motor_last_current;
  uint8_t Mode_gun;
} Rotary_t;

typedef struct _shoot
{
  uint8_t ShootMode;

  int32_t BulletToShoot;

  uint8_t isLidOpen;
  uint8_t isFricOn;

  float TriggerSpeed;
  float TriggerSpeed1;
  float TriggerSpeed2;
  Motor_t TriggerMotor[2];
  float heatRemain;
  float FreqRatio;
  float BulletSpeedLimit;
  float BulletSpeedCompensation;

  // Motor_t SwitchMotor;

  float SpeedInBulletsPerSec;
  uint32_t NumsOfOneShot;
  uint32_t ShootDelayInMs;

  float InitialKi;

  float BulletVelocity;
  float RefBulletVelocity;
  Rotary_t Rotary;
  float RefFricSpeed;
  float FricSpeed;
  float FricAccel;
  TD_t FricTD;
  Motor_t FricMotor[2];
  FirstOrderSI_t FricMotorSI[2];
} Shoot_t;

extern Shoot_t Shoot;

void Shoot_Init(void);
void Shoot_Control(void);
float ShootAndDelay(Motor_t *trigger, float speedInNumsPerSec, uint32_t NumsOfOneShot, uint32_t delayTimeInMs);
float RotateAngleInDegree(Motor_t *trigger, float speedRPM, float angleInDegree);

#endif
