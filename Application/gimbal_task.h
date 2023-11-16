#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "includes.h"
#include "system_identification.h"

#define GIMBAL_TASK_PERIOD 1

#define GIMBAL_MAX_CRUISEYAW 67.5f
#define GIMBAL_MAX_CRUISEPITCH 10.0f
#define GIMBAL_MIN_CRUISEPITCH -25.0f

// Gimbal motor define
#define YAW_MOTOR_ID 0x141
#define PITCH_MOTOR_ID 0x206
#define YAW_MOTOR_ZERO_OFFSET 64674
#define PITCH_MOTOR_ZERO_OFFSET 6877

#define SLAMYAW_MOTOR_ZERO_OFFSET 240

// Gimbal attitude define
#define GIMBAL_MAX_DEPRESSION -38
#define GIMBAL_MAX_ELEVATION 20

// Gimbal control define
#define RC_STICK_YAW_RATIO 1
#define RC_STICK_PITCH_RATIO 0.6
#define RC_MOUSE_YAW_RATIO 0.75
#define RC_MOUSE_PITCH_RATIO -1.5

#define YAW_V_PID_MAXOUT 18000
#define YAW_V_PID_MAXINTEGRAL 10000
#define YAW_V_PID_KP 750
#define YAW_V_PID_KI 850
#define YAW_V_PID_KD 0
#define YAW_V_PID_LPF 0.000001

#define YAW_A_PID_MAXOUT 320 / 60 * 2 * 3.14159
#define YAW_A_PID_MAXINTEGRAL 1.0f
#define YAW_A_PID_KP 0.65
#define YAW_A_PID_KI 0
#define YAW_A_PID_KD 0.008
#define YAW_A_PID_KP_AIMASSIST 0.6
#define YAW_A_PID_KI_AIMASSIST 1.5
#define YAW_A_PID_KD_AIMASSIST 0.004
#define YAW_A_PID_LPF 0.0001
#define YAW_A_PID_D_LPF 0.001

#define YAW_V_FFC_MAXOUT 0
#define YAW_V_FCC_C0 0.8
#define YAW_V_FCC_C1 0.2
#define YAW_V_FCC_C2 0
#define YAW_V_FCC_LPF 0.005

#define YAW_A_FFC_MAXOUT 320 / 60 * 2 * 3.14159
#define YAW_A_FCC_C0 0
#define YAW_A_FCC_C1 0.0175
#define YAW_A_FCC_C2 0
#define YAW_A_FCC_LPF 0.001

#define PITCH_V_PID_MAXOUT 18000
#define PITCH_V_PID_MAXINTEGRAL 10000
#define PITCH_V_PID_KP -7500
#define PITCH_V_PID_KI -10000
#define PITCH_V_PID_KD 0
#define PITCH_V_PID_LPF 0.001

#define PITCH_A_PID_MAXOUT 320 / 60 * 2 * 3.14159
#define PITCH_A_PID_MAXINTEGRAL 1.0f
#define PITCH_A_PID_KP 0.7
#define PITCH_A_PID_KI 0
#define PITCH_A_PID_KD 0.0025
#define PITCH_A_PID_KP_AIMASSIST 0.65
#define PITCH_A_PID_KI_AIMASSIST 1.5
#define PITCH_A_PID_KD_AIMASSIST 0
#define PITCH_A_PID_LPF 0.0001
#define PITCH_A_PID_D_LPF 0.001

#define PITCH_V_FFC_MAXOUT 18000
#define PITCH_V_FCC_C0 3
#define PITCH_V_FCC_C1 0.75
#define PITCH_V_FCC_C2 0
#define PITCH_V_FCC_LPF 0.000

#define PITCH_A_FFC_MAXOUT 320 / 60 * 2 * 3.14159
#define PITCH_A_FCC_C0 0
#define PITCH_A_FCC_C1 0.0175
#define PITCH_A_FCC_C2 0
#define PITCH_A_FCC_LPF 0.003

#define YAW_INCREMENT 0.17
#define YAW_REFANGLE_LPF 0.15

#define PITCH_INCREMENT 0.07
#define PITCH_REFANGLE_LPF 0.1

#define YAW_MOTOR_MAXOUT 2000
#define PITCH_MOTOR_MAXOUT 30000 // 30000

enum
{
    Normal_Mode = 0X00,     // 0000 0000
    AimAssist_Mode = 0x01,  // 0000 0001
    Follow_Mode = 0x02,     // 0000 0010
    Count_Mode = 0x04,      // 0000 0100
    Gimbal_Reserve4 = 0x08, // 0000 1000
    Gimbal_Reserve5 = 0x10, // 0001 0000
    Gimbal_Reserve6 = 0x20, // 0010 0000
    Gimbal_Reserve7 = 0x40, // 0100 0000
    Gimbal_Reserve8 = 0x80, // 1000 0000
};

enum
{
    wait_command = 0x00,
    listen_lidar = 0x01,
    find_target = 0x02,
    lost_target = 0x04,
};

typedef struct
{
    FirstOrderSI_t YawSI;
    FirstOrderSI_t PitchSI;

    uint8_t ResetFlag;

    float Q0;
    float Q1;
    float Q2;
    float R;
    float lambda;
} GimbalSI_t;

typedef struct _GimbalControl
{
    Motor_t YawMotor;
    Motor_t PitchMotor;

    float YawAngle;
    float PitchAngle;
    float EncoderYawAngle;
    float EncoderPitchAngle; // ��Pitch�����������õ��������̬��
    float YawAngularVelocity;
    float PitchAngularVelocity;
    float FollowYawVelocity;
    float FollowCoef;

    TD_t YawRefAngularVelocityTD;
    TD_t PitchRefAngularVelocityTD;
    TD_t YawRefAngleTD;
    TD_t PitchRefAngleTD;

    float YawRefAngularVelocity;
    float PitchRefAngularVelocity;
    float YawCtrlAngle;
    float PitchCtrlAngle;
    float YawRefAngle;
    float PitchRefAngle;

    float FilteredYawRefAngle;
    float FilteredPitchRefAngle;

    float LastYawCtrlAngle;
    float LastPitchCtrlAngle;
    float LastFilteredYawRefAngle;
    float LastFilteredPitchRefAngle;

    float rcStickYawRatio;
    float rcStickPitchRatio;
    float rcMouseYawRatio;
    float rcMousePitchRatio;

    float YawIncrement;
    int8_t YawCruiseDirection;
    int8_t LastYawCruiseDirection;
    uint16_t YawCruiseCount;
    float YawRefAngleLPF;

    float PitchIncrement;
    int8_t PitchCruiseDirection;
    int8_t LastPitchCruiseDirection;
    float PitchRefAngleLPF;

    uint8_t is_reached;
    uint8_t aim_status;
    float ChassisOmega[3];

    uint16_t ModeSwitchCount;

    uint8_t LaserState;
    uint8_t Mode;
    uint8_t ModeLast;

    uint16_t YawFrontEncoder;        // ��̨������ǰʱYaw��������ֵ
    uint16_t PitchParallelEncoder;   // ��̨ƽ����Yaw����ʱPitch��������ֵ
    float DepressionEncoderInDegree; // ��̨�������ʱPitch�������̶�Ӧ�Ƕ�ֵ
    float ElevationEncoderInDegree;  // ��̨�������ʱPitch�������̶�Ӧ�Ƕ�ֵ
    float DepressionIMU;             // ��̨�������ʱ��̬��
    float ElevationIMU;              // ��̨�������ʱ��̬��
} Gimbal_t;

typedef struct
{
    Motor_t YawMotor;

    float TotalTheta;
    float Theta;
    float FollowTheta;
    float YawMotorOutput;
} SLAM_t;

enum
{
    LaserOff = 0,
    LaserOn = 1,
};

enum
{
    VelocityMode = 0,
    AngleMode = 1,
};

extern Gimbal_t Gimbal;
extern SLAM_t SLAM;
extern uint8_t FOLLOW_Data_Buf[16];
extern uint8_t FOLLOW_Update;

void Gimbal_Init(void);
void Gimbal_Control(void);
void PitchMotor_Tuning(void);
void YawMotor_Tuning(void);
void GetAngularVelocity(uint8_t *buff);

#endif
