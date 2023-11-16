#ifndef _AIMASSIST_H
#define _AIMASSIST_H

#include "includes.h"
#include "kalman_filter.h"

#define ENABLE_CALIBRATION 0

#define AUTOAIM_TASK_PERIOD 1
#define CALIBRATION_RX_BUF_NUM 28
#define AimAssist_RX_BUF_NUM 18
#define AimAssist_TX_BUF_NUM 18

#ifndef STD_RADIAN
#define STD_RADIAN(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif

#define PITCH_MAX_DEG 15
#define YAW_MAX_DEG 20

#define X 0
#define Y 1
#define Z 2

typedef struct
{
    uint8_t CF_SOF;
    uint8_t flg;
    int16_t x; // mm
    uint16_t y;
    int16_t z;
    float rVecx; // mm
    float rVecy;
    float rVecz;
    uint32_t time_stamp_ms;
    uint8_t CF_EOF;
} CalibrationFrame_t;

/**
 * @Brief: TX2控制战车帧结构体
 */
typedef struct _controlFrame
{
    uint8_t CF_SOF;
    uint8_t flg;
    int16_t x; // mm
    uint16_t y;
    int16_t z;
    uint32_t time_stamp_ms;
    short reserve1;   // 灯条长度比
    uint8_t reserve2; // 装甲板类型
    uint8_t CF_EOF;
} ControlFrame;

typedef struct _controlFrameFull
{
    uint8_t CF_SOF;         //
    uint8_t flg;            // װ�װ�ID
    int16_t px;             // Ŀ�����ϵ���� x
    uint16_t py;            // Ŀ�����ϵ���� y
    int16_t pz;             // Ŀ�����ϵ���� z
    int16_t rx;             // Ŀ�����ϵ������ x
    int16_t ry;             // Ŀ�����ϵ������ y
    int16_t rz;             // Ŀ�����ϵ������ z
    uint16_t time_stamp_ms; // ms*10
    // uint8_t reserve1;
    // uint8_t reserve2;
    // uint8_t reserve3;
    // uint8_t reserve4;
    // uint8_t reserve5;
    // uint8_t reserve6;
    uint8_t type;   // ��/Сװ��
    uint8_t CF_EOF; // 15
} ControlFrameFull;

/**
 * @Brief: 战车回传数据帧结构体
 */
typedef struct _feedBackFrame
{
    uint8_t FDBF_SOF;
    uint8_t myteam; //  EE DD
    short pitch;    //  10k*rad
    short yaw;
    short roll;
    uint16_t bullet_speed;  // m/s
    uint16_t time_stamp_ms; // ms
    short outpost_alive;
    // short status;
    short hit2;
    uint8_t mode; // 0普通 1前哨战 2工程英雄 3哨兵
    uint8_t FDBF_EOF;
} FeedBackFrame;

enum ObjectType
{
    UNKNOWN_ARMOR = 0,
    SMALL_ARMOR = 1,
    BIG_ARMOR = 2,
    RUNE_WING = 3,
    RUNE_CENTRE = 4
};

enum miniPC_Mode
{
    NORMAL = 0,
    HIT_OUTPOST,
    HIT_1_2,
    HIT_6,
};

enum ColorChannels
{
    INVALID_COLOR = 0,
    BLUE = 1,
    RED = 2
};

enum NTP_MSG
{
    PC2STM32_T1 = 1,
    STM2PC_T3,
    PC2STM32_T4,
    NTP_SUCCESS,
    NTP_FAIL,
    NTP_START
};

enum TargetStatus
{
    TargetLost = 0,
    TargetValid,
};

enum TgtPosPredictStatus
{
    TgtLost = 0,
    TgtTracking,
    TgtSwitch,
    TgtConjecture,
};

typedef struct
{
    float T1;
    float T2;
    float T3;
    float T4;
    float deltaT;
    float TransmitDelay;
} NTP_t;

typedef struct
{
    uint8_t Status;
    uint8_t Mode;
    uint8_t LastMode;
    uint8_t miniPC_Online;
    uint8_t PerspectiveEnable;
    ControlFrame CtrlFrame;
    ControlFrameFull CtrlFrameFull;
    FeedBackFrame FdbFrame;
    uint32_t FrameTimeStamp;
    uint8_t dataValid;
    uint8_t FdbFlag;

    float TargetBodyFrame[3];
    float TargetBodyVec[3];
    float TargetEarthFrame[3];
    float TargetEarthVec[3];
    float TargetTheta;

    uint8_t TargetID;
    uint8_t armor_num;

    float YawPosition;
    float PitchPosition;
    float Distance;

    float TimeConsuming;
    float Frame_dt;
    float FrameDelayToINS;
    float DelayTime;
    uint32_t conjecture_time;
    uint32_t UpdatePeriodCount;

    NTP_t NTP;

    UART_HandleTypeDef *AA_USART;
} AimAssist_t;

enum
{
    Clockwise,
    Counterclockwise
};

enum
{
    NormalSpeedSpinning,
    HighSpeedSpinning
};

enum
{
    BALANCE_2 = 2,
    OUTPOST_3,
    NORMAL_4
};

typedef struct
{
    uint8_t SpinningCountThreshold;
    uint8_t SpinningCount;
    uint8_t SpinningSpeedStatus;

    float SpinningThresholdScale;
    float VelocityThreshold;
    float HighSpinningThreshold;
} HitSpinning_t;

typedef struct
{
    uint8_t Status;
    uint8_t LastStatus;
    uint8_t isSpinning;
    uint8_t SpinningTgtValid;
    float TargetLostTime;
    float TrackingTimeStamp;
    uint32_t LostCount;
    uint32_t TrackingCount;

    uint32_t TgtSwitchPeriod_ms;
    uint32_t TgtSwitchTick_ms;

    uint8_t ArmorType;

    mat Ccb, CcbT;
    float Ccb_data[9], CcbT_data[9];
    mat Cbn, CbnT;
    float Cbn_data[9], CbnT_data[9];
    mat Rc;
    float Rc_data[9];
    mat tempMat;
    float tempMat_data[9];
    mat H, HT, M1, M2;
    float H_data[12], HT_data[12], M1_data[12], M2_data[4];
    mat ResErr, ResErrT;
    float ResErr_data[2], ResErrT_data[2];
    KalmanFilter_t TgtMotionEst;

    float BulletVelocity;

    float Distance;
    float HorizontalDistance;
    float HorizontalDistance_dot;
    float HorizontalDistance_ddot;
    float HorizontalDistanceLPF;
    float HorizontalDistance_dotLPF;
    float TgtHeight;
    float DeltaHeight;
    float TgtHeight_dot;
    float TgtHeight_ddot;

    float Position[3];
    float Velocity[3];
    float xOyVelocity;
    float Accel[3];
    float AccLPF;
    float HeightLPF;

    float ForwardTime;

    float TargetBodyFrame[3];
    float TargetEarthFrame[3];
    float PreTargetEarthFrame[3];
    float PreTargetTheta;

    float YawPosition;
    float PitchPosition;
    float YawVelocity;
    float PitchVelocity;
    float YawAccel;
    float PitchAccel;

    float TgtHeightBuff[10];
} TgtPosPredict_t;

#define DEFAULT_RADIUS 0.25f
typedef struct
{
    uint8_t ID;
    float Center[2];
    float CenterVel[2];
    float CenterAngel;
    float CenterDistance;
    float armor_height[4];
    float Height_dot;
    float theta;
    float theta_dot;
    float yaw[4];
    float r;

    KalmanFilter_t Estimator;
    mat Ccb1, Ccbinv, CcbinvT;
    float Ccb_data1[9], Ccbinv_data[9], CcbinvT_data[9];
    mat Cbn1, Cbninv, CbninvT;
    float Cbn_data1[9], Cbninv_data[9], CbninvT_data[9];
    mat BVec, CVec, EVec;
    float BVec_data[3], CVec_data[3], EVec_data[3];

    mat Ccb, CcbT;
    float Ccb_data[9], CcbT_data[9];
    mat Cbn, CbnT;
    float Cbn_data[9], CbnT_data[9];
    mat Rc;
    float Rc_data[9];
    mat tempMat, tempMat1;
    float tempMat_data[9], tempMat1_data[9];

    mat M1, M2;
    float M1_data[12], M2_data[4];
    mat ResErr, ResErrT;
    float ResErr_data[3], ResErrT_data[3];
} ChassisPosPredict_t;

typedef struct
{
    uint8_t TgtID;
    float Center[2];
    float CenterVel[2];
    float CenterAngel;
    float CenterDistance;
    float height[2];
    float height_dot;
    float theta;
    float theta_dot;
    float r[2];
    float TargetEarthFrame[3];
    float TargetYaw;
} DebugTgt_t;

typedef struct
{
    float CamX;
    float CamY;
    float CamZ;
    float axis_offset;

    float CameraYaw;
    float CameraPitch;
    float CameraRoll;

    float BulletYaw;
    float BulletPitch;
    float BulletYaw30;
    float BulletPitch30;
    float BulletYawRune;
    float BulletPitchRune;
} OffsetCorrection_t;

typedef struct
{
    uint8_t AbleToShoot;
    uint8_t ShootFreqGain;
    uint8_t UseAccel;
    uint8_t SpinningKeepShooting;
    uint32_t TargetValidTime;
    float MaxForwardTime;
    float SafeForwardTime;
    float MaxFreqGain;
    float YawAngle;
    float YawError;
    float BulletShootDelay;

    float PreTargetEarthFrame[3];
    float PreTargetTheta;
} ShootEvaluation_t;

typedef struct
{
    float Position[3];
    float Velocity[3];
    uint32_t TimeStamp;
    uint8_t TgtStatus;
} TgtPosFrame_t;

#define Tgt_FRAME_LEN 500
typedef struct
{
    TgtPosFrame_t TgtFrame[Tgt_FRAME_LEN];
    uint16_t LatestNum;
} TgtPosBuf_t;

extern AimAssist_t AimAssist;
extern TgtPosPredict_t TgtPosPredict;
extern ShootEvaluation_t ShootEvaluation;
extern TgtPosBuf_t TgtPosBuf;
extern HitSpinning_t HitSpinning;

void AimAssist_Init(UART_HandleTypeDef *huart);
void AimAssist_Task(void);
uint8_t isAbleToShoot(float *shoot_freq_gain);
float ShootFreqGain(void);

#endif
