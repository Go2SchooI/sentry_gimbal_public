#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "includes.h"

#define INS_TASK_PERIOD 1

#define INS_GetTimeline HAL_GetTick

#define Q_FRAME_LEN 120

typedef struct
{
    float q[4];
    float TimeStamp_s;
} QuaternionFrame_t;

typedef struct
{
    float q[4]; // ��Ԫ������ֵ

    float Gyro[3];
    float Accel[3];
    float MotionAccel_b[3];
    float MotionAccel_n[3];

    float AccelLPF;

    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    QuaternionFrame_t qFrame[Q_FRAME_LEN];
} INS_t;

typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;
extern float RefTemp;

void INS_Init(void);
void INS_Task(void);
void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

void QuaternionSlerp(float *qStart, float *qEnd, float *qResult, float t);
void Insert_qFrame(QuaternionFrame_t *q_frame, float *q, float time_stamp_s);
uint16_t Find_qFrame(QuaternionFrame_t *q_frame, float match_time_stamp_s);

#endif
