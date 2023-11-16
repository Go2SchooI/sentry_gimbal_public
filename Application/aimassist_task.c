#include "aimassist_task.h"
#include "kalman_filter.h"
#include "remote_control.h"

AimAssist_t AimAssist = {0};
TgtPosPredict_t TgtPosPredict = {0};
ShootEvaluation_t ShootEvaluation = {0};
HitSpinning_t HitSpinning = {0};
TgtPosBuf_t TgtPosBuf = {0};
OffsetCorrection_t OffsetCorrection;
ChassisPosPredict_t ChassisPosPredict = {0};
DebugTgt_t DebugTgt;

/**************************** Aim Assist KF Data ******************************/
float TgtMotionEst_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1};
float ChassisEst_F[64] = {
    1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 1};
float TgtMotionEst_P[36];
float TgtMotionEst_Pinit[36] = {
    0.001, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.001, 0, 0, 0,
    0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0.001, 0,
    0, 0, 0, 0, 0, 0.1};
float ChassisEst_P[64];
float ChassisEst_Pinit[64] = {
    0.0005, 0, 0, 0, 0, 0, 0, 0,
    0, 0.0005, 0, 0, 0, 0, 0, 0,
    0, 0, 0.0005, 0, 0, 0, 0, 0,
    0, 0, 0, 0.0005, 0, 0, 0, 0,
    0, 0, 0, 0, 0.01, 0, 0, 0,
    0, 0, 0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0, 0, 0.001, 0,
    0, 0, 0, 0, 0, 0, 0, 0.001};

float TgtMotionEst_Sigma[3] = {1, 1, 0.5};
// float TgtMotionEst_Sigma[3] = {10, 10, 1};

float TgtMotionEst_R[9] = {0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005};
float ChassisEst_Sigma[4] = {0.0003, 0.0003, 0.1, 0.0001};
float TgtMotionEst_Q[36] = {
    0.1, 0, 0, 0, 0, 0,
    0, 50, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 50, 0, 0,
    0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 50};
float ChassisEst_Q[64] = {
    0.1, 0, 0, 0, 0, 0, 0, 0,
    0, 50, 0, 0, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0, 0, 0,
    0, 0, 0, 50, 0, 0, 0, 0,
    0, 0, 0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0, 0, 50, 0, 0,
    0, 0, 0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 0, 0, 50};
float ChassisEst_R[9] = {
    0.005, 0, 0,
    0, 0.0001, 0,
    0, 0, 0.01};

/*************************** Aim Assist Data Buf ******************************/
static uint8_t AimAssist_Rx_Buf[AimAssist_RX_BUF_NUM];
uint8_t AimAssist_Rx_Buf_Buf[AimAssist_RX_BUF_NUM];
static uint8_t AimAssist_Tx_Buf[AimAssist_TX_BUF_NUM];
ControlFrame CtrlFrameTemp;
static uint8_t Calibration_Rx_Buf[CALIBRATION_RX_BUF_NUM];
static CalibrationFrame_t CalibrationFrame;

/************************** Aim Assist Time Stamp *****************************/
uint32_t AimAssist_DWT_Count = 0;
static float dt = 0, t = 0;

static void HitSpinning_Init(void);
static void ChassisPosPredict_Init(void);
static void ShootEvaluation_Init(void);
static void TgtMotionEst_Init(void);
static void TgtMotionEst_Update(float dt);
static void Set_AimAssistMode(void);
static float Ballistic_Compensation(float x, float y, float y_dot, float v, float *forwardTime);
static float Ballistic_Model(float x, float v, float pitch);
static void Check_Target_Status(float dt);
static void AimAssist_Get_Target(void);
uint8_t AimAssist_Shoot_Evaluation(void);
static void AimAssist_Send_FdbFrame(void);
static void GetTargetPosition(ControlFrame CtrlFrameTemp, uint8_t *buff);
static void GetTargetPositionFull(ControlFrameFull CtrlFrameTempFull, uint8_t *buff);
static void CalibrationCallback(CalibrationFrame_t caliFrameTemp);
static void CameraOffsetCorrection(float *inputPos, float *outputPos, float *inputVec, float *outputVec);
static void TgtMotionEst_Tuning(KalmanFilter_t *kf);
static void TgtMotionEst_Set_R(KalmanFilter_t *kf);
static void TgtMotionEst_ChiSquare_Test(KalmanFilter_t *kf);
static void TgtMotionEst_Reset(float *new_position);
static void ChassisEst_Init(void);
static void ChassisEst_Update(float dt);
static void ChassisEst_Tuning(KalmanFilter_t *kf);
static void ChassisEst_Set_R_H(KalmanFilter_t *kf);
static void ChassisEst_ChiSquare_Test(KalmanFilter_t *kf);
static void ChassisEst_xhat_Limit(KalmanFilter_t *kf);
static void ChassisEst_Reset(float *new_position, float new_yaw, uint8_t tgtID);
static void Estimator_Debug(float dt);
static void AimAssist_Debug(void);
static float Angle_Process(float angle, float theta, uint8_t armor_num);
static int8_t Angle_Check(float angle, float theta, uint8_t armor_num);
static float invSqrt(float x);
static uint8_t is_Target_Spinning(void);
/******************************** Task Func ***********************************/
void AimAssist_Init(UART_HandleTypeDef *huart)
{
    AimAssist.AA_USART = huart;

    AimAssist.Mode = NORMAL;
    AimAssist.miniPC_Online = 0; // 0: offline, 1: online

#if ENABLE_CALIBRATION
    USART_IDLE_Init(huart, Calibration_Rx_Buf, CALIBRATION_RX_BUF_NUM);
#else
    USART_IDLE_Init(huart, AimAssist_Rx_Buf, AimAssist_RX_BUF_NUM);
#endif
    HitSpinning_Init();
    ShootEvaluation_Init();

    TgtPosPredict.ForwardTime = 0.002f;

    TgtPosPredict.BulletVelocity = 28.5f;

    TgtPosPredict.AccLPF = 0.025;
    TgtPosPredict.HeightLPF = 0.025;
    TgtPosPredict.HorizontalDistanceLPF = 0.005;
    TgtPosPredict.HorizontalDistance_dotLPF = 0.1;

    OffsetCorrection.CamX = 0;
    OffsetCorrection.CamY = 125 / 1000.0f;
    OffsetCorrection.CamZ = 50 / 1000.0f;
    OffsetCorrection.axis_offset = 0;
    OffsetCorrection.CameraYaw = 0.0f;
    OffsetCorrection.CameraPitch = 0.0f;
    OffsetCorrection.CameraRoll = 0.0f;
    OffsetCorrection.BulletYaw = -0.5f;
    OffsetCorrection.BulletPitch = 0.0f;

    AimAssist.FrameDelayToINS = 0.0035f;

    TgtMotionEst_Init();
    ChassisPosPredict_Init();
}

static void ChassisPosPredict_Init(void)
{
    ChassisEst_Init();
}

static void HitSpinning_Init(void)
{
    HitSpinning.SpinningThresholdScale = 3;
    HitSpinning.SpinningCountThreshold = 2;
}

static void ShootEvaluation_Init(void)
{
    ShootEvaluation.SpinningKeepShooting = 0;
    ShootEvaluation.UseAccel = 0;             // 是否使用加速度进行评估
    ShootEvaluation.MaxForwardTime = 0.35;    // 最大预测时间，预测时间超过此值则认为难以击中不进行射击
    ShootEvaluation.SafeForwardTime = 0.15;   // 安全预测时间，预测时间低于此值则认为有极高命中率
    ShootEvaluation.MaxFreqGain = 10;         // 最大额外射频
    ShootEvaluation.BulletShootDelay = 0.05f; // 发弹延迟，对打陀螺评估至关重要
}

void AimAssist_Task(void)
{
    dt = DWT_GetDeltaT(&AimAssist_DWT_Count);
    t += dt;

    Set_AimAssistMode();

    if (OfflineDebug)
        Estimator_Debug(dt);
    // TgtPosPredict.isSpinning = 1;
    if (TgtPosPredict.isSpinning == 1)
        ChassisEst_Update(dt);
    else
        TgtMotionEst_Update(dt);

    Check_Target_Status(dt);

    AimAssist_Get_Target();

    is_Target_Spinning();
    AimAssist_Shoot_Evaluation();

    AimAssist_Send_FdbFrame();

    AimAssist.LastMode = AimAssist.Mode;
    TgtPosPredict.LastStatus = TgtPosPredict.Status;
}

void InsertTgtFrame(TgtPosBuf_t *tgtBuf, float *position, float *velocity, uint32_t time_stamp, float status)
{
    if (tgtBuf->LatestNum == Tgt_FRAME_LEN - 1)
        tgtBuf->LatestNum = 0;
    else
        tgtBuf->LatestNum++;

    tgtBuf->TgtFrame[tgtBuf->LatestNum].TimeStamp = time_stamp;
    for (uint8_t i = 0; i < 3; i++)
    {
        tgtBuf->TgtFrame[tgtBuf->LatestNum].Position[i] = position[i];
        tgtBuf->TgtFrame[tgtBuf->LatestNum].Velocity[i] = velocity[i];
    }
    tgtBuf->TgtFrame[tgtBuf->LatestNum].TgtStatus = status;
}

uint16_t FindLastTrackingFrame(TgtPosBuf_t *tgtBuf)
{
    uint16_t num;

    num = tgtBuf->LatestNum;

    for (uint16_t i = 0; i < Tgt_FRAME_LEN; i++)
    {
        if (num == 0)
            num = Tgt_FRAME_LEN - 1;
        else
            num--;
        if (tgtBuf->TgtFrame[num].TgtStatus == TgtTracking)
            break;
    }

    return num;
}

static void AimAssist_Get_Target(void)
{
    static float lastBulletSpeed = 0;
    static uint32_t count, HighSpinningCount = 0;
    static float pre_target_theta, temp_yaw[4];
    static float min_distance, temp_position_x[4], temp_position_y[4], temp_position_z[4], temp_distance[4];
    static uint8_t min_num;

    if (TgtPosPredict.Status == TgtTracking || TgtPosPredict.Status == TgtConjecture)
    {
        TgtPosPredict.PitchPosition = Ballistic_Compensation(TgtPosPredict.HorizontalDistance,
                                                             TgtPosPredict.TgtHeight,
                                                             0.0f,
                                                             TgtPosPredict.BulletVelocity, &TgtPosPredict.ForwardTime);
        // !!!WARNING!!!
        // 判断变量是否为 nan，若为 nan 会导致云台无法控制，该处理方法在代码中频繁使用
        if (!isnormal(TgtPosPredict.PitchPosition))
            TgtPosPredict.PitchPosition = 0;
        if (!isnormal(TgtPosPredict.ForwardTime))
            TgtPosPredict.ForwardTime = 0;

        if (TgtPosPredict.isSpinning == 1)
        {
            if (fabsf(ChassisPosPredict.theta_dot) > 8.0f)
                HighSpinningCount++;
            else
                HighSpinningCount = 0;

            if (HighSpinningCount > 300)
                HitSpinning.SpinningSpeedStatus = HighSpeedSpinning;
            else
                HitSpinning.SpinningSpeedStatus = NormalSpeedSpinning;

            if (UseTwicePredict)
            {
                pre_target_theta = Angle_Process(ChassisPosPredict.theta + ChassisPosPredict.theta_dot * TgtPosPredict.ForwardTime, AimAssist.TargetTheta, AimAssist.armor_num);
                // pre_target_theta = STD_RADIAN(ChassisPosPredict.theta + ChassisPosPredict.theta_dot * TgtPosPredict.ForwardTime);
                min_distance = 50.0f;
                for (uint8_t i = 0; i < AimAssist.armor_num; i++)
                {
                    temp_yaw[i] = STD_RADIAN(pre_target_theta + i * 2 * PI / AimAssist.armor_num);
                    temp_position_x[i] = ChassisPosPredict.Center[X] + ChassisPosPredict.CenterVel[X] * TgtPosPredict.ForwardTime - (1 - HitSpinning.SpinningSpeedStatus) * ChassisPosPredict.r * arm_cos_f32(temp_yaw[i]);
                    temp_position_y[i] = ChassisPosPredict.Center[Y] + ChassisPosPredict.CenterVel[Y] * TgtPosPredict.ForwardTime - (1 - HitSpinning.SpinningSpeedStatus) * ChassisPosPredict.r * arm_sin_f32(temp_yaw[i]);
                    arm_sqrt_f32(temp_position_x[i] * temp_position_x[i] + temp_position_y[i] * temp_position_y[i], &temp_distance[i]);
                    if (temp_distance[i] < min_distance)
                    {
                        min_distance = temp_distance[i];
                        min_num = i;
                    }
                }

                TgtPosPredict.PreTargetTheta = temp_yaw[min_num];
                TgtPosPredict.PreTargetEarthFrame[X] = temp_position_x[min_num];
                TgtPosPredict.PreTargetEarthFrame[Y] = temp_position_y[min_num];
                TgtPosPredict.PreTargetEarthFrame[Z] = ChassisPosPredict.armor_height[min_num];

                TgtPosPredict.PitchPosition = Ballistic_Compensation(sqrtf(TgtPosPredict.PreTargetEarthFrame[X] * TgtPosPredict.PreTargetEarthFrame[X] + TgtPosPredict.PreTargetEarthFrame[Y] * TgtPosPredict.PreTargetEarthFrame[Y]),
                                                                     TgtPosPredict.PreTargetEarthFrame[Z], 0,
                                                                     TgtPosPredict.BulletVelocity, &TgtPosPredict.ForwardTime);
                if (!isnormal(TgtPosPredict.PitchPosition))
                    TgtPosPredict.PitchPosition = 0;
                if (!isnormal(TgtPosPredict.ForwardTime))
                    TgtPosPredict.ForwardTime = 0;
            }

            TgtPosPredict.ForwardTime += AimAssist.FrameDelayToINS;
            TgtPosPredict.ForwardTime = float_constrain(TgtPosPredict.ForwardTime, 0.005f, 0.75f);

            pre_target_theta = Angle_Process(ChassisPosPredict.theta + ChassisPosPredict.theta_dot * TgtPosPredict.ForwardTime, AimAssist.TargetTheta, AimAssist.armor_num);
            // pre_target_theta = STD_RADIAN(ChassisPosPredict.theta + ChassisPosPredict.theta_dot * TgtPosPredict.ForwardTime);
            min_distance = 50.0f;
            for (uint8_t i = 0; i < AimAssist.armor_num; i++)
            {
                temp_yaw[i] = STD_RADIAN(pre_target_theta + i * 2 * PI / AimAssist.armor_num);
                temp_position_x[i] = ChassisPosPredict.Center[X] + ChassisPosPredict.CenterVel[X] * TgtPosPredict.ForwardTime - (1 - HitSpinning.SpinningSpeedStatus) * ChassisPosPredict.r * arm_cos_f32(temp_yaw[i]);
                temp_position_y[i] = ChassisPosPredict.Center[Y] + ChassisPosPredict.CenterVel[Y] * TgtPosPredict.ForwardTime - (1 - HitSpinning.SpinningSpeedStatus) * ChassisPosPredict.r * arm_sin_f32(temp_yaw[i]);
                arm_sqrt_f32(temp_position_x[i] * temp_position_x[i] + temp_position_y[i] * temp_position_y[i], &temp_distance[i]);
                if (temp_distance[i] < min_distance)
                {
                    min_distance = temp_distance[i];
                    min_num = i;
                }
            }
            // TgtPosPredict.PreTargetTheta = temp_yaw[min_num];
            TgtPosPredict.PreTargetEarthFrame[X] = temp_position_x[min_num];
            TgtPosPredict.PreTargetEarthFrame[Y] = temp_position_y[min_num];
            TgtPosPredict.PreTargetEarthFrame[Z] = ChassisPosPredict.armor_height[min_num];
        }
        else
        {
            if (UseTwicePredict)
            {
                for (uint8_t i = 0; i < 3; i++)
                {
                    TgtPosPredict.PreTargetEarthFrame[i] = TgtPosPredict.Position[i] + TgtPosPredict.Velocity[i] * TgtPosPredict.ForwardTime + TgtPosPredict.Accel[i] * TgtPosPredict.ForwardTime * TgtPosPredict.ForwardTime * 0.5f * UseAccelPredict;
                }
                // ���μ��㵯������
                TgtPosPredict.PitchPosition = Ballistic_Compensation(sqrtf(TgtPosPredict.PreTargetEarthFrame[X] * TgtPosPredict.PreTargetEarthFrame[X] + TgtPosPredict.PreTargetEarthFrame[Y] * TgtPosPredict.PreTargetEarthFrame[Y]),
                                                                     TgtPosPredict.TgtHeight, 0,
                                                                     TgtPosPredict.BulletVelocity, &TgtPosPredict.ForwardTime);
                if (!isnormal(TgtPosPredict.PitchPosition))
                    TgtPosPredict.PitchPosition = 0;
                if (!isnormal(TgtPosPredict.ForwardTime))
                    TgtPosPredict.ForwardTime = 0;
            }

            TgtPosPredict.ForwardTime += AimAssist.FrameDelayToINS;
            TgtPosPredict.ForwardTime = float_constrain(TgtPosPredict.ForwardTime, 0.005f, 0.75f);
            // �����ⲻ����Ԥ��
            if (TgtPosPredict.HorizontalDistance > 8.0f)
                TgtPosPredict.ForwardTime = 0;
            for (uint8_t i = 0; i < 3; i++)
            {
                TgtPosPredict.PreTargetEarthFrame[i] = TgtPosPredict.Position[i] + TgtPosPredict.Velocity[i] * TgtPosPredict.ForwardTime + TgtPosPredict.Accel[i] * TgtPosPredict.ForwardTime * TgtPosPredict.ForwardTime * 0.5f * UseAccelPredict;
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
            TgtPosPredict.PreTargetEarthFrame[i] = TgtPosPredict.Position[i];
    }

    TgtPosPredict.YawPosition = atan2f(-TgtPosPredict.PreTargetEarthFrame[X], TgtPosPredict.PreTargetEarthFrame[Y]) * RADIAN_COEF;
    if (!isnormal(TgtPosPredict.YawPosition))
        TgtPosPredict.YawPosition = 0;

    TgtPosPredict.YawPosition += OffsetCorrection.BulletYaw;

    if (TgtPosPredict.Status != TgtLost)
        TgtPosPredict.PitchPosition += OffsetCorrection.BulletPitch;

    TgtPosPredict.PitchPosition = float_constrain(TgtPosPredict.PitchPosition,
                                                  INS.Pitch - PITCH_MAX_DEG,
                                                  INS.Pitch + PITCH_MAX_DEG);

    // 运算修正
    if (TgtPosPredict.YawPosition > 180.0f)
        TgtPosPredict.YawPosition -= -360.0f;
    if (TgtPosPredict.YawPosition < -180.0f)
        TgtPosPredict.YawPosition += 360.0f;

    /*float MaxYaw = INS.Yaw + YAW_MAX_DEG;
    float MinYaw = INS.Yaw - YAW_MAX_DEG;

    if (TgtPosPredict.YawPosition > INS.Yaw && MaxYaw < 180)
        TgtPosPredict.YawPosition = float_constrain(TgtPosPredict.YawPosition, -180, MaxYaw);
    if (TgtPosPredict.YawPosition < INS.Yaw && MinYaw > -180)
        TgtPosPredict.YawPosition = float_constrain(TgtPosPredict.YawPosition, MinYaw, 180);*/

    // 记录数据，用于判断陀螺
    if (TgtPosPredict.Status != TgtLost && count % 2 == 0)
    {
        InsertTgtFrame(&TgtPosBuf, TgtPosPredict.TargetEarthFrame, TgtPosPredict.Velocity, INS_GetTimeline(), TgtPosPredict.Status);
    }
    count++;
}

static void Check_Target_Status(float dt)
{
    static uint32_t count = 0;
    // 若当前时刻距离上一次接收到 minipc 数据的时间超过 1s，则认为 minipc 离线
    if (INS_GetTimeline() - AimAssist.FrameTimeStamp > 1000)
    {
        AimAssist.Status = TargetLost;
        AimAssist.miniPC_Online = 0;
        TgtPosPredict.Status = TgtLost;
        TgtPosPredict.TrackingCount = 0;

        // 卡尔曼滤波器复位
        TgtMotionEst_Reset(NULL);
        ChassisEst_Reset(NULL, 0, 0);
#if ENABLE_CALIBRATION
        USART_IDLE_Init(AimAssist.AA_USART, Calibration_Rx_Buf, CALIBRATION_RX_BUF_NUM);
#else
        USART_IDLE_Init(AimAssist.AA_USART, AimAssist_Rx_Buf, AimAssist_RX_BUF_NUM);
#endif
    }

    // 目标跟踪过程中 TrackingCount 自加
    if (TgtPosPredict.Status == TgtTracking)
    {
        // 若上一周期状态为目标丢失，即此周期刚收到第一帧装甲板数据，则将滤波器复位至当前位置
        if (TgtPosPredict.LastStatus == TgtLost)
        {
            ChassisEst_Reset(AimAssist.TargetEarthFrame, AimAssist.TargetTheta, AimAssist.CtrlFrameFull.flg);
            TgtMotionEst_Reset(AimAssist.TargetEarthFrame);
            TgtPosPredict.TargetEarthFrame[X] = AimAssist.TargetEarthFrame[X];
            TgtPosPredict.TargetEarthFrame[Y] = AimAssist.TargetEarthFrame[Y];
            TgtPosPredict.TargetEarthFrame[Z] = AimAssist.TargetEarthFrame[Z];
        }
        TgtPosPredict.TrackingCount++;
    }
    // 若此周期发生装甲板切换，则复位 TrackingCount
    if (TgtPosPredict.Status == TgtSwitch)
    {
        TgtPosPredict.TrackingCount = 0;

        // 记录装甲板切换周期，用于陀螺目标识别
        if (TgtPosPredict.LastStatus != TgtSwitch)
        {
            TgtPosPredict.TgtSwitchPeriod_ms = INS_GetTimeline() - TgtPosPredict.TgtSwitchTick_ms;
            TgtPosPredict.TgtSwitchTick_ms = INS_GetTimeline();
        }

        // 目标切换 复位运动信息
        // 卡尔曼滤波器复位
        ChassisEst_Reset(AimAssist.TargetEarthFrame, AimAssist.TargetTheta, AimAssist.CtrlFrameFull.flg);
        TgtMotionEst_Reset(AimAssist.TargetEarthFrame);

        TgtPosPredict.TargetEarthFrame[X] = AimAssist.TargetEarthFrame[X];
        TgtPosPredict.TargetEarthFrame[Y] = AimAssist.TargetEarthFrame[Y];
        TgtPosPredict.TargetEarthFrame[Z] = AimAssist.TargetEarthFrame[Z];
    }

    count++;
}

static uint8_t is_Target_Spinning(void)
{
    static uint32_t TgtSwitchTimeStamp, spinning_change_timestamp;
    static uint32_t outSpinningCount;
    static float SpinningSwitchPeriodThreshold = 0.4;
    static uint8_t armor_valid, last_spinning;

    if (AimAssist.armor_num == OUTPOST_3)
    {
        TgtPosPredict.isSpinning = 1;
        return TgtPosPredict.isSpinning;
    }

    HitSpinning.SpinningThresholdScale = (AimAssist.armor_num == NORMAL_4) ? 3 : 8;
    SpinningSwitchPeriodThreshold = HitSpinning.SpinningThresholdScale * TgtPosPredict.Distance / TgtPosPredict.BulletVelocity * 1000.0f;

    if ((AimAssist.armor_num == 4 && TgtPosPredict.Status == TgtLost) ||
        (AimAssist.armor_num == 2 && TgtPosPredict.LostCount > 600))
        armor_valid = 0;
    else
        armor_valid = 1;

    if (TgtPosPredict.isSpinning == 0)
    {
        last_spinning = TgtPosPredict.isSpinning;
        if (TgtPosPredict.Status == TgtSwitch && TgtPosPredict.LastStatus != TgtSwitch && armor_valid)
        {
            if (INS_GetTimeline() - TgtSwitchTimeStamp < SpinningSwitchPeriodThreshold *
                                                             (1 + TgtPosPredict.isSpinning * 0.25f))
                HitSpinning.SpinningCount++;
            else
                HitSpinning.SpinningCount = HitSpinning.SpinningCountThreshold - 1;

            if (HitSpinning.SpinningCount >= HitSpinning.SpinningCountThreshold)
            {
                TgtPosPredict.isSpinning = 1;
                TgtPosPredict.SpinningTgtValid = 1;
            }
            else
                TgtPosPredict.isSpinning = 0;

            TgtSwitchTimeStamp = INS_GetTimeline();
        }
    }
    else
    {
        if (last_spinning == 0)
            spinning_change_timestamp = INS_GetTimeline();

        if (fabsf(ChassisPosPredict.theta_dot) < 1.5f && INS_GetTimeline() - spinning_change_timestamp > 1500)
            outSpinningCount++;
        else
            outSpinningCount = 0;

        if (outSpinningCount > 300)
        {
            TgtPosPredict.isSpinning = 0;
            outSpinningCount = 0;
        }

        last_spinning = TgtPosPredict.isSpinning;
    }

    return TgtPosPredict.isSpinning;
}

static void Set_AimAssistMode(void)
{
    static uint8_t last_keyboard;
    static uint32_t jPressTimeStamp, kPressTimeStamp, lPressTimeStamp;

    // if (DataFromAerial[8] == 'J' && last_keyboard != 'J')
    //     jPressTimeStamp = INS_GetTimeline();

    // if (DataFromAerial[8] == 'K' && last_keyboard != 'K')
    //     kPressTimeStamp = INS_GetTimeline();

    // if (DataFromAerial[8] == 'L' && last_keyboard != 'L')
    //     kPressTimeStamp = INS_GetTimeline();

    // if (INS_GetTimeline() - jPressTimeStamp < 10000 || enemy_outpost_HP > 200)
    //     AimAssist.Mode = HIT_OUTPOST;
    // else if (INS_GetTimeline() - kPressTimeStamp < 10000)
    //     AimAssist.Mode = HIT_1_2;
    // else if (INS_GetTimeline() - lPressTimeStamp < 10000)
    //     AimAssist.Mode = HIT_6;
    // else
    AimAssist.Mode = NORMAL;

    last_keyboard = DataFromAerial[8];
}

static void AimAssist_Send_FdbFrame(void)
{
    static float TxTimeStamp = 0, TxTimeStamp1 = 0;
    static float JudgeTxTimeStamp = 0;
    static uint16_t Hit;

    // if (JudgeRxDataValid && JudgeRxData[1])
    // {
    //     JudgeTxTimeStamp = t;
    //     JudgeRxDataValid = 0;
    // }
    // if (t - JudgeTxTimeStamp < 10)
    //     Hit = 0x01;
    // else
    Hit = 0x01;

    AimAssist.FdbFrame.FDBF_SOF = 0x66;
    if (robot_state.robot_id <= 7)
        AimAssist.FdbFrame.myteam = RED;
    else
        AimAssist.FdbFrame.myteam = BLUE;
    AimAssist.FdbFrame.pitch = INS.Pitch * 1000.0f;
    AimAssist.FdbFrame.yaw = INS.Yaw * 1000.0f;
    AimAssist.FdbFrame.roll = INS.Roll * 1000.0f;

    AimAssist.FdbFrame.bullet_speed = TgtPosPredict.BulletVelocity * 10;
    AimAssist.FdbFrame.time_stamp_ms = INS_GetTimeline();

    if (enemy_outpost_HP > 1)
        AimAssist.FdbFrame.outpost_alive = 1;
    else
        AimAssist.FdbFrame.outpost_alive = 0;

    // if (Gimbal.aim_status == find_target || Gimbal.aim_status == wait_command)
    //     AimAssist.FdbFrame.status = 1;
    // else
    //     AimAssist.FdbFrame.status = 0;

    AimAssist.FdbFrame.hit2 = Hit; // 0不打工程 1打工程
    AimAssist.FdbFrame.mode = AimAssist.Mode;
    AimAssist.FdbFrame.FDBF_EOF = 0x88;
    memcpy(AimAssist_Tx_Buf, &AimAssist.FdbFrame, AimAssist_TX_BUF_NUM);
    if (t - TxTimeStamp1 > 0.05f && AimAssist.FdbFlag == 0)
    {
        HAL_UART_Transmit_DMA(AimAssist.AA_USART, AimAssist_Tx_Buf, AimAssist_TX_BUF_NUM);
        TxTimeStamp1 = t;
    }
    // 注意，应当避免在发送数据的过程中再次发送数据，否则会导致数据丢失，或卡死
    if (AimAssist.FdbFlag && t - TxTimeStamp1 > 0.002f)
    {
        AimAssist.FdbFlag = 0;
        if (HAL_UART_GetState(AimAssist.AA_USART) != HAL_UART_STATE_BUSY_TX &&
            (__HAL_UART_GET_FLAG(AimAssist.AA_USART, UART_FLAG_TC) == SET))
            HAL_UART_Transmit_DMA(AimAssist.AA_USART, AimAssist_Tx_Buf, AimAssist_TX_BUF_NUM);
    }
}

uint8_t AimAssist_Shoot_Evaluation(void)
{
    static float pre_target_theta, temp_yaw[4];
    static float min_distance, temp_position_x[4], temp_position_y[4], temp_position_z[4], temp_distance[4];
    static uint8_t min_num;
    // 设计评估是自瞄的重要组成部分，什么时候应当射击，应当以何种射频射击，与瞄准本身同样重要
    if (ShootEvaluation.MaxForwardTime <= ShootEvaluation.SafeForwardTime)
        ShootEvaluation.MaxForwardTime = ShootEvaluation.SafeForwardTime + 0.2f;
    ShootEvaluation.ShootFreqGain = 0;

    // 若目标丢失则不进行射击
    if (TgtPosPredict.Status == TgtLost || INS_GetTimeline() - ShootEvaluation.TargetValidTime > 25)
    {
        ShootEvaluation.ShootFreqGain = 0;
        ShootEvaluation.AbleToShoot = 0;
        return ShootEvaluation.AbleToShoot;
    }

    // 若预测时间过长则不进行射击
    if (TgtPosPredict.ForwardTime > ShootEvaluation.MaxForwardTime && TgtPosPredict.isSpinning == 0)
    {
        ShootEvaluation.ShootFreqGain = -3;
        ShootEvaluation.AbleToShoot = 1;
        return ShootEvaluation.AbleToShoot;
    }

    if (TgtPosPredict.ForwardTime > (ShootEvaluation.MaxForwardTime - 0.05f) && TgtPosPredict.isSpinning == 1 && AimAssist.TargetID == 7)
    {
        ShootEvaluation.ShootFreqGain = 0;
        ShootEvaluation.AbleToShoot = 0;
        return ShootEvaluation.AbleToShoot;
    }

    float forwardTime;
    // 计算陀螺目标预测时间需要考虑射击延迟
    if (HitSpinning.SpinningSpeedStatus == HighSpeedSpinning)
        forwardTime = TgtPosPredict.ForwardTime + ShootEvaluation.BulletShootDelay;
    else
        forwardTime = TgtPosPredict.ForwardTime;

    // 根据预测时间和目标状态信息计算目标预测位置
    if (TgtPosPredict.isSpinning == 1)
    {
        pre_target_theta = Angle_Process(ChassisPosPredict.theta + ChassisPosPredict.theta_dot * forwardTime, AimAssist.TargetTheta, AimAssist.armor_num);
        min_distance = 50.0f;
        for (uint8_t i = 0; i < AimAssist.armor_num; i++)
        {
            temp_yaw[i] = STD_RADIAN(pre_target_theta + i * 2 * PI / AimAssist.armor_num);
            temp_position_x[i] = ChassisPosPredict.Center[X] + ChassisPosPredict.CenterVel[X] * forwardTime - ChassisPosPredict.r * arm_cos_f32(temp_yaw[i]);
            temp_position_y[i] = ChassisPosPredict.Center[Y] + ChassisPosPredict.CenterVel[Y] * forwardTime - ChassisPosPredict.r * arm_sin_f32(temp_yaw[i]);
            arm_sqrt_f32(temp_position_x[i] * temp_position_x[i] + temp_position_y[i] * temp_position_y[i], &temp_distance[i]);
            if (temp_distance[i] < min_distance)
            {
                min_distance = temp_distance[i];
                min_num = i;
            }
        }
        ShootEvaluation.PreTargetEarthFrame[X] = temp_position_x[min_num];
        ShootEvaluation.PreTargetEarthFrame[Y] = temp_position_y[min_num];
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            ShootEvaluation.PreTargetEarthFrame[i] = TgtPosPredict.Position[i] + TgtPosPredict.Velocity[i] * forwardTime;
        }
    }

    ShootEvaluation.YawAngle = atan2f(-ShootEvaluation.PreTargetEarthFrame[X], ShootEvaluation.PreTargetEarthFrame[Y]) * RADIAN_COEF;
    ShootEvaluation.YawAngle += OffsetCorrection.BulletYaw;

    if (ShootEvaluation.YawAngle > 180.0f)
        ShootEvaluation.YawAngle -= -360.0f;
    if (ShootEvaluation.YawAngle < -180.0f)
        ShootEvaluation.YawAngle += 360.0f;

    // 计算 yaw 轴指向与预测结果的误差
    if (INS.Yaw - ShootEvaluation.YawAngle > 180)
        ShootEvaluation.YawError = (INS.Yaw - ShootEvaluation.YawAngle - 360) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;
    else if (INS.Yaw - ShootEvaluation.YawAngle < -180)
        ShootEvaluation.YawError = (INS.Yaw - ShootEvaluation.YawAngle + 360) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;
    else
        ShootEvaluation.YawError = (INS.Yaw - ShootEvaluation.YawAngle) / RADIAN_COEF * TgtPosPredict.HorizontalDistance;
    ShootEvaluation.YawError = ShootEvaluation.YawError * 1000.0f;
    // 若预测时间足够小，则认为命中率很高，若 yaw 轴指向误差小于装甲板尺寸，直接采用最高射频射击
    if (forwardTime < ShootEvaluation.SafeForwardTime)
    {
        switch (TgtPosPredict.ArmorType)
        {
        case SMALL_ARMOR:
            if (fabsf(ShootEvaluation.YawError) < 135.0f / 2.0f * 1.15f)
            {
                ShootEvaluation.ShootFreqGain = ShootEvaluation.MaxFreqGain;
                ShootEvaluation.AbleToShoot = 1;
                return ShootEvaluation.AbleToShoot;
            }
            break;

        case BIG_ARMOR:
            if (fabsf(ShootEvaluation.YawError) < 230.0f / 2.0f * 1.15f)
            {
                ShootEvaluation.ShootFreqGain = ShootEvaluation.MaxFreqGain;
                ShootEvaluation.AbleToShoot = 1;
                return ShootEvaluation.AbleToShoot;
            }
            break;
        }
    }

    // 若 yaw 轴指向误差大于装甲板尺寸 1.2 倍，则不进行射击
    // 对于陀螺目标略微放宽误差阈值
    switch (TgtPosPredict.ArmorType)
    {
    case SMALL_ARMOR:
        if (fabsf(ShootEvaluation.YawError) > 135.0f / 2.0f * 1.35f *
                                                  (1 - 0.15f * TgtPosPredict.isSpinning))
        {
            ShootEvaluation.ShootFreqGain = 6;
            ShootEvaluation.AbleToShoot = 0;
            return ShootEvaluation.AbleToShoot;
        }
        break;

    case BIG_ARMOR:
        if (fabsf(ShootEvaluation.YawError) > 230.0f / 2.0f * 1.35f *
                                                  (1 - 0.15f * TgtPosPredict.isSpinning))
        {
            ShootEvaluation.ShootFreqGain = 6;
            ShootEvaluation.AbleToShoot = 0;
            return ShootEvaluation.AbleToShoot;
        }
        break;
    }

    // 对于不远不近且 yaw 轴指向误差不大的一般目标，射频由预测时间计算得到
    ShootEvaluation.AbleToShoot = 1;

    return ShootEvaluation.AbleToShoot;
}

/********************************* Ballistic **********************************/
static float Ballistic_Compensation(float x, float y, float y_dot, float v, float *forwardTime)
{
    static float k;
    static float temp_x, temp_y, dy;
    static float pitch;
    static float gain = 0.6;
    static float t;
    static float heightGain = 0.1;
    static uint8_t useyModel = 0;
    static float t0, vy0, C, h, top_h;
    static float useYdot = 0;

    temp_x = x;
    temp_y = y;
    k = bulletModelK;
    for (int i = 0; i < 20; i++)
    {
        pitch = atan2f(temp_y, temp_x);
        t = Ballistic_Model(x, v, pitch);
        // 迭代出错保护
        if (t < 1e-4f)
        {
            *forwardTime = 0;
            return atan2f(y, x) * RADIAN_COEF;
        }

        if (useyModel)
        {
            // 考虑数值方向空气阻力模型，不咋好用
            vy0 = v * arm_sin_f32(pitch);
            if (pitch > 0)
            {
                t0 = atanf(vy0 / sqrtf(9.8f / k)) / sqrtf(k * 9.8f);
                if (t0 > t)
                {
                    C = -1 / k * logf(fabsf(arm_cos_f32(atanf(vy0 / sqrtf(9.8f / k)))));
                    h = 1 / k * logf(fabsf(arm_cos_f32(atanf(vy0 * sqrtf(k / 9.8f)) - sqrtf(9.8f * k) * t))) + C;
                }
                else
                {
                    C = -1 / k * logf(fabsf(arm_cos_f32(atanf(vy0 / sqrtf(9.8f / k)))));
                    top_h = 1 / k * logf(fabsf(arm_cos_f32(atanf(vy0 * sqrtf(k / 9.8f)) - sqrtf(9.8f * k) * t0))) + C;
                    C = logf(2) / k;
                    h = top_h + (1 / k * ((t - t0) * sqrtf(9.8f * k) - logf(fabsf(expf(2 * sqrtf(9.8f * k) * (t - t0)) + 1))) + C);
                }
                dy = y + y_dot * t * useYdot - h;
            }
            else
                dy = y + y_dot * t * useYdot - (v * arm_sin_f32(pitch) * t - 4.9f * t * t);
        }
        else
        {
            // 高度修正，纯纯经验公式，应当优化
            if (y > 0.6f)
                dy = y + y_dot * t * useYdot + heightGain * (y - 0.6f) - (v * arm_sin_f32(pitch) * t - 4.9f * t * t);
            else
                dy = y + y_dot * t * useYdot - (v * arm_sin_f32(pitch) * t - 4.9f * t * t);
        }
        temp_y += dy * gain;

        if (fabsf(dy) < 0.005f)
            break;
    }
    pitch = atan2f(temp_y, temp_x);
    *forwardTime = t;
    return pitch * RADIAN_COEF;
}

static float Ballistic_Model(float x, float v, float pitch)
{
    static float vx0;
    static float t;
    // static float ft, dft_dt, temp_t;
    vx0 = v * arm_cos_f32(pitch);

    t = (expf(bulletModelK * x) - 1) / (bulletModelK * vx0);
    if (t > 1)
        t = 1;
    if (!isnormal(t))
        t = 0;

    // 牛顿迭代，巨慢无比
    //    for (int i = 0; i < 20; i++)
    //    {
    //        ft = logf(k * vx0 * t + 1) / k - x_dot * t - x;
    //        dft_dt = vx0 / (k * vx0 * t + 1) - t;
    //        temp_t = t - ft / dft_dt;

    //        if (temp_t > 1.5f)
    //            temp_t = 1.5f;
    //        if (temp_t < 0)
    //            temp_t = 0;

    //        t = temp_t;
    //    }

    return t;
}

/******************************* miniPC Handle ********************************/
static void GetTargetPositionFull(ControlFrameFull CtrlFrameTempFull, uint8_t *buff)
{
    AimAssist.miniPC_Online = 1;
    AimAssist.Frame_dt = DWT_GetDeltaT(&AimAssist.UpdatePeriodCount);
    AimAssist.TimeConsuming = CtrlFrameTempFull.time_stamp_ms / 10.0f / 1000.0f;

    // ��������ϵƫ��
    AimAssist.FrameTimeStamp = INS_GetTimeline();

    if (CtrlFrameTempFull.flg != 0)
    {
        static uint16_t qFrameNum;
        // static float q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        static float rawPos[3], rawVec[3];
        static float tempPitch, tempYaw, tempRoll, q[4], cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
        static float y, theta, phi, tantheta, tanphi, costheta, cosphi;

        memcpy(&AimAssist.CtrlFrameFull, buff, AimAssist_RX_BUF_NUM);
        TgtPosPredict.LostCount = 0;

        AimAssist.dataValid = 1;
        AimAssist.TargetID = AimAssist.CtrlFrameFull.flg;

        ShootEvaluation.TargetValidTime = INS_GetTimeline();

        // �������ϵ��������Э�������
        y = AimAssist.CtrlFrameFull.py / 1000.0f;
        theta = atan2f(AimAssist.CtrlFrameFull.px, AimAssist.CtrlFrameFull.py);
        phi = atan2f(AimAssist.CtrlFrameFull.pz, AimAssist.CtrlFrameFull.py);
        tantheta = (float)AimAssist.CtrlFrameFull.px / AimAssist.CtrlFrameFull.py;
        tanphi = (float)AimAssist.CtrlFrameFull.pz / AimAssist.CtrlFrameFull.py;
        costheta = arm_cos_f32(theta);
        cosphi = arm_cos_f32(phi);

        if (UseCovariance)
        {
            TgtPosPredict.Rc_data[0] = sigmaSqY * tantheta * tantheta + sigmaSqTheta * y * y / powf(costheta, 4);
            TgtPosPredict.Rc_data[1] = sigmaSqY * tantheta;
            TgtPosPredict.Rc_data[2] = sigmaSqY * tantheta * tanphi;
            TgtPosPredict.Rc_data[3] = TgtPosPredict.Rc_data[1];
            TgtPosPredict.Rc_data[4] = sigmaSqY;
            TgtPosPredict.Rc_data[5] = sigmaSqY * tanphi;
            TgtPosPredict.Rc_data[6] = TgtPosPredict.Rc_data[2];
            TgtPosPredict.Rc_data[7] = TgtPosPredict.Rc_data[5];
            TgtPosPredict.Rc_data[8] = sigmaSqY * tanphi * tanphi + sigmaSqPhi * y * y / powf(cosphi, 4);
        }
        else
        {
            TgtPosPredict.Rc_data[0] = 5;
            TgtPosPredict.Rc_data[1] = 0;
            TgtPosPredict.Rc_data[2] = 0;
            TgtPosPredict.Rc_data[3] = 0;
            TgtPosPredict.Rc_data[4] = 25;
            TgtPosPredict.Rc_data[5] = 0;
            TgtPosPredict.Rc_data[6] = 0;
            TgtPosPredict.Rc_data[7] = 0;
            TgtPosPredict.Rc_data[8] = 5;
        }

        // ���ϵ�任����̨ϵ
        rawPos[X] = AimAssist.CtrlFrameFull.px / 1000.0f;
        rawPos[Y] = AimAssist.CtrlFrameFull.py / 1000.0f;
        rawPos[Z] = AimAssist.CtrlFrameFull.pz / 1000.0f;
        // float norm = invSqrt(AimAssist.CtrlFrameFull.rx * AimAssist.CtrlFrameFull.rx +
        //                      AimAssist.CtrlFrameFull.ry * AimAssist.CtrlFrameFull.ry +
        //                      AimAssist.CtrlFrameFull.rz * AimAssist.CtrlFrameFull.rz + 1e-6f);
        float norm = 1.0f;
        rawVec[X] = AimAssist.CtrlFrameFull.rx * norm;
        rawVec[Y] = AimAssist.CtrlFrameFull.ry * norm;
        rawVec[Z] = AimAssist.CtrlFrameFull.rz * norm;
        ChassisPosPredict.CVec_data[X] = rawVec[X];
        ChassisPosPredict.CVec_data[Y] = rawVec[Y];
        ChassisPosPredict.CVec_data[Z] = rawVec[Z];
        CameraOffsetCorrection(rawPos, AimAssist.TargetBodyFrame, rawVec, AimAssist.TargetBodyVec);

        arm_sqrt_f32(AimAssist.TargetBodyFrame[X] * AimAssist.TargetBodyFrame[X] +
                         AimAssist.TargetBodyFrame[Y] * AimAssist.TargetBodyFrame[Y] +
                         AimAssist.TargetBodyFrame[Z] * AimAssist.TargetBodyFrame[Z],
                     &AimAssist.Distance);

        TgtPosPredict.ArmorType = AimAssist.CtrlFrameFull.type;
        if (TgtPosPredict.ArmorType == BIG_ARMOR && (AimAssist.TargetID == 3 || AimAssist.TargetID == 4 || AimAssist.TargetID == 5))
            AimAssist.armor_num = BALANCE_2;
        else if (AimAssist.TargetID == 6)
            AimAssist.armor_num = OUTPOST_3;
        else
            AimAssist.armor_num = NORMAL_4;
        // AimAssist.armor_num = OUTPOST_3;
        AimAssist.conjecture_time = (AimAssist.armor_num == BALANCE_2) ? 500 : 300;

        for (uint8_t i = 0; i < 3; i++)
            TgtPosPredict.TargetBodyFrame[i] = AimAssist.TargetBodyFrame[i];

        // ��������״̬
        AimAssist.Status = TargetValid;

        AimAssist.TimeConsuming = float_constrain(AimAssist.TimeConsuming, 0.0f, 0.015f);
        AimAssist.DelayTime = AimAssist.FrameDelayToINS + AimAssist.TimeConsuming;
        qFrameNum = Find_qFrame(&INS.qFrame, INS_GetTimeline() / 1000.0f - AimAssist.DelayTime);
        memcpy(q, INS.qFrame[qFrameNum].q, sizeof(q));

        tempYaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
        tempPitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
        tempRoll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));

        cosYaw = arm_cos_f32(tempYaw);
        cosPitch = arm_cos_f32(tempPitch);
        cosRoll = arm_cos_f32(tempRoll);
        sinYaw = arm_sin_f32(tempYaw);
        sinPitch = arm_sin_f32(tempPitch);
        sinRoll = arm_sin_f32(tempRoll);

        // ������ת����     1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        TgtPosPredict.Cbn_data[0] = cosYaw * cosRoll - sinYaw * sinPitch * sinRoll;
        TgtPosPredict.Cbn_data[1] = -cosPitch * sinYaw;
        TgtPosPredict.Cbn_data[2] = cosYaw * sinRoll + cosRoll * sinYaw * sinPitch;
        TgtPosPredict.Cbn_data[3] = cosYaw * sinPitch * sinRoll + cosRoll * sinYaw;
        TgtPosPredict.Cbn_data[4] = cosYaw * cosPitch;
        TgtPosPredict.Cbn_data[5] = sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        TgtPosPredict.Cbn_data[6] = -cosPitch * sinRoll;
        TgtPosPredict.Cbn_data[7] = sinPitch;
        TgtPosPredict.Cbn_data[8] = cosPitch * cosRoll;
        AimAssist.TargetEarthFrame[X] = TgtPosPredict.Cbn_data[0] * AimAssist.TargetBodyFrame[X] +
                                        TgtPosPredict.Cbn_data[1] * AimAssist.TargetBodyFrame[Y] +
                                        TgtPosPredict.Cbn_data[2] * AimAssist.TargetBodyFrame[Z];
        AimAssist.TargetEarthFrame[Y] = TgtPosPredict.Cbn_data[3] * AimAssist.TargetBodyFrame[X] +
                                        TgtPosPredict.Cbn_data[4] * AimAssist.TargetBodyFrame[Y] +
                                        TgtPosPredict.Cbn_data[5] * AimAssist.TargetBodyFrame[Z];
        AimAssist.TargetEarthFrame[Z] = TgtPosPredict.Cbn_data[6] * AimAssist.TargetBodyFrame[X] +
                                        TgtPosPredict.Cbn_data[7] * AimAssist.TargetBodyFrame[Y] +
                                        TgtPosPredict.Cbn_data[8] * AimAssist.TargetBodyFrame[Z];

        memcpy(ChassisPosPredict.Cbn_data1, TgtPosPredict.Cbn_data, sizeof(TgtPosPredict.Cbn_data));
        Matrix_Inverse(&ChassisPosPredict.Cbn1, &ChassisPosPredict.Cbninv);
        Matrix_Transpose(&ChassisPosPredict.Cbninv, &ChassisPosPredict.CbninvT);
        Matrix_Multiply(&ChassisPosPredict.CbninvT, &ChassisPosPredict.CVec, &ChassisPosPredict.EVec);

        AimAssist.TargetEarthVec[X] = TgtPosPredict.Cbn_data[0] * AimAssist.TargetBodyVec[X] +
                                      TgtPosPredict.Cbn_data[1] * AimAssist.TargetBodyVec[Y] +
                                      TgtPosPredict.Cbn_data[2] * AimAssist.TargetBodyVec[Z];
        AimAssist.TargetEarthVec[Y] = TgtPosPredict.Cbn_data[3] * AimAssist.TargetBodyVec[X] +
                                      TgtPosPredict.Cbn_data[4] * AimAssist.TargetBodyVec[Y] +
                                      TgtPosPredict.Cbn_data[5] * AimAssist.TargetBodyVec[Z];
        AimAssist.TargetEarthVec[Z] = TgtPosPredict.Cbn_data[6] * AimAssist.TargetBodyVec[X] +
                                      TgtPosPredict.Cbn_data[7] * AimAssist.TargetBodyVec[Y] +
                                      TgtPosPredict.Cbn_data[8] * AimAssist.TargetBodyVec[Z];
        AimAssist.TargetTheta = atan2f(AimAssist.TargetEarthVec[Y], AimAssist.TargetEarthVec[X]);
        // AimAssist.TargetTheta = atan2f(AimAssist.CtrlFrameFull.ry, AimAssist.CtrlFrameFull.rx);
        debugValue[39] = atan2f(ChassisPosPredict.EVec_data[Y], ChassisPosPredict.EVec_data[X]);

        ChassisPosPredict.Estimator.MeasuredVector[0] = AimAssist.TargetEarthFrame[X];
        ChassisPosPredict.Estimator.MeasuredVector[1] = AimAssist.TargetEarthFrame[Y];
        ChassisPosPredict.Estimator.MeasuredVector[2] = AimAssist.TargetTheta;

        for (uint8_t i = 0; i < 3; i++)
        {
            if (!isnormal(ChassisPosPredict.Estimator.MeasuredVector[i]))
                ChassisPosPredict.Estimator.MeasuredVector[i] = 0;
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            TgtPosPredict.TargetEarthFrame[i] = AimAssist.TargetEarthFrame[i];
            TgtPosPredict.TgtMotionEst.MeasuredVector[i] = AimAssist.TargetEarthFrame[i];
        }

        memcpy(ChassisPosPredict.Rc_data, TgtPosPredict.Rc_data, sizeof(TgtPosPredict.Rc_data));
        memcpy(ChassisPosPredict.Ccb_data, TgtPosPredict.Ccb_data, sizeof(TgtPosPredict.Ccb_data));
        memcpy(ChassisPosPredict.Cbn_data, TgtPosPredict.Cbn_data, sizeof(TgtPosPredict.Cbn_data));

        AimAssist.YawPosition = -atan2f(AimAssist.TargetEarthFrame[X], AimAssist.TargetEarthFrame[Y]) * RADIAN_COEF;
        AimAssist.PitchPosition = atan2f(AimAssist.TargetEarthFrame[Z], AimAssist.TargetEarthFrame[Y]) * RADIAN_COEF;
    }
    else
    {
        // �Ӿ�δʶ��Ŀ��
        AimAssist.CtrlFrameFull.flg = 0;

        if (TgtPosPredict.LostCount == 0)
            TgtPosPredict.TargetLostTime = INS_GetTimeline();

        if (INS_GetTimeline() - TgtPosPredict.TargetLostTime < AimAssist.conjecture_time)
        {
            TgtPosPredict.Status = TgtConjecture;
        }
        else
        {
            // Ŀ�궪ʧ ��λ�˶���Ϣ
            AimAssist.Status = TargetLost;
            TgtPosPredict.Status = TgtLost;
            TgtPosPredict.TrackingCount = 0;

            // �������˲�����λ
            ChassisEst_Reset(NULL, 0, 0);
            TgtMotionEst_Reset(NULL);
        }
        TgtPosPredict.LostCount++;
    }
}

#if ENABLE_CALIBRATION
static void CalibrationCallback(CalibrationFrame_t CaliFrameTemp)
{
    AimAssist.TimeConsuming = CaliFrameTemp.time_stamp_ms / 10.0f / 1000.0f;
    if (CaliFrameTemp.flg != 0)
    {
        static uint16_t qFrameNum;
        static float tempPitch, tempYaw, tempRoll;
        static float q[4];

        if (INS.Gyro[X] * INS.Gyro[X] * INS.Gyro[Y] * INS.Gyro[Y] * INS.Gyro[Z] * INS.Gyro[Z] < 0.01f)
            AimAssist.dataValid = 1;

        // 获取大地坐标系下目标位置
        AimAssist.DelayTime = AimAssist.FrameDelayToINS + AimAssist.TimeConsuming;
        qFrameNum = FindTimeMatchFrame(&QuaternionBuffer, INS_GetTimeline() - (uint32_t)(AimAssist.DelayTime * 1000));

        memcpy(q, QuaternionBuffer.qFrame[qFrameNum].q, sizeof(q));
        tempYaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
        tempPitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
        tempRoll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));
    }
}
#endif

void Callback_AimAssist_Handle(uint8_t *buff)
{
#if ENABLE_CALIBRATION
    memcpy(&CalibrationFrame, buff, CALIBRATION_RX_BUF_NUM);
#else
    if (buff[0] == 0x66)
        memcpy(&CtrlFrameTempFull, buff, AimAssist_RX_BUF_NUM);

        // if (CtrlFrameTempFull.CF_SOF != 0x66)
        // {
        //     USART_IDLE_Init(AimAssist.AA_USART, AimAssist_Rx_Buf, AimAssist_RX_BUF_NUM);
        //     return;
        // }
#endif

#if ENABLE_CALIBRATION
    CalibrationCallback(CalibrationFrame);
#else
    GetTargetPositionFull(CtrlFrameTempFull, buff);
    memcpy(AimAssist_Rx_Buf_Buf, buff, sizeof(AimAssist_Rx_Buf_Buf));
#endif
    AimAssist.FdbFlag = 1;
}

static void CameraOffsetCorrection(float *inputPos, float *outputPos, float *inputVec, float *outputVec)
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    static uint8_t flag = 1;

    // 只有当程序第一次运行或偏移量发生变化时才重新计算旋转矩阵
    if (fabsf(OffsetCorrection.CameraYaw - lastYawOffset) > 0.001f ||
        fabsf(OffsetCorrection.CameraPitch - lastPitchOffset) > 0.001f ||
        fabsf(OffsetCorrection.CameraRoll - lastRollOffset) > 0.001f || flag)
    {
        cosYaw = arm_cos_f32(OffsetCorrection.CameraYaw / 57.295779513f);
        cosPitch = arm_cos_f32(OffsetCorrection.CameraPitch / 57.295779513f);
        cosRoll = arm_cos_f32(OffsetCorrection.CameraRoll / 57.295779513f);
        sinYaw = arm_sin_f32(OffsetCorrection.CameraYaw / 57.295779513f);
        sinPitch = arm_sin_f32(OffsetCorrection.CameraPitch / 57.295779513f);
        sinRoll = arm_sin_f32(OffsetCorrection.CameraRoll / 57.295779513f);

        // 更新旋转矩阵     1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll - sinYaw * sinPitch * sinRoll;
        c_12 = -cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll + cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll + cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        TgtPosPredict.Ccb_data[0] = c_11;
        TgtPosPredict.Ccb_data[1] = c_12;
        TgtPosPredict.Ccb_data[2] = c_13;
        TgtPosPredict.Ccb_data[3] = c_21;
        TgtPosPredict.Ccb_data[4] = c_22;
        TgtPosPredict.Ccb_data[5] = c_23;
        TgtPosPredict.Ccb_data[6] = c_31;
        TgtPosPredict.Ccb_data[7] = c_32;
        TgtPosPredict.Ccb_data[8] = c_33;
        flag = 0;
    }
    if (inputPos == NULL || outputPos == NULL)
        return;

    // 应注意，对于云台 yaw pitch 转轴不相交的情况
    // 此处需融合 pitch 轴电机编码器反馈进行变换，应注意时间同步
    outputPos[X] = c_11 * inputPos[X] +
                   c_12 * inputPos[Y] +
                   c_13 * inputPos[Z] + OffsetCorrection.CamX;
    outputPos[Y] = c_21 * inputPos[X] +
                   c_22 * inputPos[Y] +
                   c_23 * inputPos[Z] + OffsetCorrection.CamY;
    outputPos[Z] = c_31 * inputPos[X] +
                   c_32 * inputPos[Y] +
                   c_33 * inputPos[Z] + OffsetCorrection.CamZ;
    outputPos[Y] -= OffsetCorrection.axis_offset * arm_cos_f32(Gimbal.PitchMotor.AngleInDegree * PI / 180.0f);
    outputPos[Z] += OffsetCorrection.axis_offset * arm_sin_f32(Gimbal.PitchMotor.AngleInDegree * PI / 180.0f);

    // memcpy(ChassisPosPredict.Ccb_data1, TgtPosPredict.Ccb_data, sizeof(TgtPosPredict.Ccb_data));
    // Matrix_Inverse(&ChassisPosPredict.Ccb1, &ChassisPosPredict.Ccbinv);
    // Matrix_Transpose(&ChassisPosPredict.Ccbinv, &ChassisPosPredict.CcbinvT);
    // Matrix_Multiply(&ChassisPosPredict.CVec, &ChassisPosPredict.CcbinvT, &ChassisPosPredict.BVec);

    outputVec[X] = c_11 * inputVec[X] +
                   c_12 * inputVec[Y] +
                   c_13 * inputVec[Z];
    outputVec[Y] = c_21 * inputVec[X] +
                   c_22 * inputVec[Y] +
                   c_23 * inputVec[Z];
    outputVec[Z] = c_31 * inputVec[X] +
                   c_32 * inputVec[Y] +
                   c_33 * inputVec[Z];

    lastYawOffset = OffsetCorrection.CameraYaw;
    lastPitchOffset = OffsetCorrection.CameraPitch;
    lastRollOffset = OffsetCorrection.CameraRoll;
}

void USER_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
    if (huart == remote_control.RC_USART)
        Callback_RC_Handle(&remote_control, sbus_rx_buf);
    if (huart == AimAssist.AA_USART)
#if ENABLE_CALIBRATION
        Callback_AimAssist_Handle(Calibration_Rx_Buf);
#else
        Callback_AimAssist_Handle(AimAssist_Rx_Buf);
#endif
}

/*************************** TgtMotionEst KF Func *****************************/
static void TgtMotionEst_Init(void)
{
    // 卡尔曼滤波器初始化
    TgtPosPredict.TgtMotionEst.UseAutoAdjustment = TRUE;
    Kalman_Filter_Init(&TgtPosPredict.TgtMotionEst, 6, 0, 3);
    TgtPosPredict.TgtMotionEst.MeasurementMap[0] = 1;
    TgtPosPredict.TgtMotionEst.MeasurementMap[1] = 3;
    TgtPosPredict.TgtMotionEst.MeasurementMap[2] = 5;
    TgtPosPredict.TgtMotionEst.MeasurementDegree[0] = 1;
    TgtPosPredict.TgtMotionEst.MeasurementDegree[1] = 1;
    TgtPosPredict.TgtMotionEst.MeasurementDegree[2] = 1;
    TgtPosPredict.TgtMotionEst.MatR_DiagonalElements[0] = 0.0001;
    TgtPosPredict.TgtMotionEst.MatR_DiagonalElements[1] = 0.0001;
    TgtPosPredict.TgtMotionEst.MatR_DiagonalElements[2] = 0.0001;
    memcpy(TgtPosPredict.TgtMotionEst.F_data, TgtMotionEst_F, sizeof(TgtMotionEst_F));
    memcpy(TgtPosPredict.TgtMotionEst.P_data, TgtMotionEst_Pinit, sizeof(TgtMotionEst_Pinit));
    memcpy(TgtPosPredict.TgtMotionEst.Q_data, TgtMotionEst_Q, sizeof(TgtMotionEst_Q));
    TgtPosPredict.TgtMotionEst.SkipEq3 = TRUE;
    TgtPosPredict.TgtMotionEst.SkipEq4 = TRUE;
    TgtPosPredict.TgtMotionEst.User_Func0_f = TgtMotionEst_Tuning;
    TgtPosPredict.TgtMotionEst.User_Func2_f = TgtMotionEst_Set_R;
    TgtPosPredict.TgtMotionEst.User_Func3_f = TgtMotionEst_ChiSquare_Test;

    Matrix_Init(&TgtPosPredict.Ccb, 3, 3, (float *)TgtPosPredict.Ccb_data);
    Matrix_Init(&TgtPosPredict.CcbT, 3, 3, (float *)TgtPosPredict.CcbT_data);
    Matrix_Init(&TgtPosPredict.Cbn, 3, 3, (float *)TgtPosPredict.Cbn_data);
    Matrix_Init(&TgtPosPredict.CbnT, 3, 3, (float *)TgtPosPredict.CbnT_data);
    Matrix_Init(&TgtPosPredict.Rc, 3, 3, (float *)TgtPosPredict.Rc_data);
    Matrix_Init(&TgtPosPredict.tempMat, 3, 3, (float *)TgtPosPredict.tempMat_data);
    Matrix_Init(&TgtPosPredict.H, 2, 6, (float *)TgtPosPredict.H_data);
    Matrix_Init(&TgtPosPredict.HT, 6, 2, (float *)TgtPosPredict.HT_data);
    Matrix_Init(&TgtPosPredict.M1, 6, 2, (float *)TgtPosPredict.M1_data);
    Matrix_Init(&TgtPosPredict.M2, 2, 2, (float *)TgtPosPredict.M2_data);
    Matrix_Init(&TgtPosPredict.ResErr, 2, 1, (float *)TgtPosPredict.ResErr_data);
    Matrix_Init(&TgtPosPredict.ResErrT, 1, 2, (float *)TgtPosPredict.ResErrT_data);
}

static void TgtMotionEst_Update(float dt)
{
    // 卡尔曼滤波器更新
    static float sigmaSqrt[3];
    /*
     0  1  2  3  4  5
     6  7  8  9 10 11
    12 13 14 15 16 17
    18 19 20 21 22 23
    24 25 26 27 28 29
    30 31 32 33 34 35
    */
    for (uint8_t i = 0; i < 3; i++)
        sigmaSqrt[i] = TgtMotionEst_Sigma[i] * TgtMotionEst_Sigma[i];

    // 根据更新频率设置 F Q 矩阵
    TgtPosPredict.TgtMotionEst.F_data[1] = dt;
    TgtPosPredict.TgtMotionEst.F_data[15] = dt;
    TgtPosPredict.TgtMotionEst.F_data[29] = dt;

    TgtMotionEst_Q[0] = dt * dt * dt / 3 * sigmaSqrt[X];
    TgtMotionEst_Q[1] = dt * dt / 2 * sigmaSqrt[X];
    TgtMotionEst_Q[6] = dt * dt / 2 * sigmaSqrt[X];
    TgtMotionEst_Q[7] = dt * sigmaSqrt[X];

    TgtMotionEst_Q[14] = dt * dt * dt / 3 * sigmaSqrt[Y];
    TgtMotionEst_Q[15] = dt * dt / 2 * sigmaSqrt[Y];
    TgtMotionEst_Q[20] = dt * dt / 2 * sigmaSqrt[Y];
    TgtMotionEst_Q[21] = dt * sigmaSqrt[Y];

    TgtMotionEst_Q[28] = dt * dt * dt / 3 * sigmaSqrt[Z];
    TgtMotionEst_Q[29] = dt * dt / 2 * sigmaSqrt[Z];
    TgtMotionEst_Q[34] = dt * dt / 2 * sigmaSqrt[Z];
    TgtMotionEst_Q[35] = dt * sigmaSqrt[Z];

    Kalman_Filter_Update(&TgtPosPredict.TgtMotionEst);

    // 检查滤波器是否发散，若为 nan 则需手动置零
    for (uint8_t i = 0; i < TgtPosPredict.TgtMotionEst.xhatSize; i++)
    {
        if (!isnormal(TgtPosPredict.TgtMotionEst.xhat_data[i]))
        {
            TgtPosPredict.TgtMotionEst.xhat_data[i] = 0;
            nanCount++;
        }
        if (!isnormal(TgtPosPredict.TgtMotionEst.FilteredValue[i]))
            TgtPosPredict.TgtMotionEst.FilteredValue[i] = 0;
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        // 对速度观测值差分得到加速度
        TgtPosPredict.Accel[i] = (TgtPosPredict.TgtMotionEst.FilteredValue[i * 2 + 1] - TgtPosPredict.Velocity[i]) / (TgtPosPredict.AccLPF + dt) +
                                 TgtPosPredict.Accel[i] * TgtPosPredict.AccLPF / (TgtPosPredict.AccLPF + dt);
        TgtPosPredict.Position[i] = TgtPosPredict.TgtMotionEst.FilteredValue[i * 2];
        TgtPosPredict.Velocity[i] = TgtPosPredict.TgtMotionEst.FilteredValue[i * 2 + 1];
    }

    // 计算目标水平速度大小
    arm_sqrt_f32(TgtPosPredict.Velocity[X] * TgtPosPredict.Velocity[X] +
                     TgtPosPredict.Velocity[Y] * TgtPosPredict.Velocity[Y],
                 &TgtPosPredict.xOyVelocity);

    // 计算目标距离
    arm_sqrt_f32(TgtPosPredict.Position[X] * TgtPosPredict.Position[X] +
                     TgtPosPredict.Position[Y] * TgtPosPredict.Position[Y] +
                     TgtPosPredict.Position[Z] * TgtPosPredict.Position[Z],
                 &TgtPosPredict.Distance);
    float HorizontalDistance;
    // 计算目标水平距离
    arm_sqrt_f32(TgtPosPredict.Position[X] * TgtPosPredict.Position[X] +
                     TgtPosPredict.Position[Y] * TgtPosPredict.Position[Y],
                 &HorizontalDistance);
    // 避免出现除零错误
    if (HorizontalDistance < 1e-5f)
        HorizontalDistance = 1e-5f;
    TgtPosPredict.HorizontalDistance = HorizontalDistance * dt / (TgtPosPredict.HorizontalDistanceLPF + dt) + TgtPosPredict.HorizontalDistance * TgtPosPredict.HorizontalDistanceLPF / (TgtPosPredict.HorizontalDistanceLPF + dt);

    // 计算水平方向速度
    float HorizontalDistance_dot = TgtPosPredict.Position[X] / TgtPosPredict.HorizontalDistance * TgtPosPredict.Velocity[X] +
                                   TgtPosPredict.Position[Y] / TgtPosPredict.HorizontalDistance * TgtPosPredict.Velocity[Y];
    TgtPosPredict.HorizontalDistance_dot = HorizontalDistance_dot * dt / (TgtPosPredict.HorizontalDistance_dotLPF + dt) + TgtPosPredict.HorizontalDistance_dot * TgtPosPredict.HorizontalDistance_dotLPF / (TgtPosPredict.HorizontalDistance_dotLPF + dt);

    // 获取目标高度及其变化率
    TgtPosPredict.TgtHeight = TgtPosPredict.Position[Z];
    TgtPosPredict.TgtHeight_dot = TgtPosPredict.Velocity[Z];
}

static void TgtMotionEst_Set_R(KalmanFilter_t *kf)
{
    // 计算观测器 R 矩阵值，详见自瞄文档
    if (kf->Pminus_data[0] > TgtMotionEst_Pinit[0])
        kf->Pminus_data[0] = TgtMotionEst_Pinit[0];
    if (kf->Pminus_data[14] > TgtMotionEst_Pinit[14])
        kf->Pminus_data[14] = TgtMotionEst_Pinit[14];
    if (kf->Pminus_data[28] > TgtMotionEst_Pinit[28])
        kf->Pminus_data[28] = TgtMotionEst_Pinit[28];
    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        Matrix_Transpose(&TgtPosPredict.Ccb, &TgtPosPredict.CcbT);
        Matrix_Multiply(&TgtPosPredict.Rc, &TgtPosPredict.CcbT, &TgtPosPredict.tempMat); // tempMat = Rc·CcbT
        Matrix_Multiply(&TgtPosPredict.Ccb, &TgtPosPredict.tempMat, &kf->R);             // R = Ccb·Rc·CcbT
        Matrix_Transpose(&TgtPosPredict.Cbn, &TgtPosPredict.CbnT);
        Matrix_Multiply(&kf->R, &TgtPosPredict.CbnT, &TgtPosPredict.tempMat); // tempMat = Ccb·Rc·CcbT·CbnT
        Matrix_Multiply(&TgtPosPredict.Cbn, &TgtPosPredict.tempMat, &kf->R);  // R = Cbn·Ccb·Rc·CcbT·CbnT
        memcpy(TgtMotionEst_R, kf->R_data, sizeof(TgtMotionEst_R));
    }
}

static void TgtMotionEst_ChiSquare_Test(KalmanFilter_t *kf)
{
    static uint8_t debugchart = 0, enChiSquareTest = 1;
    static float TgtSwitchThreshold = 20, dist, xhatMinusDist, xSq, ySq, zSq;
    static uint32_t CatchSwitchCount = 0;
    float TgtMotionEst_z_H[6];
    // ���� z(k) - H xhat'(k)
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H��P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H��P'(k)��HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H��P'(k)��HT + R)

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H xhat'(k)
    memcpy(TgtMotionEst_z_H, &kf->temp_vector1, sizeof(TgtMotionEst_z_H));
    // chi-square test
    // kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    // kf->temp_matrix.numCols = 1;
    // kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H��P'(k)��HT + R)��(z(k) - H xhat'(k))
    // kf->temp_vector.numRows = 1;
    // kf->temp_vector.numCols = kf->temp_vector1.numRows;
    // kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = (z(k) - H xhat'(k))'
    // kf->temp_matrix.numRows = 1;
    // kf->temp_matrix.numCols = 1;
    // kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_vector1, &kf->temp_matrix);

    // arm_sqrt_f32(kf->z_data[X] * kf->z_data[X] +
    //                  kf->z_data[Y] * kf->z_data[Y],
    //              &dist);
    // if (dist < 100)
    //     dist = 100;

    // ���㿨������в�z(k) - h(xhat'(k))�뿨������H����
    arm_sqrt_f32(kf->z_data[X] * kf->z_data[X] +
                     kf->z_data[Y] * kf->z_data[Y],
                 &dist);
    arm_sqrt_f32(kf->xhatminus_data[0] * kf->xhatminus_data[0] +
                     kf->xhatminus_data[2] * kf->xhatminus_data[2],
                 &xhatMinusDist);
    xSq = kf->xhatminus_data[0] * kf->xhatminus_data[0];
    ySq = kf->xhatminus_data[2] * kf->xhatminus_data[2];
    zSq = kf->xhatminus_data[4] * kf->xhatminus_data[4];
    TgtPosPredict.H_data[0] = kf->xhatminus_data[2] / (xSq + ySq);
    TgtPosPredict.H_data[1] = 0;
    TgtPosPredict.H_data[2] = -kf->xhatminus_data[0] / (xSq + ySq);
    TgtPosPredict.H_data[3] = 0;
    TgtPosPredict.H_data[4] = 0;
    TgtPosPredict.H_data[5] = 0;
    TgtPosPredict.H_data[6] = -kf->xhatminus_data[0] * kf->xhatminus_data[4] / (xSq + ySq + zSq) / xhatMinusDist;
    TgtPosPredict.H_data[7] = 0;
    TgtPosPredict.H_data[8] = -kf->xhatminus_data[2] * kf->xhatminus_data[4] / (xSq + ySq + zSq) / xhatMinusDist;
    TgtPosPredict.H_data[9] = 0;
    TgtPosPredict.H_data[10] = xhatMinusDist / (xSq + ySq + zSq);
    TgtPosPredict.H_data[11] = 0;

    kf->MatStatus = Matrix_Transpose(&TgtPosPredict.H, &TgtPosPredict.HT);
    TgtPosPredict.M1.numRows = 6;
    TgtPosPredict.M1.numCols = 2;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &TgtPosPredict.HT, &TgtPosPredict.M1); // M1 = P'(k)��HT
    TgtPosPredict.M2.numRows = 2;
    TgtPosPredict.M2.numCols = 2;
    kf->MatStatus = Matrix_Multiply(&TgtPosPredict.H, &TgtPosPredict.M1, &TgtPosPredict.M2); // M2 = H��P'(k)��HT
    TgtPosPredict.M2_data[0] += sigmaSqTheta;
    TgtPosPredict.M2_data[3] += sigmaSqPhi;
    TgtPosPredict.M1.numRows = 2;
    TgtPosPredict.M1.numCols = 2;
    kf->MatStatus = Matrix_Inverse(&TgtPosPredict.M2, &TgtPosPredict.M1); // M1 = inv(H��P'(k)��HT)
    TgtPosPredict.ResErr_data[0] = atan2f(kf->z_data[X], kf->z_data[Y]) - atan2f(kf->xhatminus_data[0], kf->xhatminus_data[2]);
    TgtPosPredict.ResErr_data[1] = atan2f(kf->z_data[Z], dist) - atan2f(kf->xhatminus_data[4], xhatMinusDist);
    kf->MatStatus = Matrix_Transpose(&TgtPosPredict.ResErr, &TgtPosPredict.ResErrT);
    TgtPosPredict.M2.numRows = 2;
    TgtPosPredict.M2.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&TgtPosPredict.M1, &TgtPosPredict.ResErr, &TgtPosPredict.M2); // M2 = inv(H��P'��HT)��(z - h(xhat'))
    kf->temp_matrix.numRows = 1;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&TgtPosPredict.ResErrT, &TgtPosPredict.M2, &kf->temp_matrix); // temp_matrix = (z - h(xhat'))T��inv(H��P'��HT)��(z - h(xhat')) �õ���⺯��r

    if (debugchart)
        Serial_Debug(&huart1, 1, kf->temp_matrix.pData[0] * dist * 0.00001f, AimAssist.TargetEarthFrame[X] / 100,
                     AimAssist.TargetEarthFrame[Y] / 100, AimAssist.TargetEarthFrame[Z] / 100, TgtSwitchThreshold, dist / 100);
    // ��������
    arm_sqrt_f32((kf->z_data[X] - kf->xhatminus_data[0]) * (kf->z_data[X] - kf->xhatminus_data[0]) +
                     (kf->z_data[Y] - kf->xhatminus_data[2]) * (kf->z_data[Y] - kf->xhatminus_data[2]),
                 &xyPositonError);
    ChiSquare = kf->temp_matrix.pData[0] * dist;
    if ((ChiSquare > TgtSwitchThreshold ||
         (xyPositonError > 0.25 && INS_GetTimeline() - TgtPosPredict.TrackingTimeStamp > 125)) &&
        enChiSquareTest)
    {
        if (CatchSwitchCount <= 4)
        {
            // ��Ϣδͨ���������� ��Ԥ��
            // xhat(k) = xhat'(k)
            // P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE;
            if (CatchSwitchCount >= 2)
                TgtPosPredict.Status = TgtSwitch;
            CatchSwitchCount++;
            return;
        }
        else
        {
            kf->SkipEq5 = FALSE;
        }
    }
    else
    {
        TgtPosPredict.TrackingTimeStamp = INS_GetTimeline();
        TgtPosPredict.Status = TgtTracking;
        CatchSwitchCount = 0;
        kf->SkipEq5 = FALSE;
    }

    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)��HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)��(z(k) - H��xhat'(k))
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}
static void TgtMotionEst_Tuning(KalmanFilter_t *kf)
{
    memcpy(TgtMotionEst_F, kf->F_data, sizeof(TgtMotionEst_F));
    memcpy(TgtMotionEst_P, kf->P_data, sizeof(TgtMotionEst_P));
    memcpy(kf->Q_data, TgtMotionEst_Q, sizeof(TgtMotionEst_Q));
}
static void TgtMotionEst_Reset(float *new_position)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        if (i < 3)
        {
            TgtPosPredict.Velocity[i] = 0;
            TgtPosPredict.Accel[i] = 0;
        }
        if (i == 0 || i == 2 || i == 4)
            continue;
        TgtPosPredict.TgtMotionEst.xhat_data[i] = 0;
        TgtPosPredict.TgtMotionEst.FilteredValue[i] = 0;
    }
    TgtPosPredict.HorizontalDistance_dot = 0;
    memcpy(TgtPosPredict.TgtMotionEst.P_data, TgtMotionEst_Pinit, sizeof(TgtMotionEst_Pinit));
    if (new_position != NULL)
    {
        TgtPosPredict.TgtMotionEst.xhat_data[0] = new_position[X];
        TgtPosPredict.TgtMotionEst.xhat_data[2] = new_position[Y];
        TgtPosPredict.TgtMotionEst.xhat_data[4] = new_position[Z];
        TgtPosPredict.TgtMotionEst.FilteredValue[0] = new_position[X];
        TgtPosPredict.TgtMotionEst.FilteredValue[2] = new_position[Y];
        TgtPosPredict.TgtMotionEst.FilteredValue[4] = new_position[Z];
        arm_sqrt_f32(new_position[X] * new_position[X] +
                         new_position[Y] * new_position[Y],
                     &TgtPosPredict.HorizontalDistance);
        TgtPosPredict.TgtMotionEst.P_data[0] = TgtPosPredict.TgtMotionEst.R_data[0];
        TgtPosPredict.TgtMotionEst.P_data[2] = TgtPosPredict.TgtMotionEst.R_data[1];
        TgtPosPredict.TgtMotionEst.P_data[4] = TgtPosPredict.TgtMotionEst.R_data[2];
        TgtPosPredict.TgtMotionEst.P_data[12] = TgtPosPredict.TgtMotionEst.R_data[3];
        TgtPosPredict.TgtMotionEst.P_data[14] = TgtPosPredict.TgtMotionEst.R_data[4];
        TgtPosPredict.TgtMotionEst.P_data[16] = TgtPosPredict.TgtMotionEst.R_data[5];
        TgtPosPredict.TgtMotionEst.P_data[24] = TgtPosPredict.TgtMotionEst.R_data[6];
        TgtPosPredict.TgtMotionEst.P_data[26] = TgtPosPredict.TgtMotionEst.R_data[7];
        TgtPosPredict.TgtMotionEst.P_data[28] = TgtPosPredict.TgtMotionEst.R_data[8];
    }
}

/*************************** ChassisPosPredict KF Func *****************************/
static void ChassisEst_Init(void)
{
    // 卡尔曼滤波器初始化
    ChassisPosPredict.Estimator.UseAutoAdjustment = TRUE;
    Kalman_Filter_Init(&ChassisPosPredict.Estimator, 8, 0, 3);
    memcpy(ChassisPosPredict.Estimator.F_data, ChassisEst_F, sizeof(ChassisEst_F));
    memcpy(ChassisPosPredict.Estimator.P_data, ChassisEst_Pinit, sizeof(ChassisEst_Pinit));
    memcpy(ChassisPosPredict.Estimator.Q_data, ChassisEst_Q, sizeof(ChassisEst_Q));
    ChassisPosPredict.Estimator.MatR_DiagonalElements[0] = 0.01;
    ChassisPosPredict.Estimator.MatR_DiagonalElements[1] = 0.01;
    ChassisPosPredict.Estimator.MatR_DiagonalElements[2] = 0.01;
    ChassisPosPredict.Estimator.SkipEq3 = TRUE;
    ChassisPosPredict.Estimator.SkipEq4 = TRUE;
    ChassisPosPredict.Estimator.User_Func0_f = ChassisEst_Tuning;
    ChassisPosPredict.Estimator.User_Func2_f = ChassisEst_Set_R_H;
    ChassisPosPredict.Estimator.User_Func3_f = ChassisEst_ChiSquare_Test;
    ChassisPosPredict.Estimator.User_Func4_f = ChassisEst_xhat_Limit;

    Matrix_Init(&ChassisPosPredict.Ccb, 3, 3, (float *)ChassisPosPredict.Ccb_data);
    Matrix_Init(&ChassisPosPredict.CcbT, 3, 3, (float *)ChassisPosPredict.CcbT_data);
    Matrix_Init(&ChassisPosPredict.Cbn, 3, 3, (float *)ChassisPosPredict.Cbn_data);
    Matrix_Init(&ChassisPosPredict.CbnT, 3, 3, (float *)ChassisPosPredict.CbnT_data);

    Matrix_Init(&ChassisPosPredict.Ccb1, 3, 3, (float *)ChassisPosPredict.Ccb_data1);
    Matrix_Init(&ChassisPosPredict.Ccbinv, 3, 3, (float *)ChassisPosPredict.Ccbinv_data);
    Matrix_Init(&ChassisPosPredict.CcbinvT, 3, 3, (float *)ChassisPosPredict.CcbinvT_data);

    Matrix_Init(&ChassisPosPredict.Cbn1, 3, 3, (float *)ChassisPosPredict.Cbn_data1);
    Matrix_Init(&ChassisPosPredict.Cbninv, 3, 3, (float *)ChassisPosPredict.Cbninv_data);
    Matrix_Init(&ChassisPosPredict.CbninvT, 3, 3, (float *)ChassisPosPredict.CbninvT_data);

    Matrix_Init(&ChassisPosPredict.BVec, 3, 1, (float *)ChassisPosPredict.BVec_data);
    Matrix_Init(&ChassisPosPredict.CVec, 3, 1, (float *)ChassisPosPredict.CVec_data);
    Matrix_Init(&ChassisPosPredict.EVec, 3, 1, (float *)ChassisPosPredict.EVec_data);

    Matrix_Init(&ChassisPosPredict.Rc, 3, 3, (float *)ChassisPosPredict.Rc_data);
    Matrix_Init(&ChassisPosPredict.tempMat, 3, 3, (float *)ChassisPosPredict.tempMat_data);
    Matrix_Init(&ChassisPosPredict.tempMat1, 3, 3, (float *)ChassisPosPredict.tempMat1_data);
    Matrix_Init(&ChassisPosPredict.M1, 3, 1, (float *)ChassisPosPredict.M1_data);
    Matrix_Init(&ChassisPosPredict.M2, 1, 1, (float *)ChassisPosPredict.M2_data);
    Matrix_Init(&ChassisPosPredict.ResErr, 3, 1, (float *)ChassisPosPredict.ResErr_data);
    Matrix_Init(&ChassisPosPredict.ResErrT, 1, 3, (float *)ChassisPosPredict.ResErrT_data);
}

static void ChassisEst_Update(float dt)
{
    // 卡尔曼滤波器更新
    /*
     0     1     2     3     4     5     6     7
     8     9    10    11    12    13    14    15
    16    17    18    19    20    21    22    23
    24    25    26    27    28    29    30    31
    32    33    34    35    36    37    38    39
    40    41    42    43    44    45    46    47
    48    49    50    51    52    53    54    55
    56    57    58    59    60    61    62    63
    */

    // 根据更新频率设置 F Q 矩阵
    ChassisPosPredict.Estimator.F_data[1] = dt;
    ChassisPosPredict.Estimator.F_data[19] = dt;
    ChassisPosPredict.Estimator.F_data[37] = dt;

    if (AimAssist.armor_num == OUTPOST_3)
        ChassisEst_Sigma[2] = 0.05f;
    else
        ChassisEst_Sigma[2] = 50.0f;

    ChassisPosPredict.Estimator.Q_data[0] = dt * dt * dt / 3 * ChassisEst_Sigma[X];
    ChassisPosPredict.Estimator.Q_data[1] = dt * dt / 2 * ChassisEst_Sigma[X];
    ChassisPosPredict.Estimator.Q_data[8] = dt * dt / 2 * ChassisEst_Sigma[X];
    ChassisPosPredict.Estimator.Q_data[9] = dt * ChassisEst_Sigma[X];

    ChassisPosPredict.Estimator.Q_data[18] = dt * dt * dt / 3 * ChassisEst_Sigma[Y];
    ChassisPosPredict.Estimator.Q_data[19] = dt * dt / 2 * ChassisEst_Sigma[Y];
    ChassisPosPredict.Estimator.Q_data[26] = dt * dt / 2 * ChassisEst_Sigma[Y];
    ChassisPosPredict.Estimator.Q_data[27] = dt * ChassisEst_Sigma[Y];

    ChassisPosPredict.Estimator.Q_data[36] = dt * dt * dt / 3 * ChassisEst_Sigma[2];
    ChassisPosPredict.Estimator.Q_data[37] = dt * dt / 2 * ChassisEst_Sigma[2];
    ChassisPosPredict.Estimator.Q_data[44] = dt * dt / 2 * ChassisEst_Sigma[2];
    ChassisPosPredict.Estimator.Q_data[45] = dt * ChassisEst_Sigma[2];

    ChassisPosPredict.Estimator.Q_data[54] = dt * ChassisEst_Sigma[3];
    ChassisPosPredict.Estimator.Q_data[63] = dt * ChassisEst_Sigma[3];

    Kalman_Filter_Update(&ChassisPosPredict.Estimator);

    ChassisPosPredict.Center[X] = ChassisPosPredict.Estimator.FilteredValue[0];
    ChassisPosPredict.CenterVel[X] = ChassisPosPredict.Estimator.FilteredValue[1];
    ChassisPosPredict.Center[Y] = ChassisPosPredict.Estimator.FilteredValue[2];
    ChassisPosPredict.CenterVel[Y] = ChassisPosPredict.Estimator.FilteredValue[3];
    ChassisPosPredict.theta = ChassisPosPredict.Estimator.FilteredValue[4];
    ChassisPosPredict.theta_dot = ChassisPosPredict.Estimator.FilteredValue[5];
    if (switch_count % 2 == 0)
        ChassisPosPredict.r = ChassisPosPredict.Estimator.FilteredValue[6];
    else
        ChassisPosPredict.r = ChassisPosPredict.Estimator.FilteredValue[7];

    if (ChassisPosPredict.r < 0.0f)
        ChassisEst_Reset(AimAssist.TargetEarthFrame, AimAssist.TargetTheta, AimAssist.CtrlFrameFull.flg);
    else if (ChassisPosPredict.r > 0.6f)
        ChassisEst_Reset(AimAssist.TargetEarthFrame, AimAssist.TargetTheta, AimAssist.CtrlFrameFull.flg);

    ChassisPosPredict.r = float_constrain(ChassisPosPredict.r, 0.15f, 0.4f);

    float current_yaw;
    current_yaw = Angle_Process(ChassisPosPredict.theta, AimAssist.TargetTheta, AimAssist.armor_num);
    TgtPosPredict.Position[X] = ChassisPosPredict.Center[X] - ChassisPosPredict.r * arm_cos_f32(current_yaw);
    TgtPosPredict.Position[Y] = ChassisPosPredict.Center[Y] - ChassisPosPredict.r * arm_sin_f32(current_yaw);
    TgtPosPredict.Position[Z] = TgtPosPredict.Position[Z] * TgtPosPredict.HeightLPF / (TgtPosPredict.HeightLPF + dt) + AimAssist.TargetEarthFrame[Z] * dt / (TgtPosPredict.HeightLPF + dt);

    if (last_switch_count != switch_count)
        TgtPosPredict.DeltaHeight = TgtPosPredict.TgtHeight - AimAssist.TargetEarthFrame[Z];

    TgtPosPredict.TgtHeight = TgtPosPredict.Position[Z];

    uint8_t is_current_armor;
    for (uint8_t i = 0; i < AimAssist.armor_num; i++)
    {
        ChassisPosPredict.yaw[i] = STD_RADIAN(current_yaw + i * 2 * PI / AimAssist.armor_num);
        if (AimAssist.armor_num == NORMAL_4)
        {
            ChassisPosPredict.armor_height[i] = TgtPosPredict.Position[Z] + (is_current_armor ? 0 : TgtPosPredict.DeltaHeight);
            is_current_armor = 1 - is_current_armor;
        }
        else
            ChassisPosPredict.armor_height[i] = TgtPosPredict.Position[Z];
    }

    // 检查滤波器是否发散，若为 nan 则需手动置零
    for (uint8_t i = 0; i < 8; i++)
    {
        if (!isnormal(ChassisPosPredict.Estimator.xhat_data[i]))
        {
            ChassisPosPredict.Estimator.xhat_data[i] = 0;
            nanCount++;
        }
        if (!isnormal(ChassisPosPredict.Estimator.FilteredValue[i]))
            ChassisPosPredict.Estimator.FilteredValue[i] = 0;
    }

    float HorizontalDistance;
    arm_sqrt_f32(TgtPosPredict.Position[X] * TgtPosPredict.Position[X] +
                     TgtPosPredict.Position[Y] * TgtPosPredict.Position[Y],
                 &HorizontalDistance);
    // 避免出现除零错误
    if (HorizontalDistance < 1e-5f)
        HorizontalDistance = 1e-5f;
    TgtPosPredict.HorizontalDistance = HorizontalDistance * dt / (TgtPosPredict.HorizontalDistanceLPF + dt) + TgtPosPredict.HorizontalDistance * TgtPosPredict.HorizontalDistanceLPF / (TgtPosPredict.HorizontalDistanceLPF + dt);

    last_switch_count = switch_count;
}

// after Pmius
static void ChassisEst_Set_R_H(KalmanFilter_t *kf)
{
    static float chisquare;
    static float chisquareThreshold = 2.0f;
    static float last_yaw_data;
    static uint32_t SwitchTimeStamp;
    /*
    0     1     2     3     4     5     6     7
    8     9     10    11    12    13    14    15
    16    17    18    19    20    21    22    23
    24    25    26    27    28    29    30    31
    32    33    34    35    36    37    38    39
    40    41    42    43    44    45    46    47
    48    49    50    51    52    53    54    55
    56    57    58    59    60    61    62    63
    */
    // 计算观测器 R 矩阵值，详见自瞄文档
    for (int i = 0; i < 8; i++)
    {
        if (kf->Pminus_data[i * 9] > ChassisEst_Pinit[i * 9])
            kf->Pminus_data[i * 9] = ChassisEst_Pinit[i * 9];
    }

    if (kf->xhatminus_data[4] > PI)
        kf->xhatminus_data[4] -= 2 * PI;
    if (kf->xhatminus_data[4] < -PI)
        kf->xhatminus_data[4] += 2 * PI;

    memcpy(ChassisEst_xhatMinus, kf->xhatminus_data, sizeof(ChassisEst_xhatMinus));
    /*
           0     1     2
           3     4     5
           6     7     8
      */
    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        Matrix_Transpose(&ChassisPosPredict.Ccb, &ChassisPosPredict.CcbT);
        Matrix_Multiply(&ChassisPosPredict.Rc, &ChassisPosPredict.CcbT, &ChassisPosPredict.tempMat);      // tempMat = Rc·CcbT
        Matrix_Multiply(&ChassisPosPredict.Ccb, &ChassisPosPredict.tempMat, &ChassisPosPredict.tempMat1); // tempMat = Ccb·Rc·CcbT
        Matrix_Transpose(&ChassisPosPredict.Cbn, &ChassisPosPredict.CbnT);
        Matrix_Multiply(&ChassisPosPredict.tempMat1, &ChassisPosPredict.CbnT, &ChassisPosPredict.tempMat); // tempMat = Ccb·Rc·CcbT·CbnT
        Matrix_Multiply(&ChassisPosPredict.Cbn, &ChassisPosPredict.tempMat, &ChassisPosPredict.tempMat1);  // tempMat1 = Cbn·Ccb·Rc·CcbT·CbnT

        kf->R_data[0] = ChassisPosPredict.tempMat1_data[0];
        kf->R_data[1] = ChassisPosPredict.tempMat1_data[1];
        kf->R_data[2] = ChassisPosPredict.tempMat1_data[3];
        kf->R_data[3] = ChassisPosPredict.tempMat1_data[4];
        kf->R_data[8] = sigmaSqYaw;
        memcpy(ChassisEst_R, kf->R_data, sizeof(ChassisEst_R));
        /*
         0     1     2     3     4     5     6     7
         8     9    10    11    12    13    14    15
        16    17    18    19    20    21    22    23
        */
        memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);
        chisquare = STD_RADIAN(kf->z_data[2] - last_yaw_data) * STD_RADIAN(kf->z_data[2] - last_yaw_data);

        if (AimAssist.armor_num == NORMAL_4)
        {
            if (chisquare > 0.9f && chisquare < 3.0f && INS_GetTimeline() - SwitchTimeStamp > 80)
            {
                switch_count = switch_count + 1;
                SwitchTimeStamp = INS_GetTimeline();
            }
        }
        else
            switch_count = 0;

        float yaw = Angle_Process(kf->xhatminus_data[4], kf->z_data[2], AimAssist.armor_num);
        kf->H_data[0] = 1;
        kf->H_data[10] = 1;
        kf->H_data[20] = 1;
        if (switch_count % 2 == 0)
        {
            float r = kf->xhatminus_data[6];

            kf->H_data[4] = r * arm_sin_f32(yaw);
            kf->H_data[12] = -r * arm_cos_f32(yaw);
            kf->H_data[6] = -arm_cos_f32(yaw);
            kf->H_data[14] = -arm_sin_f32(yaw);
        }
        else
        {
            float r = kf->xhatminus_data[7];

            kf->H_data[4] = r * arm_sin_f32(yaw);
            kf->H_data[12] = -r * arm_cos_f32(yaw);
            kf->H_data[7] = -arm_cos_f32(yaw);
            kf->H_data[15] = -arm_sin_f32(yaw);
        }
        memcpy(ChassisEst_H, kf->H_data, sizeof(ChassisEst_H));

        last_yaw_data = kf->z_data[2];
    }
}

// after K
static void ChassisEst_ChiSquare_Test(KalmanFilter_t *kf)
{
    static uint8_t debugchart = 0, enChiSquareTest = 1;
    static float dist, xhatMinusDist, xSq, ySq, zSq;
    static uint32_t CatchSwitchCount = 0;
    static float c_x, c_y, yaw, yaw_, r;

    // 计算 z(k) - H xhat'(k)
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    // kf->z_data[2] = Angle_Process(kf->z_data[2], kf->xhat_data[4], AimAssist.armor_num);
    // yaw_ = kf->xhatminus_data[4];
    c_x = kf->xhatminus_data[0];
    c_y = kf->xhatminus_data[2];
    yaw = Angle_Process(kf->xhatminus_data[4], kf->z_data[2], AimAssist.armor_num);

    if (switch_count % 2 == 0)
        r = kf->xhatminus_data[6];
    else
        r = kf->xhatminus_data[7];

    // temp_vector = h(xhat'(k))
    kf->temp_vector_data[0] = c_x - r * arm_cos_f32(yaw);
    kf->temp_vector_data[1] = c_y - r * arm_sin_f32(yaw);
    kf->temp_vector_data[2] = yaw;

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))
    if (kf->temp_vector1.pData[2] > PI)
        kf->temp_vector1.pData[2] -= 2 * PI;
    if (kf->temp_vector1.pData[2] < -PI)
        kf->temp_vector1.pData[2] += 2 * PI;

    // chi - square test
    // kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    // kf->temp_matrix.numCols = 1;
    // kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - H xhat'(k))
    // kf->temp_vector.numRows = 1;
    // kf->temp_vector.numCols = kf->temp_vector1.numRows;
    // kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = (z(k) - H xhat'(k))T
    // kf->temp_matrix.numRows = 1;
    // kf->temp_matrix.numCols = 1;
    // kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_vector1, &kf->temp_matrix);
    // ChassisChiSquare = kf->temp_matrix.pData[0];

    // 卡方检验
    if (ChassisChiSquare > ChassisChiSquareThreshold / (1 + 0.1f * CatchSwitchCount) &&
        enChiSquareTest)
        TgtPosPredict.Status = TgtSwitch;
    else
    {
        // 通过卡方检验，设置目标状态为跟踪状态
        TgtPosPredict.TrackingTimeStamp = INS_GetTimeline();
        TgtPosPredict.Status = TgtTracking;
        CatchSwitchCount = 0;
    }
    kf->SkipEq5 = FALSE;

    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))s

    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

static void ChassisEst_Tuning(KalmanFilter_t *kf)
{
    memcpy(ChassisEst_F, kf->F_data, sizeof(ChassisEst_F));
    memcpy(ChassisEst_P, kf->P_data, sizeof(ChassisEst_P));
    memcpy(ChassisEst_Q, kf->Q_data, sizeof(ChassisEst_Q));
    memcpy(ChassisEst_Z, kf->z_data, sizeof(ChassisEst_Z));
    memcpy(ChassisEst_R, kf->R_data, sizeof(ChassisEst_R));
    memcpy(ChassisEst_xhatMinus, kf->xhatminus_data, sizeof(ChassisEst_xhatMinus));
}

static void ChassisEst_Reset(float *new_position, float new_yaw, uint8_t tgtID)
{
    static float lastTgtID;

    switch_count = 0;
    if (new_position != NULL)
    {
        /*
             0     1     2     3     4     5     6     7     8
             9    10    11    12    13    14    15    16    17
            18    19    20    21    22    23    24    25    26
            27    28    29    30    31    32    33    34    35
            36    37    38    39    40    41    42    43    44
            45    46    47    48    49    50    51    52    53
            54    55    56    57    58    59    60    61    62
            63    64    65    66    67    68    69    70    71
            72    73    74    75    76    77    78    79    80
        */
        /*
             0     1     2     3
             4     5     6     7
             8     9    10    11
            12    13    14    15
        */
        // if (tgtID != lastTgtID)
        // {
        memcpy(ChassisPosPredict.Estimator.P_data, ChassisEst_Pinit, sizeof(ChassisEst_Pinit));
        memcpy(ChassisPosPredict.Estimator.R_data, ChassisEst_R, sizeof(ChassisEst_R));
        memcpy(ChassisPosPredict.Estimator.Pminus_data, ChassisEst_Pinit, sizeof(ChassisEst_Pinit));

        for (uint8_t i = 0; i < 8; i++)
        {
            if (i == 0 || i == 2 || i == 4 || i == 6 || i == 7)
                continue;
            ChassisPosPredict.Estimator.xhat_data[i] = 0;
            ChassisPosPredict.Estimator.FilteredValue[i] = 0;
        }

        ChassisPosPredict.Estimator.xhat_data[0] = new_position[X] + DEFAULT_RADIUS * arm_cos_f32(new_yaw);
        ChassisPosPredict.Estimator.xhat_data[2] = new_position[Y] + DEFAULT_RADIUS * arm_sin_f32(new_yaw);
        ChassisPosPredict.Estimator.xhat_data[4] = new_yaw;
        ChassisPosPredict.Estimator.xhat_data[6] = DEFAULT_RADIUS;
        ChassisPosPredict.Estimator.xhat_data[7] = DEFAULT_RADIUS;

        ChassisPosPredict.Estimator.FilteredValue[0] = new_position[X] + DEFAULT_RADIUS * arm_cos_f32(new_yaw);
        ChassisPosPredict.Estimator.FilteredValue[2] = new_position[Y] + DEFAULT_RADIUS * arm_sin_f32(new_yaw);
        ChassisPosPredict.Estimator.FilteredValue[4] = new_yaw;
        ChassisPosPredict.Estimator.FilteredValue[6] = DEFAULT_RADIUS;
        ChassisPosPredict.Estimator.FilteredValue[7] = DEFAULT_RADIUS;
        // }
    }
    else
    {
        memcpy(ChassisPosPredict.Estimator.P_data, ChassisEst_Pinit, sizeof(ChassisEst_Pinit));
        memcpy(ChassisPosPredict.Estimator.R_data, ChassisEst_R, sizeof(ChassisEst_R));
        memcpy(ChassisPosPredict.Estimator.Pminus_data, ChassisEst_Pinit, sizeof(ChassisEst_Pinit));

        for (uint8_t i = 0; i < 8; i++)
        {
            if (i == 0 || i == 2 || i == 4)
                continue;
            ChassisPosPredict.Estimator.xhat_data[i] = 0;
            ChassisPosPredict.Estimator.FilteredValue[i] = 0;
        }

        ChassisPosPredict.Estimator.xhat_data[6] = DEFAULT_RADIUS;
        ChassisPosPredict.Estimator.xhat_data[7] = DEFAULT_RADIUS;

        ChassisPosPredict.Estimator.FilteredValue[6] = DEFAULT_RADIUS;
        ChassisPosPredict.Estimator.FilteredValue[7] = DEFAULT_RADIUS;
    }
    lastTgtID = tgtID;
}

// after xhat
static void ChassisEst_xhat_Limit(KalmanFilter_t *kf)
{
    kf->xhat_data[4] = STD_RADIAN(kf->xhat_data[4]);
    memcpy(ChassisEst_K, kf->K_data, sizeof(ChassisEst_K));
    memcpy(ChassisEst_xhat, kf->xhat_data, sizeof(ChassisEst_xhat));
}

float Gauss_Rand(void)
{
    static float U, V;
    static int phase = 0;
    float z;

    if (phase == 0)
    {
        U = rand() / (RAND_MAX + 1.0);
        V = rand() / (RAND_MAX + 1.0);
        z = sqrtf(-2.0 * logf(U)) * sin(2.0 * 3.1415926535 * V);
    }
    else
    {
        z = sqrtf(-2.0 * logf(U)) * cos(2.0 * 3.1415926535 * V);
    }

    phase = 1 - phase;
    return z;
}

static float Angle_Process(float angle, float theta, uint8_t armor_num)
{
    for (uint8_t i = 0; i < armor_num; i++)
    {
        if (fabsf(STD_RADIAN(STD_RADIAN(angle + i * PI * 2 / armor_num) - theta)) < PI / armor_num)
        {
            angle = STD_RADIAN(angle + i * PI * 2 / armor_num);
            break;
        }
    }
    return angle;
}

static int8_t Angle_Check(float angle, float theta, uint8_t armor_num)
{
    int8_t armor_ID;
    for (uint8_t i = 0; i < armor_num; i++)
    {
        if (fabsf(STD_RADIAN(angle + i * PI * 2 / armor_num - theta)) < PI / armor_num)
        {
            armor_ID = i;
            break;
        }
    }
    return armor_ID;
}

static void Estimator_Debug(float dt)
{
    static uint32_t count = 0;
    static float q[4] = {0}, yaw;
    static float start_time = 1;
    if (count == 0)
    {
        DebugTgt.Center[X] = 0;
        DebugTgt.Center[Y] = 5.0f;
        DebugTgt.CenterVel[X] = 0;
        DebugTgt.CenterVel[Y] = 0;
        DebugTgt.height[0] = 0.1f;
        DebugTgt.height[1] = 0.15f;
        // DebugTgt.height_dot = 0;
        DebugTgt.theta = PI / 4 * 1.01f;
        DebugTgt.theta_dot = 2;
        DebugTgt.r[0] = 0.25;
        DebugTgt.r[1] = 0.2;
    }
    else
    {
        // DebugTgt.theta_dot = 0;
        DebugTgt.CenterVel[X] = 2.0f * arm_cos_f32(t - start_time) * 0;
        DebugTgt.CenterVel[Y] = 2.0f * arm_cos_f32(t - start_time) * 0;
        DebugTgt.height_dot = 0.1f * arm_sin_f32(t - start_time) * 0;
    }
    DebugTgt.Center[X] += DebugTgt.CenterVel[X] * dt;
    DebugTgt.Center[Y] += DebugTgt.CenterVel[Y] * dt;
    DebugTgt.height[0] += DebugTgt.height_dot * dt;
    DebugTgt.height[1] += DebugTgt.height_dot * dt;
    DebugTgt.theta += DebugTgt.theta_dot * dt;
    if (DebugTgt.theta > PI)
        DebugTgt.theta -= 2 * PI;
    if (DebugTgt.theta < -PI)
        DebugTgt.theta += 2 * PI;

    DebugTgt.CenterAngel = atan2f(DebugTgt.Center[Y], DebugTgt.Center[X]);
    DebugTgt.CenterDistance = sqrtf(DebugTgt.Center[X] * DebugTgt.Center[X] + DebugTgt.Center[Y] * DebugTgt.Center[Y]);

    if (fabsf(STD_RADIAN(DebugTgt.theta - DebugTgt.CenterAngel)) < PI / 4)
    {
        DebugTgt.TargetYaw = STD_RADIAN(DebugTgt.theta);
        DebugTgt.TargetEarthFrame[X] = DebugTgt.Center[X] - DebugTgt.r[0] * arm_cos_f32(DebugTgt.TargetYaw);
        DebugTgt.TargetEarthFrame[Y] = DebugTgt.Center[Y] - DebugTgt.r[0] * arm_sin_f32(DebugTgt.TargetYaw);
        DebugTgt.TargetEarthFrame[Z] = DebugTgt.height[0];
    }
    else if (fabsf(STD_RADIAN(DebugTgt.theta + PI / 2 - DebugTgt.CenterAngel)) < PI / 4)
    {
        DebugTgt.TargetYaw = STD_RADIAN(DebugTgt.theta + PI / 2);
        DebugTgt.TargetEarthFrame[X] = DebugTgt.Center[X] - DebugTgt.r[1] * arm_cos_f32(DebugTgt.TargetYaw);
        DebugTgt.TargetEarthFrame[Y] = DebugTgt.Center[Y] - DebugTgt.r[1] * arm_sin_f32(DebugTgt.TargetYaw);
        DebugTgt.TargetEarthFrame[Z] = DebugTgt.height[1];
    }
    else if (fabsf(STD_RADIAN(DebugTgt.theta + PI - DebugTgt.CenterAngel)) < PI / 4)
    {
        DebugTgt.TargetYaw = STD_RADIAN(DebugTgt.theta + PI);
        DebugTgt.TargetEarthFrame[X] = DebugTgt.Center[X] - DebugTgt.r[0] * arm_cos_f32(DebugTgt.TargetYaw);
        DebugTgt.TargetEarthFrame[Y] = DebugTgt.Center[Y] - DebugTgt.r[0] * arm_sin_f32(DebugTgt.TargetYaw);
        DebugTgt.TargetEarthFrame[Z] = DebugTgt.height[0];
    }
    else
    {
        DebugTgt.TargetYaw = STD_RADIAN(DebugTgt.theta - PI / 2);
        DebugTgt.TargetEarthFrame[X] = DebugTgt.Center[X] - DebugTgt.r[1] * arm_cos_f32(DebugTgt.TargetYaw);
        DebugTgt.TargetEarthFrame[Y] = DebugTgt.Center[Y] - DebugTgt.r[1] * arm_sin_f32(DebugTgt.TargetYaw);
        DebugTgt.TargetEarthFrame[Z] = DebugTgt.height[1];
    }
    // DebugTgt.TargetYaw = DebugTgt.theta;
    // DebugTgt.TargetEarthFrame[X] = DebugTgt.Center[X] - DebugTgt.r[0] * arm_cos_f32(DebugTgt.theta);
    // DebugTgt.TargetEarthFrame[Y] = DebugTgt.Center[Y] - DebugTgt.r[0] * arm_sin_f32(DebugTgt.theta);
    // DebugTgt.TargetEarthFrame[Z] = DebugTgt.height[0];

    // if (count % 4 == 0)
    // {
    AimAssist.FrameTimeStamp = INS_GetTimeline();
    AimAssist.Status = TargetValid;
    TgtPosPredict.LostCount = 0;
    AimAssist.dataValid = 1;
    AimAssist.TargetID = AimAssist.CtrlFrame.flg;
    TgtPosPredict.ArmorType = AimAssist.CtrlFrame.reserve2;
    AimAssist.miniPC_Online = 1;

    float noise[3] = {0}, yaw_noise, dist_noise, rVec_noise;
    yaw_noise = 0.01 * Gauss_Rand() * 0.1;
    dist_noise = 0.02 * Gauss_Rand() * 0.1;
    rVec_noise = 0.1 * Gauss_Rand() * 0.1;
    noise[X] = yaw_noise * arm_cos_f32(yaw) + dist_noise * arm_sin_f32(yaw);
    noise[Y] = yaw_noise * arm_sin_f32(yaw) + dist_noise * arm_cos_f32(yaw);
    noise[Z] = Gauss_Rand() * 3;
    CameraOffsetCorrection(NULL, NULL, NULL, NULL);
    for (uint8_t i = 0; i < 3; i++)
    {
        // 设置卡尔曼滤波器的观测值
        AimAssist.TargetEarthFrame[i] = DebugTgt.TargetEarthFrame[i] + noise[i];
        TgtPosPredict.TargetEarthFrame[i] = DebugTgt.TargetEarthFrame[i] + noise[i];
        TgtPosPredict.TgtMotionEst.MeasuredVector[i] = DebugTgt.TargetEarthFrame[i] + noise[i];
    }
    ChassisPosPredict.Estimator.MeasuredVector[0] = DebugTgt.TargetEarthFrame[0] + noise[0];
    ChassisPosPredict.Estimator.MeasuredVector[1] = DebugTgt.TargetEarthFrame[1] + noise[1];
    ChassisPosPredict.Estimator.MeasuredVector[2] = DebugTgt.TargetYaw + rVec_noise;

    // 计算相机系坐标噪声协方差矩阵
    float y = DebugTgt.CenterDistance;
    float theta = atan2(ChassisPosPredict.Estimator.MeasuredVector[0], ChassisPosPredict.Estimator.MeasuredVector[1]);
    float phi = 0;
    float tantheta = ChassisPosPredict.Estimator.MeasuredVector[0] / ChassisPosPredict.Estimator.MeasuredVector[1];
    float tanphi = 0;
    float costheta = arm_cos_f32(theta);
    float cosphi = arm_cos_f32(phi);

    // 采用模型推导的协方差矩阵
    TgtPosPredict.Rc_data[0] = sigmaSqY * tantheta * tantheta + sigmaSqTheta * y * y / powf(costheta, 4);
    TgtPosPredict.Rc_data[1] = sigmaSqY * tantheta;
    TgtPosPredict.Rc_data[2] = sigmaSqY * tantheta * tanphi;
    TgtPosPredict.Rc_data[3] = TgtPosPredict.Rc_data[1];
    TgtPosPredict.Rc_data[4] = sigmaSqY;
    TgtPosPredict.Rc_data[5] = sigmaSqY * tanphi;
    TgtPosPredict.Rc_data[6] = TgtPosPredict.Rc_data[2];
    TgtPosPredict.Rc_data[7] = TgtPosPredict.Rc_data[5];
    TgtPosPredict.Rc_data[8] = sigmaSqY * tanphi * tanphi + sigmaSqPhi * y * y / powf(cosphi, 4);

    yaw = PI / 2 - atan2f(DebugTgt.TargetEarthFrame[Y], DebugTgt.TargetEarthFrame[X]);
    q[0] = arm_cos_f32(yaw / 2);
    q[3] = arm_sin_f32(yaw / 2);
    TgtPosPredict.Cbn_data[0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    TgtPosPredict.Cbn_data[1] = 2 * (q[1] * q[2] - q[0] * q[3]);
    TgtPosPredict.Cbn_data[2] = 2 * (q[1] * q[3] + q[0] * q[2]);
    TgtPosPredict.Cbn_data[3] = 2 * (q[1] * q[2] + q[0] * q[3]);
    TgtPosPredict.Cbn_data[4] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
    TgtPosPredict.Cbn_data[5] = 2 * (q[2] * q[3] - q[0] * q[1]);
    TgtPosPredict.Cbn_data[6] = 2 * (q[1] * q[3] - q[0] * q[2]);
    TgtPosPredict.Cbn_data[7] = 2 * (q[2] * q[3] + q[0] * q[1]);
    TgtPosPredict.Cbn_data[8] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    memcpy(ChassisPosPredict.Rc_data, TgtPosPredict.Rc_data, sizeof(TgtPosPredict.Rc_data));
    memcpy(ChassisPosPredict.Ccb_data, TgtPosPredict.Ccb_data, sizeof(TgtPosPredict.Ccb_data));
    memcpy(ChassisPosPredict.Cbn_data, TgtPosPredict.Cbn_data, sizeof(TgtPosPredict.Cbn_data));
    // }
    count++;
}

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
