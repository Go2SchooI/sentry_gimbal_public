#include "gimbal_task.h"
#include "remote_control.h"
#include "includes.h"
#include "ins_task.h"
#include "QuaternionEKF.h"
#include "math.h"

Gimbal_t Gimbal = {0};
GimbalSI_t GimbalSI;
SLAM_t SLAM = {0};

uint32_t Gimbal_DWT_Count = 0;
static float dt = 0, t = 0;

static void GimbalSI_Init(void);
static void Gimbal_Set_Mode(void);
static void Get_AimAssist_Status();
static void Gimbal_Get_CtrlValue(void);
static void Gimbal_CtrlValue_Limit(void);
static void Gimbal_Set_Control(void);
static void Send_Gimbal_Current(void);
static void GimbalSI_Calculate(void);

void Gimbal_Init(void)
{
    Gimbal.PitchMotor.Direction = 1;

    Gimbal.YawMotor.zero_offset = YAW_MOTOR_ZERO_OFFSET;
    Gimbal.PitchMotor.zero_offset = PITCH_MOTOR_ZERO_OFFSET;
    SLAM.YawMotor.zero_offset = SLAMYAW_MOTOR_ZERO_OFFSET;

    // Motor initialize
    Gimbal.YawMotor.CAN_ID = YAW_MOTOR_ID;
    Gimbal.PitchMotor.CAN_ID = PITCH_MOTOR_ID;

    // Max angle initialize
    Gimbal.DepressionEncoderInDegree = GIMBAL_MAX_DEPRESSION;
    Gimbal.ElevationEncoderInDegree = GIMBAL_MAX_ELEVATION;

    // Control value ratio initialize
    Gimbal.rcStickYawRatio = RC_STICK_YAW_RATIO;
    Gimbal.rcStickPitchRatio = RC_STICK_PITCH_RATIO;
    Gimbal.rcMouseYawRatio = RC_MOUSE_YAW_RATIO;
    Gimbal.rcMousePitchRatio = RC_MOUSE_PITCH_RATIO;

    Gimbal.YawCruiseDirection = 1;
    Gimbal.LastYawCruiseDirection = 1;
    Gimbal.PitchCruiseDirection = 1;
    Gimbal.LastPitchCruiseDirection = 1;

    Gimbal.YawIncrement = YAW_INCREMENT;
    Gimbal.YawRefAngleLPF = YAW_REFANGLE_LPF;
    Gimbal.PitchIncrement = PITCH_INCREMENT;
    Gimbal.PitchRefAngleLPF = PITCH_REFANGLE_LPF;

    // Gimbal ref velocity TD init
    TD_Init(&Gimbal.YawRefAngularVelocityTD, 5000000, 0.003);
    TD_Init(&Gimbal.PitchRefAngularVelocityTD, 5000000, 0.003);
    TD_Init(&Gimbal.YawRefAngleTD, 2000000, 0.003);
    TD_Init(&Gimbal.PitchRefAngleTD, 2000000, 0.003);

    // Yaw Motor Init
    PID_Init(&Gimbal.YawMotor.PID_Torque, 2000, 2000, 0,
             0, 300, 0, 0, 0, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = 1;
    c[1] = 0;
    c[2] = 0;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Torque, 2000, c, 0, 4, 4);
    Gimbal.YawMotor.Ke = 0;

    PID_Init(&Gimbal.YawMotor.PID_Velocity, YAW_V_PID_MAXOUT, YAW_V_PID_MAXINTEGRAL, 0,
             YAW_V_PID_KP, YAW_V_PID_KI, YAW_V_PID_KD, 0, 0,
             YAW_V_PID_LPF, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = YAW_V_FCC_C0;
    c[1] = YAW_V_FCC_C1;
    c[2] = YAW_V_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Velocity, YAW_V_FFC_MAXOUT, c, YAW_V_FCC_LPF, 8, 8);

    PID_Init(&Gimbal.YawMotor.PID_Angle, YAW_A_PID_MAXOUT, YAW_A_PID_MAXINTEGRAL, 0,
             YAW_A_PID_KP, YAW_A_PID_KI, YAW_A_PID_KD, 0, 0, YAW_A_PID_LPF, YAW_A_PID_D_LPF, 3,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = YAW_A_FCC_C0;
    c[1] = YAW_A_FCC_C1;
    c[2] = YAW_A_FCC_C2;
    Feedforward_Init(&Gimbal.YawMotor.FFC_Angle, YAW_A_FFC_MAXOUT, c, YAW_A_FCC_LPF, 5, 5);
    Gimbal.YawMotor.Max_Out = YAW_MOTOR_MAXOUT;

    // Pitch Motor Init
    PID_Init(&Gimbal.PitchMotor.PID_Torque, 30000, 30000, 0,
             0, 300, 0, 0, 0, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = 1;
    c[1] = 0;
    c[2] = 0;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Torque, 30000, c, 0, 4, 4);
    Gimbal.PitchMotor.Ke = 0;

    PID_Init(&Gimbal.PitchMotor.PID_Velocity, PITCH_V_PID_MAXOUT, PITCH_V_PID_MAXINTEGRAL, 0,
             PITCH_V_PID_KP, PITCH_V_PID_KI, PITCH_V_PID_KD, 0, 0, PITCH_V_PID_LPF, 0, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
    c[0] = PITCH_V_FCC_C0;
    c[1] = PITCH_V_FCC_C1;
    c[2] = PITCH_V_FCC_C2;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Velocity, PITCH_V_FFC_MAXOUT, c, PITCH_V_FCC_LPF, 8, 8);

    PID_Init(&Gimbal.PitchMotor.PID_Angle, PITCH_A_PID_MAXOUT, PITCH_A_PID_MAXINTEGRAL, 0,
             PITCH_A_PID_KP, PITCH_A_PID_KI, PITCH_A_PID_KD, 0, 0, PITCH_A_PID_LPF, PITCH_A_PID_D_LPF, 3,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = PITCH_A_FCC_C0;
    c[1] = PITCH_A_FCC_C1;
    c[2] = PITCH_A_FCC_C2;
    Feedforward_Init(&Gimbal.PitchMotor.FFC_Angle, PITCH_A_FFC_MAXOUT, c, PITCH_A_FCC_LPF, 5, 5);
    Gimbal.PitchMotor.Max_Out = PITCH_MOTOR_MAXOUT;

    GimbalSI_Init();
}

static void GimbalSI_Init(void)
{
    GimbalSI.Q0 = 0.001;
    GimbalSI.Q1 = 0.001;
    GimbalSI.Q2 = 0.001;
    GimbalSI.R = 10000;
    GimbalSI.lambda = 0.999;

    FirstOrderSI_Init(&GimbalSI.YawSI, 0, 0, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
    FirstOrderSI_Init(&GimbalSI.PitchSI, 0, 0, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
}

void Gimbal_Control(void)
{
    dt = DWT_GetDeltaT(&Gimbal_DWT_Count);
    t += dt;
    Gimbal_Set_Mode();

    if (Gimbal.Mode == AimAssist_Mode)
        Get_AimAssist_Status();
    Gimbal_Get_CtrlValue();
    Gimbal_Set_Control();
    Send_Gimbal_Current();

    GimbalSI_Calculate();
}

static void Gimbal_Set_Mode(void)
{
    // AimAssist_Mode BulletCompensation
    switch (remote_control.switch_right) // mode select
    {
    case Switch_Up:
        Gimbal.Mode = AimAssist_Mode;
        break;

    case Switch_Middle:
        Gimbal.Mode = Normal_Mode;
        break;

    case Switch_Down:
        Gimbal.Mode = Normal_Mode;
        break;
    }

    Gimbal.ModeSwitchCount++;
    if (Gimbal.Mode != Gimbal.ModeLast)
        Gimbal.ModeSwitchCount = 0;
}

static void Get_AimAssist_Status(void)
{
    Gimbal.is_reached = 1;

    if (TgtPosPredict.Status != TgtLost)
        Gimbal.aim_status = find_target;
    else
    {
        if ((t - CatchTime > 1.2f) && (Gimbal.is_reached == 1) && (AimAssist.CtrlFrameFull.flg == 0))
            Gimbal.aim_status = lost_target;
        else if ((Gimbal.is_reached == 0) && (t - CatchTime > 15.0f))
            Gimbal.aim_status = listen_lidar;
        else
            Gimbal.aim_status = wait_command;
    }
}

static void Gimbal_Get_CtrlValue(void)
{
    static float AimAssistYaw, AimAssistPitch;
    static uint16_t LastKeyCode = 0;
    static float Ins_Yaw_Angle_Last = 0;

    switch (Gimbal.Mode)
    {
    case AimAssist_Mode:
        Gimbal.YawAngle = INS.YawTotalAngle;
        Gimbal.PitchAngle = INS.Pitch;
        break;
    case Normal_Mode:
        Gimbal.YawAngle = INS.YawTotalAngle;
        Gimbal.PitchAngle = INS.Pitch;
        Gimbal.aim_status = find_target;
        Gimbal.FollowCoef = 0.0f;
        break;
    }
    Gimbal.EncoderYawAngle = Gimbal.YawMotor.AngleInDegree;
    Gimbal.EncoderPitchAngle = Gimbal.PitchMotor.AngleInDegree;
    Gimbal.YawAngularVelocity = INS.Gyro[Z] * arm_cos_f32(Gimbal.EncoderPitchAngle / RADIAN_COEF) +
                                INS.Gyro[Y] * arm_sin_f32(Gimbal.EncoderPitchAngle / RADIAN_COEF);
    Gimbal.PitchAngularVelocity = INS.Gyro[X];

    // ����������
    Gimbal.YawRefAngularVelocity = -remote_control.ch1 * Gimbal.rcStickYawRatio + Gimbal.FollowCoef * Gimbal.FollowYawVelocity * RADIAN_COEF;
    Gimbal.PitchRefAngularVelocity = remote_control.ch2 * Gimbal.rcStickPitchRatio;

    Gimbal.YawRefAngularVelocity = TD_Calculate(&Gimbal.YawRefAngularVelocityTD, Gimbal.YawRefAngularVelocity);
    Gimbal.PitchRefAngularVelocity = TD_Calculate(&Gimbal.PitchRefAngularVelocityTD, Gimbal.PitchRefAngularVelocity);
    Gimbal.YawRefAngle += Gimbal.YawRefAngularVelocity * dt;
    Gimbal.PitchRefAngle += Gimbal.PitchRefAngularVelocity * dt;

    if (Gimbal.Mode == AimAssist_Mode)
    {
        if (Gimbal.aim_status == wait_command)
        {
            Gimbal.YawRefAngle = Gimbal.YawAngle;
            Gimbal.FollowCoef = 0.0f;
        }

        if (Gimbal.aim_status == listen_lidar) // mode select
        {
            Gimbal.FollowCoef = 2.0f;
            CruiseFlag = 0;
        }

        if (Gimbal.aim_status == find_target)
        {
            Gimbal.FollowCoef = 0.0f;
            CruiseFlag = 0;
            CatchTime = t;

            AimAssistYaw = TgtPosPredict.YawPosition;
            AimAssistPitch = TgtPosPredict.PitchPosition;

            if (TgtPosPredict.TrackingCount > 25)
            {
                if (AimAssistYaw - INS.Yaw < -180.0f)
                    Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw + 360.0f;
                else if (AimAssistYaw - INS.Yaw > 180.0f)
                    Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw - 360.0f;
                else
                    Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw;
                Gimbal.PitchRefAngle = AimAssistPitch;

                Gimbal.YawRefAngle = float_constrain(Gimbal.YawRefAngle, Gimbal.YawAngle - 10.5f, Gimbal.YawAngle + 10.5f);
                Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.PitchAngle - 10.5f, Gimbal.PitchAngle + 10.5f);
            }
        }

        if (Gimbal.aim_status == lost_target)
        {
            Gimbal.FollowCoef = 0.0f;
            CruiseFlag = 1;

            Gimbal.YawRefAngle = Gimbal.FilteredYawRefAngle;
            Gimbal.YawRefAngle += Gimbal.YawIncrement * Gimbal.YawCruiseDirection;

            Gimbal.FilteredYawRefAngle = Gimbal.YawRefAngle;

            if (INS.Yaw - Ins_Yaw_Angle_Last > 180.0f)
                Gimbal.YawCruiseCount--;
            else if (INS.Yaw - Ins_Yaw_Angle_Last < -180.0f) // CCW
                Gimbal.YawCruiseCount++;

            if (Gimbal.FilteredYawRefAngle > 880.0f)
                Gimbal.YawCruiseDirection = -1;
            if (Gimbal.FilteredYawRefAngle < -215.0f)
                Gimbal.YawCruiseDirection = 1;

            Ins_Yaw_Angle_Last = INS.Yaw;

            if (Gimbal.PitchRefAngle > GIMBAL_MAX_CRUISEPITCH)
                Gimbal.PitchCruiseDirection = -1;
            else if (Gimbal.PitchRefAngle < GIMBAL_MIN_CRUISEPITCH)
                Gimbal.PitchCruiseDirection = 1;

            Gimbal.PitchRefAngle += Gimbal.PitchIncrement * Gimbal.PitchCruiseDirection;

            Gimbal.FilteredPitchRefAngle = Gimbal.PitchRefAngle * dt / (Gimbal.PitchRefAngleLPF + dt) +
                                           Gimbal.LastFilteredPitchRefAngle * Gimbal.PitchRefAngleLPF / (Gimbal.PitchRefAngleLPF + dt);
        }
    }

    if (TgtPosPredict.Status != TgtLost && remote_control.switch_right == Switch_Middle)
    {
        CruiseFlag = 0;
        CatchTime = t;

        AimAssistYaw = TgtPosPredict.YawPosition;
        AimAssistPitch = TgtPosPredict.PitchPosition;

        if (TgtPosPredict.TrackingCount > 35)
        {
            if (AimAssistYaw - INS.Yaw < -180.0f)
                Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw + 360.0f;
            else if (AimAssistYaw - INS.Yaw > 180.0f)
                Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw - 360.0f;
            else
                Gimbal.YawRefAngle = AimAssistYaw + INS.YawTotalAngle - INS.Yaw;
            Gimbal.PitchRefAngle = AimAssistPitch;

            Gimbal.YawRefAngle = float_constrain(Gimbal.YawRefAngle, Gimbal.YawAngle - 10.5f, Gimbal.YawAngle + 10.5f);
            Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.PitchAngle - 10.5f, Gimbal.PitchAngle + 10.5f);
        }
    }

    if (CruiseFlag == 0)
    {
        Gimbal.FilteredYawRefAngle = Gimbal.YawRefAngle;
        Gimbal.FilteredPitchRefAngle = Gimbal.PitchRefAngle;
    }

    Gimbal.YawCtrlAngle = TD_Calculate(&Gimbal.YawRefAngleTD, Gimbal.FilteredYawRefAngle);
    Gimbal.PitchCtrlAngle = TD_Calculate(&Gimbal.PitchRefAngleTD, Gimbal.FilteredPitchRefAngle);

    if ((is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE)) || (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE)))
    {
        Gimbal.YawCtrlAngle = Gimbal.YawAngle;
        Gimbal.PitchCtrlAngle = Gimbal.PitchAngle;

        Gimbal.YawRefAngle = Gimbal.YawAngle;
        Gimbal.FilteredYawRefAngle = Gimbal.YawAngle;
        Gimbal.PitchRefAngle = Gimbal.PitchAngle;

        Gimbal.YawRefAngleTD.x = Gimbal.YawAngle;
        Gimbal.YawRefAngleTD.dx = 0;
        Gimbal.PitchRefAngleTD.x = Gimbal.PitchAngle;
        Gimbal.PitchRefAngleTD.dx = 0;

        Gimbal.YawMotor.PID_Velocity.Iout = 0;
        Gimbal.PitchMotor.PID_Velocity.Iout = 0;

        Gimbal.YawMotor.FFC_Angle.Last_Ref = Gimbal.YawCtrlAngle;
        Gimbal.PitchMotor.FFC_Angle.Last_Ref = Gimbal.PitchCtrlAngle;
    }

    Gimbal_CtrlValue_Limit();
    LastKeyCode = remote_control.key_code;
    Gimbal.ModeLast = Gimbal.Mode;
}

static void Gimbal_CtrlValue_Limit(void)
{
    // Yaw control value clamp
    switch (Gimbal.Mode)
    {
    case Normal_Mode:
        break;
    }

    // Pitch control value clamp
    if (fabsf(Gimbal.PitchMotor.AngleInDegree - Gimbal.PitchAngle) > 10.0f)
    {
        Gimbal.DepressionIMU = Gimbal.DepressionEncoderInDegree - (Gimbal.PitchMotor.AngleInDegree - Gimbal.PitchAngle);
        Gimbal.ElevationIMU = Gimbal.ElevationEncoderInDegree - (Gimbal.PitchMotor.AngleInDegree - Gimbal.PitchAngle);
    }
    else
    {
        Gimbal.DepressionIMU = Gimbal.DepressionEncoderInDegree;
        Gimbal.ElevationIMU = Gimbal.ElevationEncoderInDegree;
    }

    // Gimbal.YawCtrlAngle = loop_float_constrain(Gimbal.YawCtrlAngle, -180.0f, 180.0f);YawSwitchVelocity
    Gimbal.PitchCtrlAngle = float_constrain(Gimbal.PitchCtrlAngle, Gimbal.DepressionIMU, Gimbal.ElevationIMU);
    Gimbal.PitchRefAngle = float_constrain(Gimbal.PitchRefAngle, Gimbal.DepressionIMU, Gimbal.ElevationIMU);

    Gimbal.FilteredPitchRefAngle = float_constrain(Gimbal.FilteredPitchRefAngle, Gimbal.DepressionIMU, Gimbal.ElevationIMU);

    Gimbal.LastFilteredYawRefAngle = Gimbal.FilteredYawRefAngle;
    Gimbal.LastFilteredPitchRefAngle = Gimbal.FilteredPitchRefAngle;

    if (GlobalDebugMode != GIMBAL_DEBUG && GimbalTuningEnable == 0)
    {
        switch (Gimbal.Mode)
        {
        case AimAssist_Mode:
            Gimbal.YawMotor.PID_Angle.Derivative_LPF_RC = YAW_A_PID_D_LPF;
            Gimbal.YawMotor.FFC_Velocity.c[1] = 0;
            Gimbal.PitchMotor.FFC_Velocity.c[1] = 0;
            Gimbal.PitchMotor.FFC_Angle.MaxOut = 0;
            break;
        case Normal_Mode:
            Gimbal.YawMotor.PID_Angle.Derivative_LPF_RC = YAW_A_PID_D_LPF;
            Gimbal.PitchMotor.FFC_Velocity.c[1] = PITCH_V_FCC_C1;
            Gimbal.PitchMotor.FFC_Angle.MaxOut = PITCH_A_FFC_MAXOUT;
            break;
        }
        if (fabsf(Gimbal.PitchMotor.AngleInDegree) > 30)
            Gimbal.YawMotor.PID_Angle.Kd = Gimbal.YawMotor.PID_Angle.Kd * 30 / fabsf(Gimbal.PitchMotor.AngleInDegree);
    }

    // Output limit
    if ((Gimbal.PitchCtrlAngle - Gimbal.DepressionIMU < 0.5f || Gimbal.ElevationIMU - Gimbal.PitchCtrlAngle < 0.5f) && (GlobalDebugMode != GIMBAL_DEBUG && GimbalTuningEnable == 0))
    {
        Gimbal.PitchMotor.FFC_Angle.MaxOut = 0;
        Gimbal.PitchMotor.FFC_Velocity.MaxOut = 0;
        Gimbal.PitchMotor.FFC_Torque.MaxOut = 0;
    }
}

static void Gimbal_Set_Control(void)
{
    static float compCoef = 0, gravityTorque;
    /********************** Yaw Calculate **********************/
    // 角度环前馈控制
    PID_Calculate(&Gimbal.YawMotor.PID_Angle, Gimbal.YawAngle, Gimbal.YawCtrlAngle);
    // �ǶȻ�ǰ������
    Gimbal.YawMotor.FFC_Angle.Output = float_constrain(Gimbal.YawMotor.FFC_Angle.c[1] * Gimbal.YawRefAngleTD.dx,
                                                       -Gimbal.YawMotor.FFC_Angle.MaxOut, Gimbal.YawMotor.FFC_Angle.MaxOut);
    float YawVelocityLoopInput = float_constrain(Gimbal.YawMotor.PID_Angle.Output + Gimbal.YawMotor.FFC_Angle.Output,
                                                 -Gimbal.YawMotor.PID_Angle.MaxOut, Gimbal.YawMotor.PID_Angle.MaxOut);
    // 速度环反馈控制
    PID_Calculate(&Gimbal.YawMotor.PID_Velocity, Gimbal.YawAngularVelocity, YawVelocityLoopInput);
    // 速度环前馈控制
    Feedforward_Calculate(&Gimbal.YawMotor.FFC_Velocity, Gimbal.YawRefAngleTD.dx);
    ChassisTorque = (Gimbal.YawAngularVelocity - Gimbal.YawMotor.OutputVel_RadPS * ang_vel_scale) * ChassisCoef;
    ChassisTorqueLPF = ChassisTorque * ChassisTorqueLPFcoef + ChassisTorqueLPF * (1 - ChassisTorqueLPFcoef);
    float YawTorqueLoopInput = float_constrain(Gimbal.YawMotor.PID_Velocity.Output + Gimbal.YawMotor.FFC_Velocity.Output + ChassisTorqueLPF,
                                               -Gimbal.YawMotor.PID_Velocity.MaxOut, Gimbal.YawMotor.PID_Velocity.MaxOut);

    PID_Calculate(&Gimbal.YawMotor.PID_Torque, Gimbal.YawMotor.Real_Current, YawTorqueLoopInput);
    Gimbal.YawMotor.Output = float_constrain(Gimbal.YawMotor.PID_Velocity.Output + Gimbal.YawMotor.FFC_Velocity.Output + ChassisTorqueLPF, -Gimbal.YawMotor.Max_Out, Gimbal.YawMotor.Max_Out);

    /********************* Pitch Calculate *********************/
    gravityTorque = arm_cos_f32((INS.Pitch + 30) / RADIAN_COEF) * compCoef;
    PID_Calculate(&Gimbal.PitchMotor.PID_Angle, Gimbal.PitchAngle, Gimbal.PitchCtrlAngle);
    // 角度环前馈控制
    Gimbal.PitchMotor.FFC_Angle.Output = float_constrain(Gimbal.PitchMotor.FFC_Angle.c[1] * Gimbal.PitchRefAngleTD.dx,
                                                         -Gimbal.PitchMotor.FFC_Angle.MaxOut, Gimbal.PitchMotor.FFC_Angle.MaxOut);
    float PitchVelocityLoopInput = float_constrain(Gimbal.PitchMotor.PID_Angle.Output + Gimbal.PitchMotor.FFC_Angle.Output,
                                                   -Gimbal.PitchMotor.PID_Angle.MaxOut, Gimbal.PitchMotor.PID_Angle.MaxOut);
    // 速度环反馈控制
    PID_Calculate(&Gimbal.PitchMotor.PID_Velocity, Gimbal.PitchAngularVelocity, PitchVelocityLoopInput);
    // 速度环前馈控制
    Feedforward_Calculate(&Gimbal.PitchMotor.FFC_Velocity, Gimbal.PitchRefAngleTD.dx);
    Gimbal.PitchMotor.Output = float_constrain(Gimbal.PitchMotor.PID_Velocity.Output + Gimbal.PitchMotor.FFC_Velocity.Output + gravityTorque,
                                               -Gimbal.PitchMotor.Max_Out, Gimbal.PitchMotor.Max_Out);
}

static void Send_Gimbal_Current(void)
{
    static uint16_t qFrameNum;
    static float q[4];

    qFrameNum = Find_qFrame(INS.qFrame, (uint32_t)(INS_GetTimeline() - 100));
    memcpy(q, INS.qFrame[qFrameNum].q, sizeof(q));
    preYaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);

    int16_t yaw1000;
    yaw1000 = (int16_t)(preYaw * 1000.0f);
    FOLLOW_Data_Buf[5] = yaw1000 & 0xFF;
    FOLLOW_Data_Buf[6] = (yaw1000 >> 8) & 0xFF;

    Send_Follow_Data_1(&hcan1, FOLLOW_Data_Buf);
    Send_Follow_Data_2(&hcan1, FOLLOW_Data_Buf);
    Send_Aerial_Data(&hcan2);

    if ((is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE) && GlobalDebugMode != GIMBAL_DEBUG))
    {
        if (Send_Motor_Current_5_8(&hcan2, 0, 0, 0, 0) == HAL_OK &&
            Send_RMD_Current_Single(&hcan1, 1, 0) == HAL_OK)
            ;
        // HAL_IWDG_Refresh(&hiwdg); // ι�� ������Ҫ
        else
            CAN_SEND_ERROR_COUNT++;
    }
    else
    {
        if (Send_Motor_Current_5_8(&hcan2, 0, Gimbal.PitchMotor.Output, 0, 0) == HAL_OK &&
            Send_RMD_Current_Single(&hcan1, 1, Gimbal.YawMotor.Output) == HAL_OK)
            // if (Send_Motor_Current_5_8(&hcan2, 0, 0, 0, 0) == HAL_OK &&
            //     Send_RMD_Current_Single(&hcan1, 1, 0) == HAL_OK)
            ;
        // HAL_IWDG_Refresh(&hiwdg); // ι�� ������Ҫ
        else
            CAN_SEND_ERROR_COUNT++;
    }

    // if (game_status.game_progress == 4)
    //     GameStatus = 1;
    // else
    //     GameStatus = 0;

    // Send_Game_Status(&hcan2, GameStatus);
}

static void GimbalSI_Calculate(void)
{
    if (GimbalSI.ResetFlag)
    {
        GimbalSI.ResetFlag = 0;

        for (uint8_t i = 0; i < 3; i++)
        {
            GimbalSI.YawSI.SI_EKF.xhat_data[i] = 0;
            GimbalSI.PitchSI.SI_EKF.xhat_data[i] = 0;
        }

        GimbalSI.YawSI.SI_EKF.P_data[0] = 10000;
        GimbalSI.YawSI.SI_EKF.P_data[4] = 10000000;
        GimbalSI.YawSI.SI_EKF.P_data[8] = 10000000;
        GimbalSI.PitchSI.SI_EKF.P_data[0] = 10000;
        GimbalSI.PitchSI.SI_EKF.P_data[4] = 10000000;
        GimbalSI.PitchSI.SI_EKF.P_data[8] = 10000000;
    }
    FirstOrderSI_EKF_Tuning(&GimbalSI.YawSI, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);
    FirstOrderSI_EKF_Tuning(&GimbalSI.PitchSI, GimbalSI.Q0, GimbalSI.Q1, GimbalSI.Q2, GimbalSI.R, GimbalSI.lambda);

    FirstOrderSI_Update(&GimbalSI.YawSI, Gimbal.YawMotor.PID_Torque.Ref, Gimbal.YawAngularVelocity, dt);
    FirstOrderSI_Update(&GimbalSI.PitchSI, Gimbal.PitchMotor.PID_Torque.Ref, Gimbal.PitchAngularVelocity, dt);
    // FirstOrderSI_Update(&GimbalSI.YawSI, Gimbal.YawMotor.Real_Current, Gimbal.YawAngularVelocity, dt);
    // FirstOrderSI_Update(&GimbalSI.PitchSI, Gimbal.PitchMotor.Real_Current, Gimbal.PitchAngularVelocity, dt);
}

void GetAngularVelocity(uint8_t *buff)
{
    static float angularVelocity;
    static uint32_t reachCount;

    Gimbal.is_reached = 1;

    angularVelocity = (int16_t)((buff[7] << 8) | buff[6]);
    Gimbal.FollowYawVelocity = (angularVelocity / 1000.0f) * dt / (dt + 0.005f) + Gimbal.FollowYawVelocity * 0.005f / (dt + 0.005f);
    Gimbal.FollowYawVelocity = float_constrain(Gimbal.FollowYawVelocity, -2.0f, 2.0f);
}
