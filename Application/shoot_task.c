#include "shoot_task.h"
#include "aimassist_task.h"
#include "math.h"
#include "usart.h"

Shoot_t Shoot = {0};

uint32_t Shoot_DWT_Count = 0;
static float dt = 0, t = 0;

uint8_t ShootDebugMode = 1;

float SpeedInBulletsPerSec = 22;
float NumsOfOneShot = 1;
float ShootDelayInMs = 0;
float DebugAmp = 1000, DebugOmega = 10, DebugOffset = 5000;
float Q0 = 35, Q1 = 0.0000000000001, Q2 = 0.000001, R = 1000;
static float BulletSpeed;

static uint16_t LastKeyCode = 0;
static void Shoot_Set_Mode(void);
static void Shoot_Get_CtrlValue(void);
static void Shoot_Set_Control(void);
static void Send_Shoot_Output(void);

static void DoubleBarrel_Coutrol(void);
static void Barrel_Switch(void);
static void Barrel_Mode_Switch(void);
static void Stallad_Self_Recovery(void);
static uint8_t Barrel_SwitchByRC(void);
static void DoubleBarrel_SetOutput(void);
static uint8_t Heat_Judgment(void);

void Shoot_Init(void)
{
    Shoot.ShootMode = NoShooting;

    Shoot.heatRemain = 20;

    Shoot.RefBulletVelocity = 28.5;
    Shoot.BulletVelocity = Shoot.RefBulletVelocity;

    for (uint8_t i = 0; i < 2; i++)
    {
        Shoot.TriggerMotor[i].Direction = NEGATIVE;
        PID_Init(
            &Shoot.TriggerMotor[i].PID_Velocity, 10000, 5000, 0, -15, -100 * 0, 0, 500, 1000, 0.001, 0.001, 5, Integral_Limit | ErrorHandle);
        Shoot.TriggerMotor[i].Max_Out = 10000.0f;
    }

    TD_Init(&Shoot.FricTD, 50000, 0.001);
    Shoot.RefFricSpeed = 0;

    //---------------------------------------------
    PID_Init(
        &Shoot.Rotary.RotaryMotor.PID_Velocity, 10000, 5000, 0, 10, 100, 0, 500, 1000, 0.001, 0.001, 5, Integral_Limit | ErrorHandle);
    Shoot.Rotary.Rotary_Output_Coefficient = 1;
    Shoot.Rotary.RotaryMotor.Max_Out = MAX_ROTARY_OUTPUT;
    Shoot.Rotary.Shooter_id = Shooter1;
    Shoot.Rotary.fixed_output = FIXED_OUTPUT;
    Shoot.Rotary.slew_output = SLEW_OUTPUT;
    Shoot.Rotary.Rotary_Output_MAX = MAX_ROTARY_OUTPUT;
    Shoot.Rotary.Recovery_time = TIMES_COUNT;
    Shoot.Rotary.current_lpf = 0.075;
    //----------------------------------------------

    for (uint8_t i = 0; i < 2; i++)
    {
        Shoot.FricMotor[i].CAN_ID = 0x207 + i;
        PID_Init(
            &Shoot.FricMotor[i].PID_Velocity, 16384, 16384, 0, 15, 80 * 0, 0, 500, 1000, 0.001, 0.001, 5, Integral_Limit | DerivativeFilter | OutputFilter);
        Shoot.InitialKi = Shoot.FricMotor[i].PID_Velocity.Ki;
        float c[3] = {0.005, 0, 0};
        Feedforward_Init(&Shoot.FricMotor[i].FFC_Velocity, 16384, c, 0, 4, 4);
        // FirstOrderSI_Init(&Shoot.FricMotorSI[i], 0.009, 0.23, 0.01, 0.001, 0.001, 10000, 1);

        Shoot.FricMotor[i].Max_Out = 16384.0f;
    }
    Shoot.isLidOpen = 1;

    Shoot.Rotary.RotaryMotor.Output = 1200.0;
}

void Shoot_Control(void)
{
    dt = DWT_GetDeltaT(&Shoot_DWT_Count);
    t += dt;
    Shoot_Set_Mode();
    Shoot_Get_CtrlValue();
    Shoot_Set_Control();
    DoubleBarrel_Coutrol();
    Send_Shoot_Output();

    LastKeyCode = remote_control.key_code;
}

static void Shoot_Set_Mode(void)
{
#ifdef USE_BULLETCOUNT
    static uint32_t BulletCountLast = 0;
#endif
    static uint32_t OneShotCount = 0;
    static uint16_t LastKeyCode = 0;
    static uint16_t LastPressLeft = 0;
    static uint8_t rcLeftSwitchValueLast = 0;
    static uint32_t KeyR_Tick = 0, KeyX_Tick = 0;
    static uint32_t shoot_stamp;

    if ((remote_control.switch_left == Switch_Down && rcLeftSwitchValueLast != Switch_Down))
    {
        if (Shoot.FricSpeed >= FRIC_MOTOR_MIN_RPM)
            Shoot.isFricOn = 0;
        else
            Shoot.isFricOn = 1;
    }

    if (Gimbal.Mode == AimAssist_Mode)
    {
        Shoot.ShootMode = AimAssistShooting;
        Shoot.isFricOn = 1;
    }

    // TriggerMotor mode set
    if (Shoot.FricSpeed >= FRIC_MOTOR_MIN_RPM)
    {
        if (Gimbal.Mode == AimAssist_Mode)
        {
            HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
            // HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
        }

        if (remote_control.mouse.press_left == 1 || remote_control.switch_left == Switch_Up)
        {
            if (GlobalDebugMode == SHOOT_DEBUG)
                Shoot.ShootMode = DebugShoot;
            else
            {
                if (rcLeftSwitchValueLast != Switch_Up)
                    shoot_stamp = INS_GetTimeline();

                if (INS_GetTimeline() - shoot_stamp > 500)
                    Shoot.ShootMode = KeepShooting;
            }
        }
        else
        {
            Shoot.ShootMode = NoShooting;
        }
    }
    else
    {
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
        Shoot.ShootMode = NoShooting;
    }

    if (is_TOE_Error(TRIGGER_MOTOR1_TOE))
    {
        Shoot.ShootMode = NoShooting;
        Shoot.isFricOn = 0;
    }

#ifdef USE_BULLETCOUNT
    if (USER_GetTick() - ModeSwitchTick > 500 && Shoot.ShootMode == DebugShoot)
        Shoot.ShootMode = NoShooting;

    if (BulletCount - BulletCountLast > 0)
        Shoot.BulletToShoot -= BulletCount - BulletCountLast;
    BulletCountLast = BulletCount;
    if (Shoot.BulletToShoot < 0)
        Shoot.BulletToShoot = 0;
#endif

    switch (Shoot.ShootMode)
    {
    case NoShooting:
        Shoot.SpeedInBulletsPerSec = 0;
        Shoot.NumsOfOneShot = 0;
        Shoot.ShootDelayInMs = 0;
        break;

    case DebugShoot:
        Shoot.SpeedInBulletsPerSec = SpeedInBulletsPerSec;
        Shoot.NumsOfOneShot = NumsOfOneShot;
        Shoot.ShootDelayInMs = ShootDelayInMs;
        break;

    case OneShot:
        if (Shoot.FricAccel < -5000 || OneShotCount > 1000)
        {
            OneShotCount = 0;
            Shoot.ShootMode = NoShooting;
            Shoot.SpeedInBulletsPerSec = 0;
            Shoot.NumsOfOneShot = 0;
            Shoot.ShootDelayInMs = 0;
            break;
        }
        OneShotCount++;
        Shoot.SpeedInBulletsPerSec = 8;
        Shoot.NumsOfOneShot = 4;
        Shoot.ShootDelayInMs = 0;
        break;

    case CounterShot:
        Shoot.SpeedInBulletsPerSec = 12;
        Shoot.NumsOfOneShot = Shoot.BulletToShoot;
        Shoot.ShootDelayInMs = UINT32_MAX;
        break;

    case KeepShooting:
        Shoot.SpeedInBulletsPerSec = 8;
        Shoot.NumsOfOneShot = UINT32_MAX;
        Shoot.ShootDelayInMs = 0;
        break;

    case DebugSpinning:
        Shoot.RefFricSpeed = DebugAmp * sinf(DebugOmega * t) + DebugOffset;
        Shoot.SpeedInBulletsPerSec = 0;
        Shoot.NumsOfOneShot = UINT32_MAX;
        Shoot.ShootDelayInMs = 0;
        break;

    case AimAssistShooting:
        Shoot.SpeedInBulletsPerSec = 12;
        Shoot.NumsOfOneShot = UINT32_MAX;
        Shoot.ShootDelayInMs = 0;
        break;

    default:
        Shoot.SpeedInBulletsPerSec = 0;
        Shoot.NumsOfOneShot = 0;
        Shoot.ShootDelayInMs = 0;
        break;
    }

    rcLeftSwitchValueLast = remote_control.switch_left;
    LastPressLeft = remote_control.mouse.press_left;
    LastKeyCode = remote_control.key_code;
}

static void Shoot_Get_CtrlValue(void)
{
    static float heatGain = 2.5;
    static float LastBulletSpeed = 0;
    static float LowSpeedCount = 0;
    // static float BulletSpeed;
    static uint8_t heatornot;
    static float current_heat, warning_heat, max_heat;

    Shoot.BulletSpeedLimit = robot_state.shooter_id1_17mm_speed_limit;
    if (Shoot.BulletSpeedLimit > 30.5f)
        Shoot.BulletSpeedLimit /= 5.0f;

    BulletSpeed = Shoot.SpeedInBulletsPerSec;
    // ����ģʽ�µ�����Ƶ
    if (Gimbal.Mode == AimAssist_Mode ||
        (Shoot.SpeedInBulletsPerSec > 5))
    {
        if (ShootEvaluation.AbleToShoot)
        {
            if (HitSpinning.SpinningSpeedStatus == HighSpeedSpinning)
                BulletSpeed = 25;
            else
                BulletSpeed = 18 + ShootEvaluation.ShootFreqGain;
            Shoot.NumsOfOneShot = UINT32_MAX;
            Shoot.ShootDelayInMs = 0;
        }
        else if (Shoot.ShootMode == KeepShooting)
            BulletSpeed = Shoot.SpeedInBulletsPerSec + ShootEvaluation.ShootFreqGain;
        else
            BulletSpeed = 0;
    }
    // Shoot.TriggerSpeed = ShootAndDelay(&Shoot.TriggerMotor[0], BulletSpeed, Shoot.NumsOfOneShot, Shoot.ShootDelayInMs);

    if (robot_state.shooter_id1_17mm_cooling_limit < 20)
        robot_state.shooter_id1_17mm_cooling_limit = 240;
    if (robot_state.shooter_id2_17mm_cooling_limit < 20)
        robot_state.shooter_id2_17mm_cooling_limit = 240;

    if (!is_TOE_Error(JUDGE_TOE))
    {
        if (Shoot.Rotary.Shooter_id == Shooter1)
        {
            current_heat = power_heat_data_t.shooter_id1_17mm_cooling_heat;
            warning_heat = robot_state.shooter_id1_17mm_cooling_limit - 100;
            max_heat = robot_state.shooter_id1_17mm_cooling_limit;
            if (current_heat > warning_heat)
            {
                Shoot.FreqRatio = (current_heat - max_heat) / (warning_heat - max_heat);
            }
            else
            {
                Shoot.FreqRatio = 1;
            }
        }
        else
        {
            current_heat = power_heat_data_t.shooter_id2_17mm_cooling_heat;
            warning_heat = robot_state.shooter_id2_17mm_cooling_limit - 100;
            max_heat = robot_state.shooter_id2_17mm_cooling_limit;
            if (current_heat > warning_heat)
            {
                Shoot.FreqRatio = (current_heat - max_heat) / (warning_heat - max_heat);
            }
            else
            {
                Shoot.FreqRatio = 1;
            }
        }
        BulletSpeed = float_constrain(BulletSpeed, 0, 25 * Shoot.FreqRatio);
    }
    Shoot.TriggerSpeed = ShootAndDelay(&Shoot.TriggerMotor[0], BulletSpeed, Shoot.NumsOfOneShot, Shoot.ShootDelayInMs);

    Shoot.heatRemain = BulletSpeed * heatGain;
    if (Shoot.heatRemain < 35)
        Shoot.heatRemain = 35;

    heatornot = Heat_Judgment();
    if (!is_TOE_Error(JUDGE_TOE) && heatornot)
        Shoot.TriggerSpeed = 0;

    if (is_TOE_Error(JUDGE_TOE) && GlobalDebugMode != SHOOT_DEBUG && Shoot.SpeedInBulletsPerSec > 1)
        Shoot.TriggerSpeed = 15.0f / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60;

    Shoot.RefFricSpeed = -6800;

    if (Shoot.TriggerSpeed > 10 && Shoot.FricSpeed < fabsf(Shoot.RefFricSpeed) + Shoot.BulletSpeedCompensation)
    {
        Shoot.FricMotor[0].PID_Velocity.Ki = 0;
        Shoot.FricMotor[1].PID_Velocity.Ki = 0;
    }
    else
    {
        Shoot.FricMotor[0].PID_Velocity.Ki = Shoot.InitialKi;
        Shoot.FricMotor[1].PID_Velocity.Ki = Shoot.InitialKi;
    }
}

static void Shoot_Set_Control(void)
{
    static float last_fric_speed;
    float tempSpeed;

    if (Shoot.FricSpeed < (fabsf(Shoot.RefFricSpeed) + Shoot.BulletSpeedCompensation) * 0.95f || Shoot.Rotary.Shooter_state == Unfixed)
        Shoot.TriggerSpeed = 0;

    Shoot.TriggerSpeed = float_constrain(Shoot.TriggerSpeed, 0, 25.0f / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60);
    Shoot.TriggerSpeed1 = -Shoot.TriggerSpeed;
    // Shoot.TriggerSpeed1 = float_constrain(Shoot.TriggerSpeed1, -25.0f / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60, 25.0f / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60);
    // Shoot.TriggerSpeed2 = -0.7f * Shoot.TriggerSpeed;
    // Shoot.TriggerSpeed2 = float_constrain(Shoot.TriggerSpeed2, -25.0f / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60, 25.0f / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60);

    Motor_Speed_Calculate(&Shoot.TriggerMotor[0], Shoot.TriggerMotor[0].Velocity_RPM, Shoot.TriggerSpeed1);
    // Motor_Speed_Calculate(&Shoot.TriggerMotor[1], Shoot.TriggerMotor[1].Velocity_RPM, Shoot.TriggerSpeed2);

    Shoot.FricSpeed = (fabsf(Shoot.FricMotor[0].Velocity_RPM) + fabsf(Shoot.FricMotor[1].Velocity_RPM)) / 2.0f;
    Shoot.FricAccel = (Shoot.FricSpeed - last_fric_speed) / (0.01f + dt) + Shoot.FricAccel * 0.01f / (0.01f + dt);

    if (Shoot.isFricOn == 1)
    {
        tempSpeed = float_constrain(Shoot.RefFricSpeed + Shoot.BulletSpeedCompensation * Shoot.isFricOn, -FRIC_MOTOR_MAX_RPM, 0);
    }
    else
    {
        tempSpeed = 0;
    }
    TD_Calculate(&Shoot.FricTD, tempSpeed);
    Motor_Speed_Calculate(&Shoot.FricMotor[0], Shoot.FricMotor[0].Velocity_RPM, -Shoot.FricTD.x);
    Motor_Speed_Calculate(&Shoot.FricMotor[1], Shoot.FricMotor[1].Velocity_RPM, Shoot.FricTD.x);

    last_fric_speed = Shoot.FricSpeed;
}

static void Send_Shoot_Output(void)
{
    static uint32_t triggerMotorBlockCount1 = 0;
    static uint32_t triggerMotorBlockCount2 = 0;
    if (GlobalDebugMode != SHOOT_DEBUG)
    {
        if (Shoot.isLidOpen)
            TIM_Set_PWM(&htim8, TIM_CHANNEL_1, 1900);
        else
            TIM_Set_PWM(&htim8, TIM_CHANNEL_1, 700);
    }

    // FirstOrderSI_EKF_Tuning(&Shoot.FricMotorSI[0], Q0, Q1, Q2, R, 1);
    // FirstOrderSI_EKF_Tuning(&Shoot.FricMotorSI[1], Q0, Q1, Q2, R, 1);
    // FirstOrderSI_Update(&Shoot.FricMotorSI[0], Shoot.FricMotor[0].Output, Shoot.FricMotor[0].Velocity_RPM, dt);
    // FirstOrderSI_Update(&Shoot.FricMotorSI[1], Shoot.FricMotor[1].Output, Shoot.FricMotor[1].Velocity_RPM, dt);

    // ����������ߺ�ر�Ħ���֣����ⷢ���Դ�ϵ����Υ��
    if ((is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE) && GlobalDebugMode != SHOOT_DEBUG) || is_TOE_Error(TRIGGER_MOTOR1_TOE))
    {
        Send_Motor_Current_1_4(&hcan2, 0, 0, 0, 0);
    }
    else
    {
        if (Shoot.TriggerMotor[0].PID_Velocity.ERRORHandler.ERRORType == Motor_Blocked)
        {
            if (Shoot.TriggerMotor[0].Direction == NEGATIVE)
                Shoot.TriggerMotor[0].Output = -5000;
            else
                Shoot.TriggerMotor[0].Output = 5000;
            triggerMotorBlockCount1++;
            if (triggerMotorBlockCount1 > 100)
            {
                Shoot.TriggerMotor[0].PID_Velocity.ERRORHandler.ERRORCount = 0;
                Shoot.TriggerMotor[0].PID_Velocity.ERRORHandler.ERRORType = PID_ERROR_NONE;
            }
        }
        else
            triggerMotorBlockCount1 = 0;

        // if (Shoot.TriggerMotor[1].PID_Velocity.ERRORHandler.ERRORType == Motor_Blocked)
        // {
        //     if (Shoot.TriggerMotor[1].Direction == NEGATIVE)
        //         Shoot.TriggerMotor[1].Output = -3000;
        //     else
        //         Shoot.TriggerMotor[1].Output = 3000;
        //     triggerMotorBlockCount2++;
        //     if (triggerMotorBlockCount2 > 100)
        //     {
        //         Shoot.TriggerMotor[1].PID_Velocity.ERRORHandler.ERRORCount = 0;
        //         Shoot.TriggerMotor[1].PID_Velocity.ERRORHandler.ERRORType = PID_ERROR_NONE;
        //     }
        // }
        // else
        //     triggerMotorBlockCount2 = 0;

        Send_Motor_Current_1_4(&hcan2, Shoot.TriggerMotor[0].Output, Shoot.Rotary.RotaryMotor.Output,
                               Shoot.FricMotor[0].Output, Shoot.FricMotor[1].Output);
        // Send_Motor_Current_1_4(&hcan2, 0, 0, 0, 0);
    }
}

static void DoubleBarrel_Coutrol(void)
{
    Barrel_Switch();
    Barrel_Mode_Switch();
    DoubleBarrel_SetOutput();
}

static void Barrel_Switch(void)
{
    static uint8_t key_count, switch_enable;
    static uint8_t turnornot;
    static uint32_t shootTick;

    turnornot = Barrel_SwitchByRC();
    if (Shoot.FricAccel < -5000.0f)
        shootTick = INS_GetTimeline();

    if (((power_heat_data_t.shooter_id1_17mm_cooling_heat + Shoot.heatRemain) >= (robot_state.shooter_id1_17mm_cooling_limit)) ||
        ((remote_control.key_code & Key_Q) && (LastKeyCode != Key_Q) && (key_count == RESET)) || Shoot.Rotary.Barrel_Debug == Shooter2 || turnornot == Shooter2)
    {
        switch_enable = 1;
    }
    else if (((power_heat_data_t.shooter_id2_17mm_cooling_heat + Shoot.heatRemain) >= (robot_state.shooter_id2_17mm_cooling_limit)) ||
             ((remote_control.key_code & Key_Q) && (LastKeyCode != Key_Q) && (key_count == SET)) || Shoot.Rotary.Barrel_Debug == Shooter1 || turnornot == Shooter1)
    {
        switch_enable = 2;
    }

    if (switch_enable == 1 && INS_GetTimeline() - shootTick > 150)
    {
        Shoot.Rotary.Shooter_id = Shooter2;
        Shoot.Rotary.Rotary_Output_Coefficient = -1;
        key_count = SET;
        switch_enable = 0;
    }
    else if (switch_enable == 2 && INS_GetTimeline() - shootTick > 150)
    {
        Shoot.Rotary.Shooter_id = Shooter1;
        Shoot.Rotary.Rotary_Output_Coefficient = 1;
        key_count = RESET;
        switch_enable = 0;
    }
}

static void Barrel_Mode_Switch(void)
{
    static uint8_t Use_Selfrecovery = TRUE;
    Shoot.Rotary.current_judge = Shoot.Rotary.RotaryMotor.Real_Current * dt / (dt + Shoot.Rotary.current_lpf) + Shoot.Rotary.current_last * Shoot.Rotary.current_lpf / (dt + Shoot.Rotary.current_lpf);
    float lpf = 0.01;
    Shoot.Rotary.RotaryMotorVelocity = Shoot.Rotary.RotaryMotor.Velocity_RPM * dt / (lpf + dt) + Shoot.Rotary.RotaryMotorVelocity * lpf / (lpf + dt);
    if (fabsf(Shoot.Rotary.RotaryMotorVelocity) <= 50 && fabsf(Shoot.Rotary.current_judge) > 1400.0f)
    {
        Shoot.Rotary.Resume_timing++;
        if (Shoot.Rotary.Resume_timing > Shoot.Rotary.Recovery_time)
        {
            Shoot.Rotary.Shooter_state = BarrelNormal;
        }
    }
    else
    {
        Shoot.Rotary.Shooter_state = Unfixed;
        Shoot.Rotary.Resume_timing = 0;
    }

    if (Use_Selfrecovery)
    {
        Stallad_Self_Recovery();
    }
    Shoot.Rotary.current_last = Shoot.Rotary.current_judge;
    Shoot.Rotary.Rotary_Output = float_constrain(Shoot.Rotary.Rotary_Output, -Shoot.Rotary.Rotary_Output_MAX, Shoot.Rotary.Rotary_Output_MAX);
    Shoot.Rotary.motor_last_current = Shoot.Rotary.RotaryMotor.Real_Current;
    Shoot.Rotary.Last_Shooterid = Shoot.Rotary.Shooter_id;
}

static void Stallad_Self_Recovery(void)
{
    static int16_t temp_count = 0;
    if (remote_control.key_code & Key_G)
    {
        temp_count++;
        if (temp_count > 100)
        {
            Shoot.Rotary.Shooter_state = Stallad;
        }
    }
    else
    {
        temp_count = 0;
    }
}

static void DoubleBarrel_SetOutput(void)
{
    switch (Shoot.Rotary.Shooter_state)
    {
    case Unfixed:
    {
        Shoot.Rotary.Rotary_Output = Shoot.Rotary.slew_output * Shoot.Rotary.Rotary_Output_Coefficient;
        break;
    }
    case BarrelNormal:
    {
        Shoot.Rotary.Rotary_Output = Shoot.Rotary.fixed_output * Shoot.Rotary.Rotary_Output_Coefficient;
        break;
    }
    case Stallad:
    {
        Shoot.Rotary.Rotary_Output = 0;
        break;
    }
    default:
        break;
    }
#ifdef USE_CLOSEDLOOP
    Motor_Speed_Calculate(&Shoot.RotaryMotor, Shoot.RotaryMotor.Velocity_RPM, Shoot.Rotary.Rotary_Output);
#else
    Shoot.Rotary.RotaryMotor.Output = Shoot.Rotary.Rotary_Output;
#endif
}

static uint8_t Barrel_SwitchByRC(void)
{
    static uint8_t count = 1;
    static uint8_t switchornot;
    static int16_t last_remote_data;
    if (remote_control.switch_right == Switch_Down && remote_control.ch4 == -660 && last_remote_data != -660)
    {
        count = !count;
        switch (count)
        {
        case 0:
        {
            switchornot = 2;
            break;
        }
        case 1:
        {
            switchornot = 1;
            break;
        }
        default:
            break;
        }
    }
    if (remote_control.switch_right != Switch_Down)
    {
        switchornot = FALSE;
    }
    last_remote_data = remote_control.ch4;

    return switchornot;
}

static uint8_t Heat_Judgment(void)
{
    static uint8_t TrueOrFalse;
    if (((Shoot.Rotary.Shooter_state != BarrelNormal) ||
         ((power_heat_data_t.shooter_id1_17mm_cooling_heat + Shoot.heatRemain >= robot_state.shooter_id1_17mm_cooling_limit) && (Shoot.Rotary.Shooter_id == 1)) ||
         ((power_heat_data_t.shooter_id2_17mm_cooling_heat + Shoot.heatRemain >= robot_state.shooter_id2_17mm_cooling_limit) && (Shoot.Rotary.Shooter_id == 2))) &&
        !Shoot.Rotary.Rotary_debug)
        TrueOrFalse = TRUE;
    else
    {
        TrueOrFalse = FALSE;
    }

    return TrueOrFalse;
}
/**
 * @brief          ����������Ƶ���㲦������ٶȣ���ʵ����������������Ƶ�������������ӵ�
 * @param[1]       ����ṹ��ָ��
 * @param[2]       ��Ƶ����λ����/s
 * @param[3]       ��������ӵ���
 * @param[4]       ����������ʱ��
 * @retval         ������������ٶ� ��λ��RPM
 */
float ShootAndDelay(Motor_t *trigger, float speedInNumsPerSec, uint32_t numsOfOneShot, uint32_t delayTimeInMs)
{
    static uint32_t ticksInMs = 0, lastNumsOfOneShot = 0, lastDelayTimeInMs = 0, count = 0;
    static int32_t lastAngle = 0;
    static float speed = 0;
    if (count == 0 || lastNumsOfOneShot != numsOfOneShot || lastDelayTimeInMs != delayTimeInMs)
    {
        ticksInMs = USER_GetTimeline() + delayTimeInMs + 1;
        lastAngle = abs(trigger->total_angle);
    }
    if (abs(trigger->total_angle) - lastAngle > 8191 * TRIGGER_MOTOR_REDUCTION_RATIO / BULLETS_PER_ROUND * numsOfOneShot)
    {
        lastAngle = abs(trigger->total_angle);
        speed = 0;
        ticksInMs = USER_GetTimeline();
    }

    if (USER_GetTimeline() - ticksInMs > delayTimeInMs)
        speed = speedInNumsPerSec / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60;

    count++;
    lastNumsOfOneShot = numsOfOneShot;
    lastDelayTimeInMs = delayTimeInMs;
    return speed;
}

/**
 * @brief          ʹ��������Թ̶��ٶ���תһ���Ƕ�
 * @param[1]       ������
 * @param[2]       �����������ת�� ��λ��RPM
 * @param[3]       ������������ת���ĽǶ� ��λ���ȡ�
 * @retval         ������������ٶ� ��λ��RPM
 */
float RotateAngleInDegree(Motor_t *trigger, float speedRPM, float angleInDegree)
{
    static uint32_t count = 0;
    static int32_t lastAngle = 0;
    static float speed = 0;
    if (count == 0)
        lastAngle = trigger->total_angle;
    count++;
    if (angleInDegree > 0)
    {
        speed = speedRPM;
        if (trigger->total_angle - lastAngle > angleInDegree / 360 * 8191 * TRIGGER_MOTOR_REDUCTION_RATIO)
        {
            count = 0;
            speed = 0;
        }
    }
    if (angleInDegree < 0)
    {
        speed = -speedRPM;
        if (trigger->total_angle - lastAngle < angleInDegree / 360 * 8191 * TRIGGER_MOTOR_REDUCTION_RATIO)
        {
            count = 0;
            speed = 0;
        }
    }
    return speed;
}
