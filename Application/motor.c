#include "motor.h"
#include "includes.h"

uint8_t RMD_data[16];
uint16_t RMD_Angle;

float Motor_Torque_Calculate(Motor_t *motor, float torque, float target_torque)
{
    Feedforward_Calculate(&motor->FFC_Torque, target_torque);

    PID_Calculate(&motor->PID_Torque, torque, target_torque);

    if (motor->TorqueCtrl_User_Func_f != NULL)
        motor->TorqueCtrl_User_Func_f(motor);

    if (motor->Direction != NEGATIVE)
        motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output + motor->Ke * motor->Velocity_RPM;
    else
        motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output - motor->Ke * motor->Velocity_RPM;
    // ����޷�
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Speed_Calculate(Motor_t *motor, float velocity, float target_speed)
{
    Feedforward_Calculate(&motor->FFC_Velocity, target_speed);
    PID_Calculate(&motor->PID_Velocity, velocity, target_speed);
    LDOB_Calculate(&motor->LDOB, velocity, motor->Output);

    if (motor->SpeedCtrl_User_Func_f != NULL)
        motor->SpeedCtrl_User_Func_f(motor);

    motor->Output = motor->FFC_Velocity.Output + motor->PID_Velocity.Output - motor->LDOB.Disturbance;
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Angle_Calculate(Motor_t *motor, float angle, float velocity, float target_angle)
{
    Feedforward_Calculate(&motor->FFC_Angle, target_angle);
    PID_Calculate(&motor->PID_Angle, angle, target_angle);

    if (motor->AngleCtrl_User_Func_f != NULL)
        motor->AngleCtrl_User_Func_f(motor);

    Motor_Speed_Calculate(motor, velocity, motor->FFC_Angle.Output + motor->PID_Angle.Output);

    return motor->Output;
}

/**
 * @Func	    void get_moto_info(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
 * @Brief      process data received from CAN
 * @Param	    Motor_t *ptr  CAN_HandleTypeDef *_hcan
 * @Retval	    None
 * @Date       2019/11/5
 **/
void get_moto_info(Motor_t *ptr, uint8_t *aData)
{
    if (ptr->Direction != NEGATIVE)
    {
        ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = (int16_t)(aData[2] << 8 | aData[3]);
    }
    else
    {
        ptr->RawAngle = 8191 - (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = -(int16_t)(aData[2] << 8 | aData[3]);
    }

    if (ptr->ReductionRatio > 1e-6f)
        ptr->OutputVel_RadPS = ptr->Velocity_RPM * 0.10471975511965f / ptr->ReductionRatio;

    ptr->Real_Current = (int16_t)(aData[4] << 8 | aData[5]);
    ptr->Temperature = aData[6];

    if (ptr->RawAngle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->RawAngle - ptr->last_angle < -4096)
        ptr->round_cnt++;

    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -4095, 4096);

    ptr->AngleInDegree = ptr->Angle * 0.0439507f;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->RawAngle - ptr->offset_angle;

    ptr->last_angle = ptr->RawAngle; // update last_angle
}

/*this function should be called after system+can init */
void get_moto_offset(Motor_t *ptr, uint8_t *aData)
{
    ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
    ptr->offset_angle = ptr->RawAngle;
    ptr->last_angle = ptr->RawAngle; // update last_angle
}

/**
 * @Func	    void get_moto_info(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
 * @Brief      process data received from CAN
 * @Param	    Motor_t *ptr  CAN_HandleTypeDef *_hcan
 * @Retval	    None
 * @Date       2019/11/5
 **/
void get_RMD_info(Motor_t *ptr, uint8_t *aData)
{
    if (ptr->Direction != NEGATIVE)
    {
        ptr->RawAngle = (uint16_t)(aData[7] << 8 | aData[6]);
        ptr->Velocity_RPM = (int16_t)(aData[5] << 8 | aData[4]) * 0.159154943f;
    }
    else
    {
        ptr->RawAngle = 32767 - (uint16_t)(aData[7] << 8 | aData[6]);
        ptr->Velocity_RPM = -(int16_t)(aData[5] << 8 | aData[4]) * 0.159154943f;
    }

    ptr->Real_Current = (int16_t)(aData[3] << 8 | aData[2]);
    ptr->Temperature = aData[1];

    ptr->OutputVel_RadPS = ptr->Velocity_RPM * 0.10471975511965f;

    if (ptr->RawAngle - ptr->last_angle > 32767)
        ptr->round_cnt--;
    else if (ptr->RawAngle - ptr->last_angle < -32767)
        ptr->round_cnt++;

    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -32767, 32767);

    ptr->AngleInDegree = ptr->Angle * 0.00549324788281f;

    ptr->total_angle = ptr->round_cnt * 65535 + ptr->RawAngle - ptr->offset_angle;

    ptr->last_angle = ptr->RawAngle; // update last_angle
}

/*this function should be called after system+can init */
void get_RMD_offset(Motor_t *ptr, uint8_t *aData)
{
    ptr->RawAngle = (uint16_t)(aData[7] << 8 | aData[6]);
    ptr->offset_angle = ptr->RawAngle;
    ptr->last_angle = ptr->RawAngle; // update last_angle
}

/**
  * @Func	    void Send_RMD_Current(CAN_HandleTypeDef* hcan,
                                    int16_t c1, int16_t c2, int16_t c3, int16_t c4)
  * @Brief
  * @Param	    cx x=1,2,3,4
  * @Retval	    None
  * @Date       2019/11/5
 **/
HAL_StatusTypeDef Send_RMD_Current(CAN_HandleTypeDef *_hcan,
                                   int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x280;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    CAN_Send_Data[0] = c1;
    CAN_Send_Data[1] = (c1 >> 8);
    CAN_Send_Data[2] = c2;
    CAN_Send_Data[3] = (c2 >> 8);
    CAN_Send_Data[4] = c3;
    CAN_Send_Data[5] = (c3 >> 8);
    CAN_Send_Data[6] = c4;
    CAN_Send_Data[7] = (c4 >> 8);

    uint32_t count = 0;
    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
        count++;
        if (count > 10000)
            break;
    }
    count = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
        count++;
        if (count > 10000)
            break;
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX0;
    }
    /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX1;
    }

    /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

/**
  * @Func	    void Send_RMD_Current(CAN_HandleTypeDef* hcan,
                                    int16_t c1, int16_t c2, int16_t c3, int16_t c4)
  * @Brief
  * @Param	    cx x=1,2,3,4
  * @Retval	    None
  * @Date       2019/11/5
 **/
HAL_StatusTypeDef Send_RMD_Current_Single(CAN_HandleTypeDef *_hcan,
                                          int8_t ID, int16_t c)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x140 + ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    CAN_Send_Data[0] = 0xA1;
    CAN_Send_Data[1] = 0;
    CAN_Send_Data[2] = 0;
    CAN_Send_Data[3] = 0;
    CAN_Send_Data[4] = c;
    CAN_Send_Data[5] = (c >> 8);
    CAN_Send_Data[6] = 0;
    CAN_Send_Data[7] = 0;

    uint32_t count = 0;
    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
        count++;
        if (count > 10000)
            break;
    }
    count = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
        count++;
        if (count > 10000)
            break;
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX0;
    }
    /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX1;
    }

    /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

/**
  * @Func	    void Send_Motor_Current(CAN_HandleTypeDef* hcan,
                                    int16_t c1, int16_t c2, int16_t c3, int16_t c4)
  * @Brief
  * @Param	    cx x=1,2,3,4
  * @Retval	    None
  * @Date       2019/11/5
 **/
HAL_StatusTypeDef Send_Motor_Current_1_4(CAN_HandleTypeDef *_hcan,
                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_Transmit_1_4_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;

    CAN_Send_Data[0] = (c1 >> 8);
    CAN_Send_Data[1] = c1;
    CAN_Send_Data[2] = (c2 >> 8);
    CAN_Send_Data[3] = c2;
    CAN_Send_Data[4] = (c3 >> 8);
    CAN_Send_Data[5] = c3;
    CAN_Send_Data[6] = (c4 >> 8);
    CAN_Send_Data[7] = c4;

    uint32_t count = 0;
    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
        count++;
        if (count > 10000)
            break;
    }
    count = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
        count++;
        if (count > 10000)
            break;
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX0;
    }
    /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX1;
    }

    /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

HAL_StatusTypeDef Send_Motor_Current_5_8(CAN_HandleTypeDef *_hcan,
                                         int16_t c1, int16_t c2, int16_t c3, int16_t c4)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_Transmit_5_8_ID;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = (c1 >> 8);
    CAN_Send_Data[1] = c1;
    CAN_Send_Data[2] = (c2 >> 8);
    CAN_Send_Data[3] = c2;
    CAN_Send_Data[4] = (c3 >> 8);
    CAN_Send_Data[5] = c3;
    CAN_Send_Data[6] = (c4 >> 8);
    CAN_Send_Data[7] = c4;

    uint32_t count = 0;
    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
        count++;
        if (count > 10000)
            break;
    }
    count = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
        count++;
        if (count > 10000)
            break;
    }
    /* Check Tx Mailbox 1 status */
    if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX0;
    }
    /* Check Tx Mailbox 1 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX1;
    }

    /* Check Tx Mailbox 2 status */
    else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    {
        send_mail_box = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

/**
 * @brief       convert output torque to int16_t current
 * @param[in]   motor: pointer to A1_t
 */
int16_t M3508_Current_Solver(Motor_t *motor, float torque)
{
    float raw_torque = torque / (3591 / 187.0f) * motor->ReductionRatio;
    float current = raw_torque / 0.3f;
    return current / 20.0f * 16384;
}
/**
 * @brief       convert output torque to int16_t current
 * @param[in]   motor: pointer to A1_t
 */
int16_t RMD9025V1_Current_Solver(float torque)
{
    return torque * RMD9025V1_CURRENT_COEF;
}
int16_t RMD9025V2_Current_Solver(float torque)
{
    return torque * RMD9025V2_CURRENT_COEF;
}
