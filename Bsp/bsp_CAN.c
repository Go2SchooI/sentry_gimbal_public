/**
 ******************************************************************************
 * @file    bsp_CAN.c
 * @author  Wang Hongxi
 * @version V1.5.0
 * @date    2021/4/20
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_CAN.h"
#include "bsp_dwt.h"

uint8_t FOLLOW_Data_Buf[16] = {0};
uint8_t FOLLOW_Update = 0;
uint16_t enemy_outpost_HP = 1500;
uint8_t JudgeRxDataValid = 0;
float theta;
int16_t PlanDot[2][2];
int16_t PosDot[2];
uint8_t InPosCount;
uint8_t InPosFlag;
uint8_t DataFromAerial[16];

/**
 * @Func		CAN_Device_Init
 * @Brief
 * @Param		CAN_HandleTypeDef* hcan
 * @Retval		None
 * @Date       2019/11/4
 **/
// void CAN_Device_Init(CAN_HandleTypeDef *_hcan)
// {
//     // ��ʼ��CAN������Ϊ������״̬ ��Ϊ�������� �����
//     CAN_FilterTypeDef can_filter_st;
//     can_filter_st.FilterActivation = ENABLE;
//     can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//     can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//     can_filter_st.FilterIdHigh = 0x0000;
//     can_filter_st.FilterIdLow = 0x0000;
//     can_filter_st.FilterMaskIdHigh = 0x0000;
//     can_filter_st.FilterMaskIdLow = 0x0000;
//     can_filter_st.FilterBank = 0;
//     can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

//     // CAN2��CAN1��FilterBank��ͬ
//     if (_hcan == &hcan2)
//     {
//         can_filter_st.SlaveStartFilterBank = 14;
//         can_filter_st.FilterBank = 14;
//     }

//     // CAN ��������ʼ��
//     HAL_CAN_ConfigFilter(_hcan, &can_filter_st);
//     // ����CAN
//     HAL_CAN_Start(_hcan);
//     // ����֪ͨ
//     HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
// }
void CAN_Device_Init(void)
{
    // ��ʼ��CAN������Ϊ������״̬ ��Ϊ�������� �����
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    // CAN ��������ʼ��
    while (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
    }
    // ����CAN
    while (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
    }
    // ����֪ͨ
    while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    // CAN ��������ʼ��
    while (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
    {
    }
    // ����CAN
    while (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
    }
    // ����֪ͨ
    while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }
}

/**
 * @Func	    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
 * @Brief      CAN���߽��ջص����� ���ڽ��յ������
 * @Param	    CAN_HandleTypeDef* _hcan
 * @Retval	    None
 * @Date       2019/11/4
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    static uint8_t RC_Data_Buf[16];
    static uint8_t SLAM_Data_Buf[4];

    HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (_hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        case SWITCH_MOTOR_ID:
            // �Ե����������Ϣ�����Ի�ȡ�����Ϣ
            if (Shoot.Rotary.RotaryMotor.msg_cnt++ <= 50)
                get_moto_offset(&Shoot.Rotary.RotaryMotor, rx_data);
            else
                get_moto_info(&Shoot.Rotary.RotaryMotor, rx_data);
            Detect_Hook(SWITCH_MOTOR_TOE);
            break;
        case FRIC_RM3508_LEFT_ID:
            // �Ե����������Ϣ�����Ի�ȡ�����Ϣ
            if (Shoot.FricMotor[0].msg_cnt++ <= 50)
                get_moto_offset(&Shoot.FricMotor[0], rx_data);
            else
                get_moto_info(&Shoot.FricMotor[0], rx_data);
            break;
        case FRIC_RM3508_RIGHT_ID:
            // �Ե����������Ϣ�����Ի�ȡ�����Ϣ
            if (Shoot.FricMotor[1].msg_cnt++ <= 50)
                get_moto_offset(&Shoot.FricMotor[1], rx_data);
            else
                get_moto_info(&Shoot.FricMotor[1], rx_data);
            break;
        case TRIGGER_MOTOR_LEFT_ID:
            if (Shoot.TriggerMotor[0].msg_cnt++ <= 50)
                get_moto_offset(&Shoot.TriggerMotor[0], rx_data);
            else
                get_moto_info(&Shoot.TriggerMotor[0], rx_data);
            Detect_Hook(TRIGGER_MOTOR1_TOE);
            break;
        case PITCH_MOTOR_ID:
            // �Ե����������Ϣ�����Ի�ȡ�����Ϣ
            if (Gimbal.PitchMotor.msg_cnt++ <= 50)
                get_moto_offset(&Gimbal.PitchMotor, rx_data);
            else
                get_moto_info(&Gimbal.PitchMotor, rx_data);
            Detect_Hook(GIMBAL_PITCH_MOTOR_TOE);
            break;
        // case 0x281:
        //     GetAngularVelocity(rx_data);
        //     FOLLOW_Update = 1;
        //     break;
        case 0x221:
            FOLLOW_Data_Buf[0] = rx_data[0]; // CF_SOF
            FOLLOW_Data_Buf[1] = rx_data[1]; // POSX
            FOLLOW_Data_Buf[2] = rx_data[2]; // POSX
            FOLLOW_Data_Buf[3] = rx_data[3]; // POSY
            FOLLOW_Data_Buf[4] = rx_data[4]; // POSY
            FOLLOW_Data_Buf[5] = rx_data[5]; // YAW
            FOLLOW_Data_Buf[6] = rx_data[6]; // YAW
            FOLLOW_Data_Buf[7] = rx_data[7]; // planX 1
            theta = (int16_t)((rx_data[6] << 8) | rx_data[5]) / 1000.0f;
            // if (FOLLOW_Data_Buf[0] == 0x66 && FOLLOW_Data_Buf[7] == 0x88)
            //     FOLLOW_Update = 1;
            break;

        case 0x222:
            FOLLOW_Data_Buf[8] = rx_data[0];  // planX 1
            FOLLOW_Data_Buf[9] = rx_data[1];  // planY 1
            FOLLOW_Data_Buf[10] = rx_data[2]; // planY 1
            FOLLOW_Data_Buf[11] = rx_data[3]; // planX 2
            FOLLOW_Data_Buf[12] = rx_data[4]; // planX 2
            FOLLOW_Data_Buf[13] = rx_data[5]; // planY 2
            FOLLOW_Data_Buf[14] = rx_data[6]; // planY 2
            FOLLOW_Data_Buf[15] = rx_data[7]; // CF_EOF
            if (FOLLOW_Data_Buf[0] == 0x66 && FOLLOW_Data_Buf[15] == 0x88)
                FOLLOW_Update = 1;
            PlanDot[0][0] = (int16_t)((FOLLOW_Data_Buf[8] << 8) | (FOLLOW_Data_Buf[7]));
            PlanDot[0][1] = (int16_t)((FOLLOW_Data_Buf[10] << 8) | (FOLLOW_Data_Buf[9]));
            PlanDot[1][0] = (int16_t)((FOLLOW_Data_Buf[12] << 8) | (FOLLOW_Data_Buf[11]));
            PlanDot[1][1] = (int16_t)((FOLLOW_Data_Buf[14] << 8) | (FOLLOW_Data_Buf[13]));
            PosDot[0] = (int16_t)((FOLLOW_Data_Buf[2] << 8) | (FOLLOW_Data_Buf[1]));
            PosDot[1] = (int16_t)((FOLLOW_Data_Buf[4] << 8) | (FOLLOW_Data_Buf[3]));
            if ((fabs(PosDot[0] - PlanDot[0][0]) <= 10) && (fabs(PosDot[1] - PlanDot[0][1]) <= 10))
            {
                InPosCount++;
                if (InPosCount >= 500)
                {
                    InPosFlag = 1;
                    InPosCount = 500;
                }
            }
            else
            {
                InPosCount = 0;
                InPosFlag = 0;
            }
            break;
        }
    }

    if (_hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        // �����������ư巢����ң��������
        case CAN_RC_DATA_Frame_0:
            RC_Data_Buf[0] = rx_data[0];
            RC_Data_Buf[1] = rx_data[1];
            RC_Data_Buf[2] = rx_data[2];
            RC_Data_Buf[3] = rx_data[3];
            RC_Data_Buf[4] = rx_data[4];
            RC_Data_Buf[5] = rx_data[5];
            RC_Data_Buf[6] = rx_data[6];
            RC_Data_Buf[7] = rx_data[7];
            break;
        case CAN_RC_DATA_Frame_1:
            RC_Data_Buf[8] = rx_data[0];
            RC_Data_Buf[9] = rx_data[1];
            RC_Data_Buf[10] = rx_data[2];
            RC_Data_Buf[11] = rx_data[3];
            RC_Data_Buf[12] = rx_data[4];
            RC_Data_Buf[13] = rx_data[5];
            RC_Data_Buf[14] = rx_data[6];
            RC_Data_Buf[15] = rx_data[7];
            Callback_RC_Handle(&remote_control, RC_Data_Buf);
            break;
        case YAW_MOTOR_ID:
            Detect_Hook(GIMBAL_YAW_MOTOR_TOE);
            get_RMD_info(&Gimbal.YawMotor, rx_data);
            break;
        // case TRIGGER_MOTOR_RIGHT_ID:
        //     // �Ե����������Ϣ�����Ի�ȡ�����Ϣ
        //     if (Shoot.TriggerMotor[1].msg_cnt++ <= 50)
        //         get_moto_offset(&Shoot.TriggerMotor[1], rx_data);
        //     else
        //         get_moto_info(&Shoot.TriggerMotor[1], rx_data);
        //     Detect_Hook(TRIGGER_MOTOR2_TOE);
        //     break;
        case 0x133:
            // �Ե����������Ϣ�����Ի�ȡ�����Ϣ
            robot_state.robot_id = rx_data[0];
            robot_state.shooter_id1_17mm_cooling_limit = (uint16_t)((rx_data[1] << 8) | rx_data[2]);
            power_heat_data_t.shooter_id1_17mm_cooling_heat = (uint16_t)((rx_data[3] << 8) | rx_data[4]);
            shoot_data.bullet_speed = (uint16_t)((rx_data[5] << 8) | rx_data[6]) / 10.0f;
            robot_state.shooter_id1_17mm_speed_limit = rx_data[7];
            Detect_Hook(JUDGE_TOE);
            break;
        case 0x134:
            robot_state.shooter_id2_17mm_cooling_limit = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
            power_heat_data_t.shooter_id2_17mm_cooling_heat = (uint16_t)((rx_data[2] << 8) | rx_data[3]);
            robot_state.shooter_id2_17mm_speed_limit = rx_data[4];
            game_status.game_progress = rx_data[5];
            enemy_outpost_HP = (uint16_t)((rx_data[6] << 8) | rx_data[7]);
            Detect_Hook(JUDGE_TOE);
            break;
        case 0x235:
            JudgeRxData[0] = rx_data[0];
            JudgeRxData[1] = rx_data[1];
            JudgeRxDataValid = 1;
            break;
        case CAN_AERIAL_DATA_1:
            DataFromAerial[0] = rx_data[0];
            DataFromAerial[1] = rx_data[1];
            DataFromAerial[2] = rx_data[2];
            DataFromAerial[3] = rx_data[3];
            DataFromAerial[4] = rx_data[4];
            DataFromAerial[5] = rx_data[5];
            DataFromAerial[6] = rx_data[6];
            DataFromAerial[7] = rx_data[7];
            break;
        case CAN_AERIAL_DATA_2:
            DataFromAerial[8] = rx_data[0];
            DataFromAerial[15] = rx_data[7];
            break;
        }
    }
}

// ͨ��CAN���߷���ң������Ϣ ��������δʹ��
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_RC_DATA_Frame_0;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = rc_data[0];
    CAN_Send_Data[1] = rc_data[1];
    CAN_Send_Data[2] = rc_data[2];
    CAN_Send_Data[3] = rc_data[3];
    CAN_Send_Data[4] = rc_data[4];
    CAN_Send_Data[5] = rc_data[5];
    CAN_Send_Data[6] = rc_data[6];
    CAN_Send_Data[7] = rc_data[7];

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

    TX_MSG.StdId = CAN_RC_DATA_Frame_1;
    CAN_Send_Data[0] = rc_data[8];
    CAN_Send_Data[1] = rc_data[9];
    CAN_Send_Data[2] = rc_data[10];
    CAN_Send_Data[3] = rc_data[11];
    CAN_Send_Data[4] = rc_data[12];
    CAN_Send_Data[5] = rc_data[13];
    CAN_Send_Data[6] = rc_data[14];
    CAN_Send_Data[7] = rc_data[15];

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Robot_Info(CAN_HandleTypeDef *_hcan, int8_t ID, uint16_t heatLimit, uint16_t heat, uint16_t bulletSpeed)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x430;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = ID;
    CAN_Send_Data[1] = (heatLimit << 8) & 0xFF;
    CAN_Send_Data[2] = heatLimit & 0xFF;
    CAN_Send_Data[3] = (heat << 8) & 0xFF;
    CAN_Send_Data[4] = heat & 0xFF;
    CAN_Send_Data[5] = (bulletSpeed << 8) & 0xFF;
    CAN_Send_Data[6] = bulletSpeed & 0xFF;
    CAN_Send_Data[7] = 0;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Gimbal_Info(CAN_HandleTypeDef *_hcan, uint8_t aim_flg)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x152;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = aim_flg;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Reset_Command(CAN_HandleTypeDef *_hcan)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_SYSTEM_RESET_CMD;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = 0;
    CAN_Send_Data[1] = 0;
    CAN_Send_Data[2] = 0;
    CAN_Send_Data[3] = 0;
    CAN_Send_Data[4] = 0;
    CAN_Send_Data[5] = 0;
    CAN_Send_Data[6] = 0;
    CAN_Send_Data[7] = 0;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Follow_Data_1(CAN_HandleTypeDef *_hcan, uint8_t *FOLLOW_Data_Buf)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x151;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = FOLLOW_Data_Buf[0]; // CF_SOF
    CAN_Send_Data[1] = FOLLOW_Data_Buf[1]; // POSX
    CAN_Send_Data[2] = FOLLOW_Data_Buf[2]; // POSX
    CAN_Send_Data[3] = FOLLOW_Data_Buf[3]; // POSY
    CAN_Send_Data[4] = FOLLOW_Data_Buf[4]; // POSY
    CAN_Send_Data[5] = FOLLOW_Data_Buf[5]; // YAW
    CAN_Send_Data[6] = FOLLOW_Data_Buf[6]; // YAW
    CAN_Send_Data[7] = FOLLOW_Data_Buf[7]; // planX 1

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Follow_Data_2(CAN_HandleTypeDef *_hcan, uint8_t *FOLLOW_Data_Buf)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x150;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = FOLLOW_Data_Buf[8];  // planX 1
    CAN_Send_Data[1] = FOLLOW_Data_Buf[9];  // planY 1
    CAN_Send_Data[2] = FOLLOW_Data_Buf[10]; // planY 1
    CAN_Send_Data[3] = FOLLOW_Data_Buf[11]; // planX 2
    CAN_Send_Data[4] = FOLLOW_Data_Buf[12]; // planX 2
    CAN_Send_Data[5] = FOLLOW_Data_Buf[13]; // planY 2
    CAN_Send_Data[6] = FOLLOW_Data_Buf[14]; // planY 2
    CAN_Send_Data[7] = FOLLOW_Data_Buf[15]; // mode

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Game_Status(CAN_HandleTypeDef *_hcan, uint8_t GameStatus)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = 0x215;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = GameStatus;

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Aerial_Data(CAN_HandleTypeDef *_hcan)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_AERIAL_DATA_1;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = DataFromAerial[0]; // XData
    CAN_Send_Data[1] = DataFromAerial[1]; // XData
    CAN_Send_Data[2] = DataFromAerial[2]; // XData
    CAN_Send_Data[3] = DataFromAerial[3]; // XData
    CAN_Send_Data[4] = DataFromAerial[4]; // YData
    CAN_Send_Data[5] = DataFromAerial[5]; // YData
    CAN_Send_Data[6] = DataFromAerial[6]; // YData
    CAN_Send_Data[7] = DataFromAerial[7]; // YData

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

    TX_MSG.StdId = CAN_AERIAL_DATA_2;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    CAN_Send_Data[0] = DataFromAerial[8]; // commd_keyboard
    CAN_Send_Data[7] = DataFromAerial[15];

    while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
    {
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
    {
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
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}