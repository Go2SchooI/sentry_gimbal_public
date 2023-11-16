/**
 ******************************************************************************
 * @file	 bsp_CAN.h
 * @author  Wang Hongxi
 * @version V1.5.0
 * @date    2021/4/20
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "includes.h"

#define CAN_RC_DATA_Frame_0 0x131
#define CAN_RC_DATA_Frame_1 0x132

#define CAN_SYSTEM_RESET_CMD 0xAAA

#define CAN_PowerContol_ID 0x301

#define SLAM_YAWMOTOR_OUTPUT 0x140

#define SLAM_YAW_MOTOR_ID 0x205

#define CAN_AERIAL_DATA_1 0x501
#define CAN_AERIAL_DATA_2 0x502

extern uint16_t enemy_outpost_HP;
extern uint8_t JudgeRxDataValid;
extern int16_t PlanDot[2][2];
extern int16_t PosDot[2];
extern uint8_t InPosCount;
extern uint8_t InPosFlag;
extern uint8_t DataFromAerial[16];

void CAN_Device_Init(void);

void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data);
void Send_VTM_Data(CAN_HandleTypeDef *_hcan, uint8_t *vtm_data);
void Send_Robot_Info(CAN_HandleTypeDef *_hcan, int8_t ID, uint16_t heatLimit, uint16_t heat, uint16_t bulletSpeed);
void Send_Gimbal_Info(CAN_HandleTypeDef *_hcan, uint8_t aimassist_online);
void Send_Reset_Command(CAN_HandleTypeDef *_hcan);
void Send_Follow_Data_1(CAN_HandleTypeDef *_hcan, uint8_t *FOLLOW_Data_Buf);
void Send_Game_Status(CAN_HandleTypeDef *_hcan, uint8_t GameStatus);
void Send_Follow_Data_2(CAN_HandleTypeDef *_hcan, uint8_t *FOLLOW_Data_Buf);
void Send_Aerial_Data(CAN_HandleTypeDef *_hcan);

#endif
