#ifndef __DETECT_TASK_H
#define __DETECT_TASK_H

#include <stdint.h>
#include "includes.h"

#define DETECT_TASK_PERIOD 2

// #ifdef _CMSIS_OS_H
// #define USER_GetTick xTaskGetTickCount
// #else
// #define USER_GetTick HAL_GetTick
// #endif
#define USER_GetTick HAL_GetTick

#ifdef _CMSIS_OS_H
#define USER_Delay_ms vTaskDelay
#else
#define USER_Delay_ms HAL_Delay
#endif

enum errorList
{
    GIMBAL_YAW_MOTOR_TOE,
    GIMBAL_PITCH_MOTOR_TOE,
    TRIGGER_MOTOR1_TOE,
    TRIGGER_MOTOR2_TOE,
    SWITCH_MOTOR_TOE,
    RC_TOE,
    AUTOAIM_DEVICE,
    JUDGE_TOE,
    VTM_TOE,
    DETECT_LIST_LENGHT,
    SLAM_YAW_MOTOR_TOE,
};

typedef struct
{
    uint32_t New_Time;
    uint32_t Last_Time;
    uint32_t Lost_Time;
    uint32_t Work_Time;
    uint16_t Offline_Time : 12;
    uint16_t Online_Time : 12;
    uint8_t Enable : 1;
    uint8_t Priority : 4;
    uint8_t Error_Exist : 1;
    uint8_t is_Lost : 1;
    uint8_t is_Data_Error : 1;

    float frequency;
    uint8_t (*f_is_Data_Error)(void);
    void (*f_Solve_Lost)(void);
    void (*f_Solve_Data_Error)(void);
} Detect_t;

typedef struct
{
    float Voltage;
    float Temperature;
} BoardState_t;

extern Detect_t Detect_List[DETECT_LIST_LENGHT + 1];
extern uint32_t resetCount;
void Detect_Task_Init(void);

void Detect_Task(void);

uint8_t is_TOE_Error(uint8_t toe);

void Detect_Hook(uint8_t toe);

void Detect_Hook(uint8_t toe);

#endif
