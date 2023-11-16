#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "stdint.h"
#include "string.h"

#define RCFILTER_TASK_PERIOD 2

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define Key_W ((uint16_t)0x01 << 0)
#define Key_S ((uint16_t)0x01 << 1)
#define Key_A ((uint16_t)0x01 << 2)
#define Key_D ((uint16_t)0x01 << 3)
#define Key_SHIFT ((uint16_t)0x01 << 4)
#define Key_CTRL ((uint16_t)0x01 << 5)
#define Key_Q ((uint16_t)0x01 << 6)
#define Key_E ((uint16_t)0x01 << 7)
#define Key_R ((uint16_t)0x01 << 8)
#define Key_F ((uint16_t)0x01 << 9)
#define Key_G ((uint16_t)0x01 << 10)
#define Key_Z ((uint16_t)0x01 << 11)
#define Key_X ((uint16_t)0x01 << 12)
#define Key_C ((uint16_t)0x01 << 13)
#define Key_V ((uint16_t)0x01 << 14)
#define Key_B ((uint16_t)0x01 << 15)

#define RC_Get_Timeline USER_GetTimeline

typedef struct
{
    UART_HandleTypeDef *RC_USART;

    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;

    uint8_t switch_left; // 3 value
    uint8_t switch_right;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t press_left;
        uint8_t press_right;
    } mouse;

    uint16_t key_code;
    /**********************************************************************************
     * 15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
     *  V    C    X	   Z    G    F   R   E   Q  CTRL  SHIFT  D   A   S   W
     ************************************************************************************/

    uint8_t RC_state;

    uint32_t UpdateTick;

} RC_Type;

enum
{
    Switch_Up = 1,
    Switch_Middle = 3,
    Switch_Down = 2,
};

enum
{
    Disconnected = 0,
    Connected = 1,
};

extern RC_Type remote_control;

extern uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
extern uint8_t RC_Data_Buffer[16];
extern uint8_t RC_Update;

extern uint64_t Latest_Remote_Control_Pack_Time;

extern float ch1_buf;

void Remote_Control_Init(UART_HandleTypeDef *huart);
void Callback_RC_Handle(RC_Type *rc, uint8_t *buff);
uint8_t is_RC_Lost(void);
void Solve_RC_Lost(void);
void Solve_RC_Data_Error(void);
uint8_t RC_Data_is_Error(void);

#endif
