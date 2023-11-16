#ifndef __INCLUDES_H
#define __INCLUDES_H

#define M3508 0
#define RMD9025_V1 1
#define RMD9025_V2 2
#define HT03 3

#define USER_GetTimeline HAL_GetTick

// CubeMX
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// std C
#include <stdio.h>
#include <stdint.h>

// DSP lib
#include "arm_math.h"

// user lib
#include "user_lib.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

// bsp
#include "bsp_CAN.h"
#include "bsp_dwt.h"
#include "bsp_PWM.h"
#include "bsp_usart_idle.h"
// #include "bsp_adc.h"

// application
#include "motor.h"
#include "remote_control.h"
#include "SerialDebug.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "ins_task.h"
#include "aimassist_task.h"
#include "judgement_info.h"

#endif
