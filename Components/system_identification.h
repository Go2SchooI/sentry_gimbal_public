/**
 ******************************************************************************
 * @file    system_identification.c
 * @author  Wang Hongxi
 * @version V1.0.1
 * @date    2021/10/28
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __SYSTEM_IDENTIFICATION_H
#define __SYSTEM_IDENTIFICATION_H

#include "kalman_filter.h"

typedef struct
{
    float xhat;
    float c0;
    float c1;

    float u;
    float x;

    float lambda;

    KalmanFilter_t SI_EKF;
} FirstOrderSI_t;

void FirstOrderSI_Init(FirstOrderSI_t *sysID_t, float c0, float c1, float Q0, float Q1, float Q2, float R, float lambda);
void FirstOrderSI_Update(FirstOrderSI_t *sysID_t, float u, float x, float dt);
void FirstOrderSI_EKF_Tuning(FirstOrderSI_t *sysID_t, float Q0, float Q1, float Q2, float R, float lambda);

#endif
