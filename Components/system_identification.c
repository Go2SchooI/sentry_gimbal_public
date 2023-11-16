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
#include "system_identification.h"

void FirstOrderSI_Init(FirstOrderSI_t *sysID_t, float c0, float c1, float Q0, float Q1, float Q2, float R, float lambda)
{
    sysID_t->c0 = c0;
    sysID_t->c1 = c1;

    Kalman_Filter_Init(&sysID_t->SI_EKF, 3, 0, 1);

    sysID_t->SI_EKF.P_data[0] = 10000;
    sysID_t->SI_EKF.P_data[4] = 10000000;
    sysID_t->SI_EKF.P_data[8] = 10000000;

    sysID_t->SI_EKF.SkipEq1 = 1;

    sysID_t->SI_EKF.F_data[4] = 1;
    sysID_t->SI_EKF.F_data[8] = 1;

    sysID_t->SI_EKF.Q_data[0] = Q0;
    sysID_t->SI_EKF.Q_data[4] = Q1;
    sysID_t->SI_EKF.Q_data[8] = Q2;

    sysID_t->SI_EKF.H_data[0] = 1;

    sysID_t->SI_EKF.R_data[0] = R;

    sysID_t->lambda = lambda;

    sysID_t->SI_EKF.xhat_data[1] = sysID_t->c0;
    sysID_t->SI_EKF.xhat_data[2] = sysID_t->c1;
}

void FirstOrderSI_Update(FirstOrderSI_t *sysID_t, float u, float x, float dt)
{
    sysID_t->u = u;
    sysID_t->x = x;

    sysID_t->SI_EKF.MeasuredVector[0] = x;

    sysID_t->SI_EKF.xhatminus_data[0] = (1 + sysID_t->SI_EKF.xhat_data[1] * dt) * sysID_t->SI_EKF.xhat_data[0] +
                                        sysID_t->SI_EKF.xhat_data[2] * u * dt;
    sysID_t->SI_EKF.xhatminus_data[1] = sysID_t->SI_EKF.xhat_data[1];
    sysID_t->SI_EKF.xhatminus_data[2] = sysID_t->SI_EKF.xhat_data[2];

    sysID_t->SI_EKF.F_data[0] = 1 + sysID_t->SI_EKF.xhat_data[1] * dt;
    sysID_t->SI_EKF.F_data[1] = sysID_t->SI_EKF.xhat_data[0] * dt;
    sysID_t->SI_EKF.F_data[2] = u * dt;

    Kalman_Filter_Update(&sysID_t->SI_EKF);

    for (uint8_t i = 0; i < 9; i++)
        sysID_t->SI_EKF.P_data[i] /= sysID_t->lambda;

    sysID_t->xhat = sysID_t->SI_EKF.FilteredValue[0];
    sysID_t->c0 = -sysID_t->SI_EKF.FilteredValue[1] / sysID_t->SI_EKF.FilteredValue[2];
    sysID_t->c1 = 1 / sysID_t->SI_EKF.FilteredValue[2];
}

void FirstOrderSI_EKF_Tuning(FirstOrderSI_t *sysID_t, float Q0, float Q1, float Q2, float R, float lambda)
{
    sysID_t->SI_EKF.Q_data[0] = Q0;
    sysID_t->SI_EKF.Q_data[4] = Q1;
    sysID_t->SI_EKF.Q_data[8] = Q2;

    sysID_t->SI_EKF.R_data[0] = R;

    sysID_t->lambda = lambda;
}
