#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "user_lib.h"
#include "filter32.h"
#include "arm_math.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

#ifndef mat
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32
#endif // !mat

/******************************** FUZZY PID **********************************/
#define NB -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PB 3

typedef __packed struct
{
    float KpFuzzy;
    float KiFuzzy;
    float KdFuzzy;

    float (*FuzzyRuleKp)[7];
    float (*FuzzyRuleKi)[7];
    float (*FuzzyRuleKd)[7];

    float KpRatio;
    float KiRatio;
    float KdRatio;

    float eStep;
    float ecStep;

    float e;
    float ec;
    float eLast;

    uint32_t DWT_CNT;
    float dt;
} FuzzyRule_t;

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep);
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref);

/******************************* PID CONTROL *********************************/
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        // 0000 0000
    Integral_Limit = 0x01,              // 0000 0001
    Derivative_On_Measurement = 0x02,   // 0000 0010
    Trapezoid_Intergral = 0x04,         // 0000 0100
    Proportional_On_Measurement = 0x08, // 0000 1000
    OutputFilter = 0x10,                // 0001 0000
    ChangingIntegrationRate = 0x20,     // 0010 0000
    DerivativeFilter = 0x40,            // 0100 0000
    ErrorHandle = 0x80,                 // 1000 0000
} PID_Improvement_e;

typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef __packed struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef __packed struct pid_t
{
    float Ref;
    float Kp;
    float Ki;
    float Kd;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float CoefA;         // For Changing Integral
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;

    uint16_t OLS_Order;
    Ordinary_Least_Squares_t OLS;

    uint32_t DWT_CNT;
    float dt;

    FuzzyRule_t *FuzzyRule;

    uint8_t Improve;

    PID_ErrorHandler_t ERRORHandler;

    void (*User_Func1_f)(struct pid_t *pid);
    void (*User_Func2_f)(struct pid_t *pid);
} PID_t;

void PID_Init(
    PID_t *pid,
    float max_out,
    float intergral_limit,
    float deadband,

    float kp,
    float ki,
    float kd,

    float A,
    float B,

    float output_lpf_rc,
    float derivative_lpf_rc,

    uint16_t ols_order,

    uint8_t improve);
float PID_Calculate(PID_t *pid, float measure, float ref);

/*************************** FEEDFORWARD CONTROL *****************************/
typedef __packed struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Ref;
    float Last_Ref;

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Ref_dot;
    float Ref_ddot;
    float Last_Ref_dot;

    uint16_t Ref_dot_OLS_Order;
    Ordinary_Least_Squares_t Ref_dot_OLS;
    uint16_t Ref_ddot_OLS_Order;
    Ordinary_Least_Squares_t Ref_ddot_OLS;

    float Output;
    float MaxOut;

} Feedforward_t;

void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order);

float Feedforward_Calculate(Feedforward_t *ffc, float ref);

/************************* LINEAR DISTURBANCE OBSERVER *************************/
typedef __packed struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Measure;
    float Last_Measure;

    float u; // system input

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Measure_dot;
    float Measure_ddot;
    float Last_Measure_dot;

    uint16_t Measure_dot_OLS_Order;
    Ordinary_Least_Squares_t Measure_dot_OLS;
    uint16_t Measure_ddot_OLS_Order;
    Ordinary_Least_Squares_t Measure_ddot_OLS;

    float Disturbance;
    float Output;
    float Last_Disturbance;
    float Max_Disturbance;
} LDOB_t;

void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order);

float LDOB_Calculate(LDOB_t *ldob, float measure, float u);

/*************************** Tracking Differentiator ***************************/
typedef __packed struct
{
    float Input;

    float h0;
    float r;

    float x;
    float dx;
    float ddx;

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);

/************** Second Order System based Tracking Differentiator **************/
typedef __packed struct
{
    float Input;

    float Omega;

    float x;
    float dx;
    float ddx;
    float dddx;

    float last_dx;
    float last_ddx;
    float last_dddx;

    uint32_t DWT_CNT;
    float dt;
} ThirdOrderTD_t;

void ThirdOrder_TD_Init(ThirdOrderTD_t *tf, float omega);
float ThirdOrder_TD_Calculate(ThirdOrderTD_t *tf, float input);

/**************************** SYSTEM IDENTIFICATION ****************************/
// �ݶ��½����� ������ ��д������С���˷�
//  typedef struct
//  {
//      uint8_t System_Order;
//      uint8_t Dynamic_Friction; // Whether or not to consider dynamic friction

//     uint16_t Sample_Size;

//     float a; // Gradient descent coef

//     float c[4]; // G(s) = 1/(c2s^2 + c1s + c0)  u = c3*sign(x) + s2*ddx + c1*dx + c0*x
//     float *u;   // system input
//     float *x;
//     float *x_dot;
//     float *x_ddot;

//     float Gradient[4];
//     float Cross_Product;
//     float Cost;

//     float DeadBand;

//     uint32_t DWT_CNT;
//     float dt;

//     uint8_t Iterate_Accomplish;

//     float LPF_RC; //Q(s) = 1/(s/c + 1)

//     First_Order_Filter_t u_lpf;
//     First_Order_Filter_t x_lpf;
//     First_Order_Filter_t x_dot_lpf;
//     First_Order_Filter_t x_ddot_lpf;

//     uint32_t count;

// } SI_t;
typedef __packed struct
{
    uint8_t System_Order;
    uint8_t Dynamic_Friction; // Whether or not to consider dynamic friction

    uint16_t Sample_Size;

    float a; // Gradient descent coef

    float c[4]; // G(s) = 1/(c2s^2 + c1s + c0)  u = c3*sign(x) + s2*ddx + c1*dx + c0*x
    float u;    // system input
    float x;
    float x_dot;
    float x_ddot;

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    First_Order_Filter_t u_lpf;
    First_Order_Filter_t x_lpf;
    First_Order_Filter_t x_dot_lpf;
    First_Order_Filter_t x_ddot_lpf;

    uint32_t count;

} SI_t;

void SI_Init(
    SI_t *si,
    float *c,
    uint8_t order,
    uint8_t fric,
    float a,
    uint8_t *param_to_idf,
    uint16_t sample_size,
    float lpf_rc);

void SI_Sample_Update(SI_t *si, float x, float u);
void SI_Iterate(SI_t *si);
void SI_Init(
    SI_t *si,
    float *c,
    uint8_t order,
    uint8_t fric,
    float a,
    uint8_t *param_to_idf,
    uint16_t sample_size,
    float lpf_rc);

void SI_Sample_Update(SI_t *si, float x, float u);
void SI_Iterate(SI_t *si);

#endif
