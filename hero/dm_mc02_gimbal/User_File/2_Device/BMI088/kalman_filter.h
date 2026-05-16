/**
 * @file kalman_filter.h
 * @brief 卡尔曼滤波框架头文件
 */

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

typedef struct kf_t
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;
    float *MeasurementDegree;
    float *MatR_DiagonalElements;
    float *StateMinVariance;
    uint8_t *temp;

    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    mat xhat;
    mat xhatminus;
    mat u;
    mat z;
    mat P;
    mat Pminus;
    mat F, FT;
    mat B;
    mat H, HT;
    mat Q;
    mat R;
    mat K;
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    float *xhat_data;
    float *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data;
    float *Pminus_data;
    float *F_data;
    float *FT_data;
    float *B_data;
    float *H_data;
    float *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data;
    float *temp_matrix_data;
    float *temp_matrix_data1;
    float *temp_vector_data;
    float *temp_vector_data1;
} KalmanFilter_t;

extern uint16_t sizeof_float;
extern uint16_t sizeof_double;

void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
void Kalman_Filter_Measure(KalmanFilter_t *kf);
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_SetK(KalmanFilter_t *kf);
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);
void Kalman_Filter_P_Update(KalmanFilter_t *kf);
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#ifdef __cplusplus
}
#endif

#endif