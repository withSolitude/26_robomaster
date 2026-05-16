/**
* @file QuaternionEKF.h
 * @brief 四元数扩展卡尔曼滤波(EKF)姿态解算头文件
 */

#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H

#include "kalman_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

    typedef struct
    {
        uint8_t Initialized;
        KalmanFilter_t IMU_QuaternionEKF;
        uint8_t ConvergeFlag;
        uint8_t StableFlag;
        uint64_t ErrorCount;
        uint64_t UpdateCount;

        float q[4];
        float GyroBias[3];

        float Gyro[3];
        float Accel[3];

        float OrientationCosine[3];

        float accLPFcoef;
        float gyro_norm;
        float accl_norm;
        float AdaptiveGainScale;

        float Roll;
        float Pitch;
        float Yaw;

        float YawTotalAngle;

        float Q1;
        float Q2;
        float R;

        float dt;
        mat ChiSquare;
        float ChiSquare_Data[1];
        float ChiSquareTestThreshold;
        float lambda;

        int16_t YawRoundCount;

        float YawAngleLast;
    } QEKF_INS_t;

    extern QEKF_INS_t QEKF_INS;
    extern float chiSquare;
    extern float ChiSquareTestThreshold;

    void IMU_QuaternionEKF_Init(float process_noise1,
                                float process_noise2,
                                float measure_noise,
                                float lambda,
                                float lpf);

    void IMU_QuaternionEKF_Update(float gx,
                                  float gy,
                                  float gz,
                                  float ax,
                                  float ay,
                                  float az,
                                  float dt);

#ifdef __cplusplus
}
#endif

#endif
