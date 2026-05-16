#ifndef C_BOARD_IMU_H
#define C_BOARD_IMU_H

#include <cstdint>
#include "BMI088_Driver.h"
#include "QuaternionEKF.h"

#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2

class Class_Board_IMU
{
public:
    void Init(SPI_HandleTypeDef *hspi);
    void Update(void);

    /**
     * @brief IMU采样任务，1ms周期调用，读取传感器原始数据
     */
    void RTOS_IMU_1ms_Sampling_Task(void);

    /**
     * @brief 设置EKF计算时间步长
     * @param dt_val 时间步长，单位秒
     */
    inline void Set_EKF_Dt(float dt_val) { dt = dt_val; }

    /**
     * @brief EKF更新回调函数，1ms周期调用
     */
    void RTOS_IMU_1ms_EKF_Callback(void);

    inline float Get_Accel_X(void) const { return INS.Accel[IMU_X]; }
    inline float Get_Accel_Y(void) const { return INS.Accel[IMU_Y]; }
    inline float Get_Accel_Z(void) const { return INS.Accel[IMU_Z]; }
    inline float Get_Gyro_X(void) const { return INS.Gyro[IMU_X]; }
    inline float Get_Gyro_Y(void) const { return INS.Gyro[IMU_Y]; }
    inline float Get_Gyro_Z(void) const { return INS.Gyro[IMU_Z]; }
    inline float Get_Roll(void) const { return INS.Roll; }
    inline float Get_Pitch(void) const { return INS.Pitch; }
    inline float Get_Yaw(void) const { return INS.Yaw; }
    inline float Get_YawTotalAngle(void) const { return INS.YawTotalAngle; }
    inline float Get_dt(void) const { return dt; }
    inline const float* Get_q(void) const { return INS.q; }
    inline const float* Get_xn(void) const { return INS.xn; }
    inline const float* Get_yn(void) const { return INS.yn; }
    inline const float* Get_zn(void) const { return INS.zn; }
    inline const float* Get_MotionAccel_b(void) const { return INS.MotionAccel_b; }
    inline const float* Get_MotionAccel_n(void) const { return INS.MotionAccel_n; }
    inline float Get_GyroOffset_X(void) const { return bmi088_data.GyroOffset[IMU_X]; }
    inline float Get_GyroOffset_Y(void) const { return bmi088_data.GyroOffset[IMU_Y]; }
    inline float Get_GyroOffset_Z(void) const { return bmi088_data.GyroOffset[IMU_Z]; }
    inline bool Get_IsStationary(void) const { return is_stationary; }
    inline float Get_GyroNorm(void) const { return gyro_norm; }
    inline float Get_AccelNorm(void) const { return accel_norm; }
    inline float Get_AccelError(void) const { return accel_error; }
    inline void Set_dt(float dt_val) { dt = dt_val; }


    inline float Get_Q1(void) const { return INS.q[0]; }
    inline float Get_Q2(void) const { return INS.q[1]; }
    inline float Get_Q3(void) const { return INS.q[2]; }
    inline float Get_Q4(void) const { return INS.q[3]; }


private:
    void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
    void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
    void Update_Gyro_Bias_Estimator(void);
    void Update_Stationary_Detection(void);
    static float Norm3(const float *v);
    static void Math_Constrain(float *val, float min, float max);

private:
    SPI_HandleTypeDef *hspi;
    IMU_Data_t bmi088_data;
    float dt;
    uint8_t offset_flag;

    // 静止检测参数（使用误差阈值）
    static constexpr float gravity_ref = 9.807f;                           // 参考重力加速度 m/s²
    static constexpr float stationary_gyro_enter_threshold = 0.045f;       // 进入静止角速度阈值 rad/s
    static constexpr float stationary_gyro_exit_threshold = 0.05f;      // 退出静止角速度阈值 rad/s
    static constexpr float stationary_accel_enter_threshold = 0.2f;     // 进入静止加速度误差阈值 m/s²
    static constexpr float stationary_accel_exit_threshold = 0.2f;     // 退出静止加速度误差阈值 m/s²
    static constexpr uint16_t stationary_enter_confirm_count = 200U;     // 进入静止确认次数
    static constexpr uint16_t stationary_exit_confirm_count = 1U;       // 退出静止确认次数

    // 静态检测状态
    uint16_t stationary_enter_counter = 0U;
    uint16_t stationary_exit_counter = 0U;
    bool is_stationary = false;

    // 调试变量（用于观测静态检测条件）
    float gyro_norm = 0.0f;
    float accel_norm = 0.0f;
    float accel_error = 0.0f;

    // 陀螺仪零偏估计器参数
    static constexpr float gyro_bias_update_norm_threshold = 0.07f;      // 零偏模阈值（降低）
    static constexpr float gyro_bias_update_axis_threshold = 0.06f;     // 单轴零偏阈值（降低）
    static constexpr float gyro_bias_stationary_alpha = 0.001f;        // 低通滤波系数（降低）
    static constexpr float gyro_bias_stationary_step_limit = 0.0001f;  // 单步更新限制（降低）
    static constexpr uint16_t gyro_bias_init_count_target = 100U;      // 初始累加次数
    static constexpr float gyro_bias_max = 0.004f;                     // 零偏最大值 rad/s

    // 陀螺仪零偏估计器状态
    uint16_t gyro_bias_init_count = 0U;
    float gyro_bias_acc_x = 0.0f;
    float gyro_bias_acc_y = 0.0f;
    float gyro_bias_acc_z = 0.0f;

    struct {
        float q[4];
        float Gyro[3];
        float Accel[3];
        float MotionAccel_b[3];
        float MotionAccel_n[3];
        float AccelLPF;
        float xn[3];
        float yn[3];
        float zn[3];
        float Roll;
        float Pitch;
        float Yaw;
        float YawTotalAngle;
    } INS;
};

extern Class_Board_IMU BMI088;

#endif
