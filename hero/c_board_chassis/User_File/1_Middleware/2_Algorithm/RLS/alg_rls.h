//
// Created by ChatGPT on 2026/02/16.
//
// 说明：
// - 这是一个非常轻量的 RLS(递推最小二乘)模块，用于在线估计线性模型参数。
// - 目前提供 2 维参数版本：y = theta0 * phi0 + theta1 * phi1
// - 典型用法：
//   1) Init(lambda, p0, theta0_init, theta1_init)
//   2) 每次有观测时 Update(y, phi0, phi1)
//   3) Get_Theta0()/Get_Theta1() 读取参数

#ifndef TEST_ROBOWAKER_ALG_RLS_H
#define TEST_ROBOWAKER_ALG_RLS_H

#include <stdint.h>

class Class_RLS
{
public:
    void Init(float __Lambda, float __P0, float __Theta0_Init, float __Theta1_Init);

    void Reset(float __P0, float __Theta0_Init, float __Theta1_Init);

    // y: 测量值
    // phi0/phi1: 回归向量
    void Update(float y, float phi0, float phi1);

    inline float Get_Theta0() { return Theta[0]; }
    inline float Get_Theta1() { return Theta[1]; }

private:
    // 遗忘因子 (0,1]，越接近 1 越稳定
    float Lambda = 0.999f;

    // 参数
    float Theta[2] = {0.0f, 0.0f};

    // 协方差矩阵 P (2x2)
    float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
};

#endif // TEST_ROBOWAKER_ALG_RLS_H
