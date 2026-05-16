//
// Created by ChatGPT on 2026/02/16.
//

#include "alg_rls.h"

void Class_RLS::Init(float __Lambda, float __P0, float __Theta0_Init, float __Theta1_Init)
{
    Lambda = __Lambda;
    Reset(__P0, __Theta0_Init, __Theta1_Init);
}

void Class_RLS::Reset(float __P0, float __Theta0_Init, float __Theta1_Init)
{
    Theta[0] = __Theta0_Init;
    Theta[1] = __Theta1_Init;

    // P = p0 * I
    P[0][0] = __P0;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = __P0;
}

void Class_RLS::Update(float y, float phi0, float phi1)
{
    // phi = [phi0, phi1]^T
    // K = P*phi / (lambda + phi^T*P*phi)
    float pphi0 = P[0][0] * phi0 + P[0][1] * phi1;
    float pphi1 = P[1][0] * phi0 + P[1][1] * phi1;

    float den = Lambda + (phi0 * pphi0 + phi1 * pphi1);
    if (den < 1e-9f)
    {
        return;
    }

    float k0 = pphi0 / den;
    float k1 = pphi1 / den;

    // e = y - phi^T*theta
    float y_hat = phi0 * Theta[0] + phi1 * Theta[1];
    float e = y - y_hat;

    // theta = theta + K*e
    Theta[0] += k0 * e;
    Theta[1] += k1 * e;

    // P = (P - K*phi^T*P) / lambda
    // 先算 (K*phi^T) 这个 2x2
    float kphi00 = k0 * phi0;
    float kphi01 = k0 * phi1;
    float kphi10 = k1 * phi0;
    float kphi11 = k1 * phi1;

    float p00 = P[0][0] - (kphi00 * P[0][0] + kphi01 * P[1][0]);
    float p01 = P[0][1] - (kphi00 * P[0][1] + kphi01 * P[1][1]);
    float p10 = P[1][0] - (kphi10 * P[0][0] + kphi11 * P[1][0]);
    float p11 = P[1][1] - (kphi10 * P[0][1] + kphi11 * P[1][1]);

    float inv_lambda = 1.0f / Lambda;
    P[0][0] = p00 * inv_lambda;
    P[0][1] = p01 * inv_lambda;
    P[1][0] = p10 * inv_lambda;
    P[1][1] = p11 * inv_lambda;
}
