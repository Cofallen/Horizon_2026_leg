#include "kalman_filter.h"
#include "observe.h"
#include "All_Init.h"

KalmanFilter_t vaEstimateKF;

float KF_F[4] = {1.0f, 0.001f, 0.0f, 1.0f};            // dt应取实际值
float KF_P[4] = {1.0f, 0.0f, 0.0f, 1.0f};
float KF_Q[4] = {0.1f, 0.0f, 0.0f, 1.0f};
float KF_R[4] = {10000.0f, 0.0f, 0.0f, 200000.0f};      // 噪声
float KF_K[4] = {1.0f, 0.0f, 0.0f, 1.0f};
float KF_H[4] = {1.0f, 0.0f, 0.0f, 1.0f};

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF, float dt)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);
    memcpy(EstimateKF->F_data, KF_F, sizeof(KF_F));
    memcpy(EstimateKF->P_data, KF_P, sizeof(KF_P));
    memcpy(EstimateKF->Q_data, KF_Q, sizeof(KF_Q));
    memcpy(EstimateKF->R_data, KF_R, sizeof(KF_R));
    memcpy(EstimateKF->H_data, KF_H, sizeof(KF_H));
}

// 反馈滤波后速度
float xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float vel, float acc)
{
    EstimateKF->MeasuredVector[0] = vel;
    EstimateKF->MeasuredVector[1] = acc;

    Kalman_Filter_Update(EstimateKF);
    return EstimateKF->FilteredValue[0];
}