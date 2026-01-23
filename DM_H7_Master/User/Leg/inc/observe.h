#ifndef __OBSERVE_H__
#define __OBSERVE_H__

#include "main.h"

extern KalmanFilter_t vaEstimateKF;

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF, float dt);
float xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float vel, float acc);

#endif // !__OBSERVE_H__