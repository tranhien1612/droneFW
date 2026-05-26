#include "filter.h"

/* Kalman filter array */
KalmanFilter_Struct kfs[3] = {
    {0.02f, 0, 0, 0, 0.001f, 0.543f},
    {0.02f, 0, 0, 0, 0.001f, 0.543f},
    {0.02f, 0, 0, 0, 0.001f, 0.543f}
};

int16_t Filter_LowPass(int16_t newValue, int16_t preFilteredValue){
    return (int16_t)(ALPHA * newValue + (1 - ALPHA) * preFilteredValue);
}

double Filter_KalmanFilter(KalmanFilter_Struct *kf, double input){
    kf->Now_P = kf->LastP + kf->Q;
    kf->Kg = kf->Now_P / (kf->Now_P + kf->R);
    kf->out = kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1 - kf->Kg) * kf->Now_P;
    return kf->out;
}