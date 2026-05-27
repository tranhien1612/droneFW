#ifndef ___FILTER_H
#define ___FILTER_H

#include <stdint.h>

// First-order low-pass filter coefficients. The smaller the value, the stronger the low-pass filtering effect.
#define ALPHA 0.15f /* First-order low-pass filter exponential weighting coefficients */

/* Kalman filter structure */
typedef struct{
    float LastP; // State error (covariance) at the previous time step
    float Now_P; // State error (covariance) at the current time step
    float out; // Filter output value (optimal estimated state)
    float Kg; // Kalman gain, used to determine the weight between the predicted and measured values
    float Q; // Variance of process noise, reflecting the uncertainty of the system model
    float R; // Variance of measurement noise, reflecting the uncertainty of the measurement process
} KalmanFilter_Struct;

extern KalmanFilter_Struct kfs[3];

/**
* @brief First-order low-pass filter
* @param newValue The value to be filtered
* @param preFilteredValue The value after the previous filtering
* @return The filtered value
*/
int16_t Filter_LowPass(int16_t newValue, int16_t preFilteredValue);

/**
* @brief Kalman filter
* @param kf Pointer to the Kalman filter structure
* @param input Input value
* @return Filtered value
*/
double Filter_KalmanFilter(KalmanFilter_Struct *kf, double input);

#endif