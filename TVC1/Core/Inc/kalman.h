#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

// State vector size
#define STATE_SIZE 6  // [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]

typedef struct {
    float x[STATE_SIZE];     // State vector
    float P[STATE_SIZE][STATE_SIZE];  // Covariance matrix
    float Q[STATE_SIZE][STATE_SIZE];  // Process noise matrix
    float R[STATE_SIZE][STATE_SIZE];  // Measurement noise matrix
    float dt;                // Time step
} KalmanFilter;

// Initialize the Kalman filter
void kalman_init(KalmanFilter* kf, float dt);

// Predict step
void kalman_predict(KalmanFilter* kf);

// Update step with gyro measurements
void kalman_update_gyro(KalmanFilter* kf, float gx, float gy, float gz);

// Get current state estimates
void kalman_get_state(KalmanFilter* kf, float* roll, float* pitch, float* yaw,
                     float* roll_rate, float* pitch_rate, float* yaw_rate);

#endif // KALMAN_H 