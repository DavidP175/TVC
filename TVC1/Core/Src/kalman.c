#include "kalman.h"
#include <math.h>
#include <string.h>

// Initialize the Kalman filter
void kalman_init(KalmanFilter* kf, float dt) {
    // Initialize state vector to zero
    memset(kf->x, 0, sizeof(kf->x));
    
    // Initialize covariance matrix
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            kf->P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Initialize process noise matrix
    // Higher values for angular rates (indices 3-5) as they're more uncertain
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            kf->Q[i][j] = (i == j) ? ((i < 3) ? 0.001f : 0.1f) : 0.0f;
        }
    }
    
    // Initialize measurement noise matrix
    // Lower values for gyro measurements as they're more reliable
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            kf->R[i][j] = (i == j) ? ((i < 3) ? 0.1f : 0.01f) : 0.0f;
        }
    }
    
    kf->dt = dt;
}

// Predict step
void kalman_predict(KalmanFilter* kf) {
    // State transition matrix
    float F[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        F[i][i] = 1.0f;
        if(i < 3) {  // For angles
            F[i][i+3] = kf->dt;  // Integrate rates to get angles
        }
    }
    
    // Predict state
    float x_pred[STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        x_pred[i] = 0;
        for(int j = 0; j < STATE_SIZE; j++) {
            x_pred[i] += F[i][j] * kf->x[j];
        }
    }
    memcpy(kf->x, x_pred, sizeof(x_pred));
    
    // Predict covariance
    float P_pred[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            P_pred[i][j] = 0;
            for(int k = 0; k < STATE_SIZE; k++) {
                P_pred[i][j] += F[i][k] * kf->P[k][j];
            }
        }
    }
    
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            kf->P[i][j] = 0;
            for(int k = 0; k < STATE_SIZE; k++) {
                kf->P[i][j] += P_pred[i][k] * F[j][k];
            }
            kf->P[i][j] += kf->Q[i][j];
        }
    }
}

// Update step with gyro measurements
void kalman_update_gyro(KalmanFilter* kf, float gx, float gy, float gz) {
    // Measurement vector
    float z[STATE_SIZE] = {0};
    z[3] = gx;  // Roll rate
    z[4] = gy;  // Pitch rate
    z[5] = gz;  // Yaw rate
    
    // Measurement matrix (we only measure rates directly)
    float H[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 3; i < STATE_SIZE; i++) {
        H[i][i] = 1.0f;
    }
    
    // Innovation
    float y[STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        y[i] = z[i];
        for(int j = 0; j < STATE_SIZE; j++) {
            y[i] -= H[i][j] * kf->x[j];
        }
    }
    
    // Innovation covariance
    float S[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            S[i][j] = kf->R[i][j];
            for(int k = 0; k < STATE_SIZE; k++) {
                S[i][j] += H[i][k] * kf->P[k][j];
            }
        }
    }
    
    // Kalman gain
    float K[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            K[i][j] = 0;
            for(int k = 0; k < STATE_SIZE; k++) {
                K[i][j] += kf->P[i][k] * H[j][k];
            }
            K[i][j] /= S[j][j];
        }
    }
    
    // Update state
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            kf->x[i] += K[i][j] * y[j];
        }
    }
    
    // Update covariance
    float I[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        I[i][i] = 1.0f;
    }
    
    float temp[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            temp[i][j] = I[i][j];
            for(int k = 0; k < STATE_SIZE; k++) {
                temp[i][j] -= K[i][k] * H[k][j];
            }
        }
    }
    
    float P_new[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++) {
        for(int j = 0; j < STATE_SIZE; j++) {
            P_new[i][j] = 0;
            for(int k = 0; k < STATE_SIZE; k++) {
                P_new[i][j] += temp[i][k] * kf->P[k][j];
            }
        }
    }
    memcpy(kf->P, P_new, sizeof(P_new));
}

// Get current state estimates
void kalman_get_state(KalmanFilter* kf, float* roll, float* pitch, float* yaw,
                     float* roll_rate, float* pitch_rate, float* yaw_rate) {
    *roll = kf->x[0];
    *pitch = kf->x[1];
    *yaw = kf->x[2];
    *roll_rate = kf->x[3];
    *pitch_rate = kf->x[4];
    *yaw_rate = kf->x[5];
} 