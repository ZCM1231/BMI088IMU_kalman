#ifndef __KALMAN_CONFIG_H__
#define __KALMAN_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

// EKF参数（外部声明）
extern const float KF_PROCESS_NOISE_Q;
extern const float KF_PROCESS_NOISE_B;
extern const float KF_MEASUREMENT_NOISE_A;

// 滤波器参数（外部声明）
extern const float KF_DT_NOMINAL;

// 自适应观测噪声参数（外部声明）
extern const float KF_ADAPTIVE_R_THRESHOLD;
extern const float KF_ADAPTIVE_R_MAX_SCALE;
extern const float KF_ADAPTIVE_R_EXPONENT;

// 偏置限幅（外部声明）
extern const float KF_MAX_BIAS;

#ifdef __cplusplus
}
#endif

#endif // __KALMAN_CONFIG_H__

