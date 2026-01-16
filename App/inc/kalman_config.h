#ifndef __KALMAN_CONFIG_H__
#define __KALMAN_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// 算法选择开关
// ==============================================================================

/**
 * 四元数积分方法选择
 * 0 = 一阶欧拉法（快速，误差 O(dt²)，计算时间约60μs）
 * 1 = RK4四阶龙格-库塔（高精度，误差 O(dt⁵)，计算时间约100μs）
 * 
 * 建议：高动态应用（快速旋转/大加速度）使用 RK4
 */
#define USE_RK4_INTEGRATION  1

// ==============================================================================
// EKF参数（外部声明）
// ==============================================================================

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

