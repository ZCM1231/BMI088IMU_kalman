#ifndef __KALMAN_CONFIG_H__
#define __KALMAN_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

// ============ EKF 参数配置 (使用宏定义避免被链接器优化) ============

// 过程噪声
#define KF_PROCESS_NOISE_Q      1e-5f   // 四元数过程噪声
#define KF_PROCESS_NOISE_B      1e-8f   // 陀螺仪偏置过程噪声

// 观测噪声
#define KF_MEASUREMENT_NOISE_A  0.1f    // 加速度计观测噪声

// 时间步长
#define KF_DT_NOMINAL           (1.0f / 1000.0f)  // 1ms

// 自适应观测噪声参数
#define KF_ADAPTIVE_R_THRESHOLD  0.15f  // 水平加速度阈值 (m/s^2)
#define KF_ADAPTIVE_R_MAX_SCALE  100.0f // R的最大缩放倍数
#define KF_ADAPTIVE_R_EXPONENT   1.0f   // 缩放幂指数 (1.0=线性)

// 偏置限幅
#define KF_MAX_BIAS             0.0f    // 最大陀螺仪偏置 (rad/s)

// 静止检测参数
#define KF_STATIC_GYRO_THRESH   0.1f    // 陀螺仪静止阈值 (rad/s)
#define KF_STATIC_ACCEL_DEV     1.0f    // 加速度偏离重力阈值 (m/s²)

#ifdef __cplusplus
}
#endif

#endif // __KALMAN_CONFIG_H__
