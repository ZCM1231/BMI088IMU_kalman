#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <stdint.h>
#include "kalman_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// 数据结构定义
// ==============================================================================

// 3D向量
typedef struct {
  float x, y, z;
} Vec3_t;

// 四元数 (w, x, y, z)
typedef struct {
  float w, x, y, z;
} Quat_t;

// 7x1 状态向量 [q(4), b(3)]
typedef struct {
  Quat_t q;  // 姿态四元数
  Vec3_t b;  // 陀螺仪偏置
} State7_t;

// 3x3 矩阵
typedef struct {
  float m[3][3];
} Mat3x3_t;

// 7x7 协方差矩阵
typedef struct {
  float m[7][7];
} Mat7x7_t;

// EKF 主结构体
typedef struct {
  State7_t x;           // 状态向量
  Mat7x7_t P;           // 协方差矩阵
  Mat7x7_t Q;           // 过程噪声矩阵
  Mat3x3_t R;           // 观测噪声矩阵（自适应调整）
  Mat3x3_t R_base;      // 基础观测噪声矩阵（不变）
  
  // 运行时参数（从Config初始化）
  float process_noise_q; // 陀螺仪噪声标量
  float g;              // 重力常数
  float max_bias;       // 偏置最大值
  
  // 自适应R参数
  float adaptive_r_threshold;  // 水平加速度阈值
  float adaptive_r_max_scale;  // R的最大缩放倍数
  float adaptive_r_exponent;   // 缩放幂指数
  float r_scale;               // 当前R的缩放因子（用于调试输出）
  float acc_horizontal;        // 当前水平加速度（用于调试输出）
  
  // 时间步长
  float dt;  // 当前时间步长（秒），用于调试
  
  // 调试信息：纯加速度计计算的欧拉角（度）
  float accel_only_euler[3];  // [roll, pitch, yaw=0]
  
  // 调试信息：卡尔曼增益
  float debug_K[7][3];
  
  // 新息（测量残差，用于调试）
  float y[3];
  
} AttitudeEKF_t;

// ==============================================================================
// 函数声明
// ==============================================================================

/**
 * @brief  初始化EKF
 * @param  ekf: EKF结构体指针
 * @param  initial_accel: 初始加速度（用于计算初始姿态）
 * @retval None
 */
void EKF_Init(AttitudeEKF_t *ekf, const Vec3_t *initial_accel);

/**
 * @brief  EKF预测步骤
 * @param  ekf: EKF结构体指针
 * @param  gyro_meas: 陀螺仪测量值 (rad/s)
 * @param  dt: 时间步长 (秒)
 * @retval None
 */
void EKF_Predict(AttitudeEKF_t *ekf, const Vec3_t *gyro_meas, float dt);

/**
 * @brief  EKF更新步骤（使用自适应观测噪声）
 * @param  ekf: EKF结构体指针
 * @param  acc_meas: 加速度计测量值 (m/s²)
 * @retval None
 */
void EKF_Update(AttitudeEKF_t *ekf, const Vec3_t *acc_meas);

/**
 * @brief  四元数转欧拉角
 * @param  q: 四元数
 * @param  roll: 横滚角输出 (度)
 * @param  pitch: 俯仰角输出 (度)
 * @param  yaw: 偏航角输出 (度)
 * @retval None
 */
void Quat_ToEuler(const Quat_t *q, float *roll, float *pitch, float *yaw);

/**
 * @brief  计算纯加速度计欧拉角（用于调试对比）
 * @param  ekf: EKF结构体指针
 * @param  acc_meas: 加速度测量值 (m/s²)
 * @retval None
 * @note   结果存储在 ekf->accel_only_euler[3] 中（度）
 */
void EKF_CalculateAccelOnlyEuler(AttitudeEKF_t *ekf, const Vec3_t *acc_meas);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_FILTER_H__ */

