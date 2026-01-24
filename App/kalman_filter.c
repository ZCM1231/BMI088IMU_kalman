/**
 ******************************************************************************
 * @file    kalman_filter.c
 * @brief   姿态估计扩展卡尔曼滤波器实现
 * @note    移植自 imu_module 项目，适配 BMI088 传感器
 * @date    2026-01-15
 ******************************************************************************
 */

#include "kalman_filter.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ==============================================================================
// 辅助函数 - 基础数学运算
// ==============================================================================

static inline float Vec3_Norm(const Vec3_t *v) {
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

static void Vec3_Normalize(Vec3_t *v) {
    float norm = Vec3_Norm(v);
    if (norm > 1e-6f) {
        v->x /= norm;
        v->y /= norm;
        v->z /= norm;
    }
}

static void Quat_Normalize(Quat_t *q) {
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 1e-6f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

static void Mat3_Identity(Mat3x3_t *m) {
    memset(m, 0, sizeof(Mat3x3_t));
    m->m[0][0] = m->m[1][1] = m->m[2][2] = 1.0f;
}

static void Mat3_Scale(Mat3x3_t *result, const Mat3x3_t *m, float s) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            result->m[i][j] = m->m[i][j] * s;
}

static void Mat7_Identity(Mat7x7_t *m) {
    memset(m, 0, sizeof(Mat7x7_t));
    for (int i = 0; i < 7; i++)
        m->m[i][i] = 1.0f;
}

static void Mat7_Zero(Mat7x7_t *m) {
    memset(m, 0, sizeof(Mat7x7_t));
}

// ==============================================================================
// EKF初始化
// ==============================================================================

void EKF_Init(AttitudeEKF_t *ekf, const Vec3_t *initial_accel) {
    
    // 初始化参数 (从Config读取)
    ekf->process_noise_q = KF_PROCESS_NOISE_Q;
    ekf->g = 9.81f;
    ekf->max_bias = KF_MAX_BIAS;
    
    // 自适应R参数
    ekf->adaptive_r_threshold = KF_ADAPTIVE_R_THRESHOLD;
    ekf->adaptive_r_max_scale = KF_ADAPTIVE_R_MAX_SCALE;
    ekf->adaptive_r_exponent = KF_ADAPTIVE_R_EXPONENT;
    ekf->r_scale = 1.0f;
    ekf->acc_horizontal = 0.0f;
    
    // 时间步长
    ekf->dt = KF_DT_NOMINAL;
    
    // 初始化调试信息
    ekf->accel_only_euler[0] = 0.0f;
    ekf->accel_only_euler[1] = 0.0f;
    ekf->accel_only_euler[2] = 0.0f;
    memset(ekf->debug_K, 0, sizeof(ekf->debug_K));
    ekf->y[0] = 0.0f;
    ekf->y[1] = 0.0f;
    ekf->y[2] = 0.0f;
    
    // 从加速度计初始化姿态
    float ax = initial_accel->x;
    float ay = initial_accel->y;
    float az = initial_accel->z;
    
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_norm < 1e-6f) {
        // 防止除零
        ekf->x.q.w = 1.0f; ekf->x.q.x = 0.0f; ekf->x.q.y = 0.0f; ekf->x.q.z = 0.0f;
    } else {
        float pitch_check = -ax / acc_norm;
        
        if (fabsf(pitch_check) > 0.99f) {
            // 接近万向节死锁，使用零旋转
            ekf->x.q.w = 1.0f;
            ekf->x.q.x = 0.0f;
            ekf->x.q.y = 0.0f;
            ekf->x.q.z = 0.0f;
        } else {
            // 使用加速度计计算初始姿态
            float roll = atan2f(ay, az);
            float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
            float yaw = 0.0f;
            
            // 欧拉角转四元数 (ZYX顺序)
            float cr = cosf(roll * 0.5f);
            float sr = sinf(roll * 0.5f);
            float cp = cosf(pitch * 0.5f);
            float sp = sinf(pitch * 0.5f);
            float cy = cosf(yaw * 0.5f);
            float sy = sinf(yaw * 0.5f);
            
            ekf->x.q.w = cr * cp * cy + sr * sp * sy;
            ekf->x.q.x = sr * cp * cy - cr * sp * sy;
            ekf->x.q.y = cr * sp * cy + sr * cp * sy;
            ekf->x.q.z = cr * cp * sy - sr * sp * cy;
        }
    }
    
    // 初始化偏置为零
    ekf->x.b.x = 0.0f;
    ekf->x.b.y = 0.0f;
    ekf->x.b.z = 0.0f;
    
    // 初始化协方差矩阵 P
    Mat7_Identity(&ekf->P);
    for (int i = 0; i < 4; i++)
        ekf->P.m[i][i] = 0.01f;  // 四元数部分
    for (int i = 4; i < 7; i++)
        ekf->P.m[i][i] = 1.0f;   // 偏置部分
    
    // 初始化过程噪声 Q
    Mat7_Zero(&ekf->Q);
    ekf->Q.m[4][4] = KF_PROCESS_NOISE_B;
    ekf->Q.m[5][5] = KF_PROCESS_NOISE_B;
    ekf->Q.m[6][6] = KF_PROCESS_NOISE_B;
    
    // 初始化观测噪声 R（基础值）
    Mat3_Identity(&ekf->R_base);
    Mat3_Scale(&ekf->R_base, &ekf->R_base, KF_MEASUREMENT_NOISE_A);
    ekf->R = ekf->R_base;  // 初始时R = R_base
}


// ==============================================================================
// EKF预测步骤
// ==============================================================================

void EKF_Predict(AttitudeEKF_t *ekf, const Vec3_t *gyro_meas, float dt) {
    // 提取状态
    float q0 = ekf->x.q.w;
    float q1 = ekf->x.q.x;
    float q2 = ekf->x.q.y;
    float q3 = ekf->x.q.z;
    
    // 应用EKF估计的偏置
    float wx = gyro_meas->x - ekf->x.b.x;
    float wy = gyro_meas->y - ekf->x.b.y;
    float wz = gyro_meas->z - ekf->x.b.z;
    
    // 构建 omega 矩阵 (4x4)
    float omega[4][4] = {
        {0.0f,  -wx,   -wy,   -wz},
        {wx,    0.0f,   wz,   -wy},
        {wy,    -wz,   0.0f,   wx},
        {wz,     wy,   -wx,   0.0f}
    };
    
    // 预测四元数: q_new = (I + 0.5*omega*dt) * q
    float q_old[4] = {q0, q1, q2, q3};
    float q_new[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    
    for (int i = 0; i < 4; i++) {
        q_new[i] = q_old[i]; // I * q
        for (int j = 0; j < 4; j++) {
            q_new[i] += 0.5f * omega[i][j] * dt * q_old[j];
        }
    }
    
    // 归一化四元数
    float q_norm = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1] +
                         q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (q_norm > 1e-6f) {
        ekf->x.q.w = q_new[0] / q_norm;
        ekf->x.q.x = q_new[1] / q_norm;
        ekf->x.q.y = q_new[2] / q_norm;
        ekf->x.q.z = q_new[3] / q_norm;
    }
    
    // 计算状态转移矩阵 F (7x7)
    float F[7][7];
    memset(F, 0, sizeof(F));
    
    // F_qq = I + 0.5*omega*dt
    for (int i = 0; i < 4; i++) {
        F[i][i] = 1.0f;
        for (int j = 0; j < 4; j++) {
            F[i][j] += 0.5f * omega[i][j] * dt;
        }
    }
    
    // F_qb = -0.5*dt*Xi(q)
    F[0][4] = 0.5f * dt * q1;  F[0][5] = 0.5f * dt * q2;  F[0][6] = 0.5f * dt * q3;
    F[1][4] = -0.5f * dt * q0; F[1][5] = 0.5f * dt * q3;  F[1][6] = -0.5f * dt * q2;
    F[2][4] = -0.5f * dt * q3; F[2][5] = -0.5f * dt * q0; F[2][6] = 0.5f * dt * q1;
    F[3][4] = 0.5f * dt * q2;  F[3][5] = -0.5f * dt * q1; F[3][6] = -0.5f * dt * q0;
    
    // F_bb = I
    F[4][4] = 1.0f;
    F[5][5] = 1.0f;
    F[6][6] = 1.0f;
    
    // 计算过程噪声传播矩阵 G (4x3)
    float G[4][3];
    G[0][0] = -0.5f * dt * q1;  G[0][1] = -0.5f * dt * q2;  G[0][2] = -0.5f * dt * q3;
    G[1][0] = 0.5f * dt * q0;   G[1][1] = -0.5f * dt * q3;  G[1][2] = 0.5f * dt * q2;
    G[2][0] = 0.5f * dt * q3;   G[2][1] = 0.5f * dt * q0;   G[2][2] = -0.5f * dt * q1;
    G[3][0] = -0.5f * dt * q2;  G[3][1] = 0.5f * dt * q1;   G[3][2] = -0.5f * dt * q0;
    
    // Q_q = G * Q_gyro * G^T
    float Q_gyro_scalar = ekf->process_noise_q;
    float Q_q[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Q_q[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                Q_q[i][j] += G[i][k] * Q_gyro_scalar * G[j][k];
            }
        }
    }
    
    // 更新 Q 矩阵的四元数部分
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            ekf->Q.m[i][j] = Q_q[i][j];
    
    // 协方差预测: P = F*P*F^T + Q
    Mat7x7_t P_old = ekf->P;
    Mat7x7_t FP, FPFT;
    
    // FP = F * P
    memset(&FP, 0, sizeof(Mat7x7_t));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                FP.m[i][j] += F[i][k] * P_old.m[k][j];
            }
        }
    }
    
    // FPFT = FP * F^T
    memset(&FPFT, 0, sizeof(Mat7x7_t));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                FPFT.m[i][j] += FP.m[i][k] * F[j][k];
            }
        }
    }
    
    // P = FPFT + Q
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            ekf->P.m[i][j] = FPFT.m[i][j] + ekf->Q.m[i][j];
        }
    }
    
    // 强制Z轴偏置为0 (工程技巧)
    ekf->x.b.z = 0.0f;
}

// ==============================================================================
// EKF更新步骤
// ==============================================================================

void EKF_Update(AttitudeEKF_t *ekf, const Vec3_t *acc_meas) {
    // 归一化加速度
    Vec3_t z = *acc_meas;
    Vec3_Normalize(&z);
    
    // 提取四元数
    float qw = ekf->x.q.w;
    float qx = ekf->x.q.x;
    float qy = ekf->x.q.y;
    float qz = ekf->x.q.z;
    
    // --- 自适应观测噪声 R ---
    // 1. 构建旋转矩阵 Cnb 并将归一化加速度转到导航系
    // fn = Cnb * fb (从机体系到导航系)
    // Cnb 矩阵（四元数表示）:
    // [1-2(qy²+qz²),  2(qxqy-qwqz),  2(qxqz+qwqy)]
    // [2(qxqy+qwqz),  1-2(qx²+qz²),  2(qyqz-qwqx)]
    // [2(qxqz-qwqy),  2(qyqz+qwqx),  1-2(qx²+qy²)]
    float acc_nav_x = (1 - 2*(qy*qy + qz*qz)) * z.x + 2*(qx*qy - qw*qz) * z.y + 2*(qx*qz + qw*qy) * z.z;
    float acc_nav_y = 2*(qx*qy + qw*qz) * z.x + (1 - 2*(qx*qx + qz*qz)) * z.y + 2*(qy*qz - qw*qx) * z.z;
    
    // 2. 计算水平加速度（理论上应该为0）
    ekf->acc_horizontal = sqrtf(acc_nav_x * acc_nav_x + acc_nav_y * acc_nav_y);
    
    // 3. 动态调整 R
    if (ekf->acc_horizontal > ekf->adaptive_r_threshold) {
        // 水平加速度越大，R 越大（权重越低）
        float ratio = ekf->acc_horizontal / ekf->adaptive_r_threshold;
        ekf->r_scale = powf(ratio, ekf->adaptive_r_exponent);
        // 限制最大缩放倍数
        if (ekf->r_scale > ekf->adaptive_r_max_scale) {
            ekf->r_scale = ekf->adaptive_r_max_scale;
        }
        Mat3_Scale(&ekf->R, &ekf->R_base, ekf->r_scale);
    } else {
        // 接近静止，使用基础 R
        ekf->r_scale = 1.0f;
        ekf->R = ekf->R_base;
    }
    
    // 预测观测 z_pred (重力方向)
    float z_pred[3];
    z_pred[0] = 2.0f * (qx * qz - qw * qy);
    z_pred[1] = 2.0f * (qy * qz + qw * qx);
    z_pred[2] = qw*qw - qx*qx - qy*qy + qz*qz;
    
    // 观测雅可比矩阵 H_q (3x4)
    float H_q[3][4] = {
        {-2.0f*qy,  2.0f*qz, -2.0f*qw,  2.0f*qx},
        { 2.0f*qx,  2.0f*qw,  2.0f*qz,  2.0f*qy},
        { 2.0f*qw, -2.0f*qx, -2.0f*qy,  2.0f*qz}
    };
    
    // 完整的观测矩阵 H (3x7) = [H_q, 0(3x3)]
    float H[3][7];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++)
            H[i][j] = H_q[i][j];
        for (int j = 4; j < 7; j++)
            H[i][j] = 0.0f;
    }
    
    // 计算 S = H*P*H^T + R (3x3)
    float HP[3][7];
    memset(HP, 0, sizeof(HP));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                HP[i][j] += H[i][k] * ekf->P.m[k][j];
            }
        }
    }
    
    float S[3][3];
    memset(S, 0, sizeof(S));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 7; k++) {
                S[i][j] += HP[i][k] * H[j][k];
            }
            if (i == j)
                S[i][j] += ekf->R.m[i][j];
        }
    }
    
    // 计算 S 的逆 (3x3矩阵求逆)
    float S_inv[3][3];
    float det = S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    
    float inv_det = 1.0f / det;
    S_inv[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det;
    S_inv[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) * inv_det;
    S_inv[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det;
    S_inv[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2]) * inv_det;
    S_inv[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    S_inv[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) * inv_det;
    S_inv[2][0] = (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * inv_det;
    S_inv[2][1] = (S[0][1]*S[2][0] - S[0][0]*S[2][1]) * inv_det;
    S_inv[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;
    
    // 计算卡尔曼增益 K = P*H^T*S_inv (7x3)
    float PHT[7][3];
    memset(PHT, 0, sizeof(PHT));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 7; k++) {
                PHT[i][j] += ekf->P.m[i][k] * H[j][k];
            }
        }
    }
    
    float K[7][3];
    memset(K, 0, sizeof(K));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                K[i][j] += PHT[i][k] * S_inv[k][j];
            }
        }
    }
    
    // 【调试】保存卡尔曼增益
    memcpy(ekf->debug_K, K, sizeof(K));
    
    // 计算新息 y = z - z_pred
    ekf->y[0] = z.x - z_pred[0];
    ekf->y[1] = z.y - z_pred[1];
    ekf->y[2] = z.z - z_pred[2];
    
    // 状态更新 x = x + K*y
    float Ky[7];
    for (int i = 0; i < 7; i++) {
        Ky[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            Ky[i] += K[i][j] * ekf->y[j];
        }
    }
    
    ekf->x.q.w += Ky[0];
    ekf->x.q.x += Ky[1];
    ekf->x.q.y += Ky[2];
    ekf->x.q.z += Ky[3];
    ekf->x.b.x += Ky[4];
    ekf->x.b.y += Ky[5];
    ekf->x.b.z += Ky[6];
    
    // 归一化四元数
    Quat_Normalize(&ekf->x.q);
    
    // 限制偏置范围
    ekf->x.b.x = fmaxf(-ekf->max_bias, fminf(ekf->max_bias, ekf->x.b.x));
    ekf->x.b.y = fmaxf(-ekf->max_bias, fminf(ekf->max_bias, ekf->x.b.y));
    ekf->x.b.z = 0.0f;  // 强制Z轴偏置为0
    
    // 协方差更新 (Joseph形式): P = (I-KH)*P*(I-KH)^T + K*R*K^T
    float I_KH[7][7];
    memset(I_KH, 0, sizeof(I_KH));
    for (int i = 0; i < 7; i++) {
        I_KH[i][i] = 1.0f;
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 3; k++) {
                I_KH[i][j] -= K[i][k] * H[k][j];
            }
        }
    }
    
    // P1 = (I-KH)*P
    float P1[7][7];
    memset(P1, 0, sizeof(P1));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                P1[i][j] += I_KH[i][k] * ekf->P.m[k][j];
            }
        }
    }
    
    // P2 = P1 * (I-KH)^T
    float P2[7][7];
    memset(P2, 0, sizeof(P2));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                P2[i][j] += P1[i][k] * I_KH[j][k];
            }
        }
    }
    
    // KR = K * R
    float KR[7][3];
    memset(KR, 0, sizeof(KR));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                KR[i][j] += K[i][k] * ekf->R.m[k][j];
            }
        }
    }
    
    // KRKT = KR * K^T
    float KRKT[7][7];
    memset(KRKT, 0, sizeof(KRKT));
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 3; k++) {
                KRKT[i][j] += KR[i][k] * K[j][k];
            }
        }
    }
    
    // P = P2 + KRKT
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            ekf->P.m[i][j] = P2[i][j] + KRKT[i][j];
        }
    }
}

// ==============================================================================
// 四元数转欧拉角
// ==============================================================================

void Quat_ToEuler(const Quat_t *q, float *roll, float *pitch, float *yaw) {
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    
    // Roll (X轴)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (Y轴)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(M_PI / 2.0f, sinp);
    else
        *pitch = asinf(sinp);
    
    // Yaw (Z轴)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny_cosp, cosy_cosp);
    
    // 转换为度
    *roll *= 180.0f / M_PI;
    *pitch *= 180.0f / M_PI;
    *yaw *= 180.0f / M_PI;
}

// ==============================================================================
// 计算纯加速度计欧拉角（调试用）
// ==============================================================================

void EKF_CalculateAccelOnlyEuler(AttitudeEKF_t *ekf, const Vec3_t *acc_meas) {
    // 归一化加速度
    Vec3_t z = *acc_meas;
    Vec3_Normalize(&z);
    
    // 计算纯加速度计欧拉角（度）
    ekf->accel_only_euler[0] = atan2f(z.y, z.z) * 180.0f / M_PI;  // Roll
    ekf->accel_only_euler[1] = atan2f(-z.x, sqrtf(z.y*z.y + z.z*z.z)) * 180.0f / M_PI;  // Pitch
    ekf->accel_only_euler[2] = 0.0f;  // Yaw无法从加速度计获得
}

