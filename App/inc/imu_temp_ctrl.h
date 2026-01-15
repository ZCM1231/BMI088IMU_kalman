/**
  ******************************************************************************
  * @file    imu_temp_ctrl.h
  * @brief   IMU温度控制头文件 (无OS版本)
  ******************************************************************************
  */

#ifndef __IMU_TEMP_CTRL_H
#define __IMU_TEMP_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "kalman_filter.h"

/* Exported types ------------------------------------------------------------*/

/* USB数据包结构体 (参考App工程格式) */
#pragma pack(push, 1)  // 1字节对齐
typedef struct {
    uint16_t header;              // 包头标识 0x9527
    
    // IMU裸数据
    float accel[3];               // 加速度 (g)
    float gyro[3];                // 角速度 (rad/s)
    float quaternion[4];          // 四元数 (w,x,y,z) - 暂不解算，置零
    
    // 维测信息
    uint16_t timestamp;           // 时间戳
    float bias[3];                // 陀螺仪零偏估计 - 暂置零
    uint16_t temperature;         // 温度原始值 (ICM42688格式)
    
    uint8_t status;               // 状态位
    
    uint16_t crc;                 // CRC校验
} IMU_USBPacket_t;
#pragma pack(pop)

/* Exported constants --------------------------------------------------------*/
#define IMU_PACKET_HEADER   0x9527  // 数据包头

#define DES_TEMP    40.0f   // 目标温度
#define KP          100.f   // 比例系数
#define KI          50.f    // 积分系数
#define KD          10.f    // 微分系数
#define MAX_OUT     500     // 最大输出

/* Exported variables --------------------------------------------------------*/
extern float gyro[3];
extern float accel[3];
extern float temp;
extern uint8_t forceStop;
extern AttitudeEKF_t ekf;  // EKF滤波器实例

/* Exported functions prototypes ---------------------------------------------*/
void IMU_TempCtrl_Init(void);
void IMU_TempCtrl_Loop(void);
void IMU_SetDataReadyFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_TEMP_CTRL_H */

