/**
 ******************************************************************************
 * @file    imu_commands.h
 * @brief   IMU 命令处理
 * @note    兼容 imu_module 上位机协议
 ******************************************************************************
 */

#ifndef __IMU_COMMANDS_H__
#define __IMU_COMMANDS_H__

#include <stdint.h>
#include "imu_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 全局配置变量 ==================== */

extern imu_config_t g_imu_config;

/* ==================== 命令码定义 ==================== */

#define CMD_HEARTBEAT             0x0001  /* 心跳 */
#define CMD_SET_IMU_CONFIG        0x0070  /* 设置IMU配置参数 */
#define CMD_GET_IMU_CONFIG        0x0071  /* 读取IMU配置参数 */
#define CMD_SET_CALIBRATION_MODE  0x0072  /* 设置标定模式 */
#define CMD_GET_CALIBRATION_MODE  0x0073  /* 查询标定模式 */

/* ==================== 公共 API ==================== */

/**
 * @brief  注册 IMU 命令处理器
 */
void imu_commands_register(void);

/**
 * @brief  暂停 IMU 数据发送
 * @note   收到命令时调用，让出 USB 通道
 */
void imu_suspend(void);

/**
 * @brief  恢复 IMU 数据发送
 */
void imu_resume(void);

/**
 * @brief  查询 IMU 是否暂停
 * @return 1=暂停, 0=正常
 * @note   超时 3 秒自动恢复
 */
uint8_t imu_is_suspended(void);

/**
 * @brief  设置标定模式
 * @param  enable: 1=标定模式, 0=正常模式
 */
void imu_set_calibration_mode(uint8_t enable);

/**
 * @brief  查询标定模式
 * @return 1=标定模式, 0=正常模式
 */
uint8_t imu_get_calibration_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_COMMANDS_H__ */

