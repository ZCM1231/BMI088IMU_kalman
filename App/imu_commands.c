/**
 ******************************************************************************
 * @file    imu_commands.c
 * @brief   IMU 命令处理实现
 * @note    兼容 imu_module 上位机协议
 ******************************************************************************
 */

#include "imu_commands.h"
#include "msg_server.h"
#include "imu_flash.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ==================== IMU 暂停控制 ==================== */

static volatile uint8_t s_imu_suspended = 0;
static uint32_t s_imu_suspend_time = 0;

/* ==================== 标定模式控制 ==================== */

static volatile uint8_t s_calibration_mode = 0;

/* ==================== IMU 配置（全局） ==================== */

imu_config_t g_imu_config;

/* ==================== 暂停控制函数 ==================== */

void imu_suspend(void)
{
    s_imu_suspended = 1;
    s_imu_suspend_time = HAL_GetTick();
}

void imu_resume(void)
{
    s_imu_suspended = 0;
}

uint8_t imu_is_suspended(void)
{
    /* 超时3秒自动恢复，防止异常卡住 */
    if (s_imu_suspended && (HAL_GetTick() - s_imu_suspend_time > 3000)) {
        s_imu_suspended = 0;
    }
    return s_imu_suspended;
}

/* ==================== 标定模式控制函数 ==================== */

void imu_set_calibration_mode(uint8_t enable)
{
    s_calibration_mode = enable ? 1 : 0;
}

uint8_t imu_get_calibration_mode(void)
{
    return s_calibration_mode;
}

/* ==================== 命令处理函数 ==================== */

/**
 * @brief  心跳命令处理 (0x0001)
 */
static msg_error_t h_heartbeat(const msg_packet_t *pkt)
{
    /* 收到心跳，暂停 IMU 数据发送 */
    imu_suspend();
    
    return msg_server_send_response(pkt, 0x00, NULL, 0);
}

/**
 * @brief  设置 IMU 配置参数命令处理 (0x0070)
 * @note   数据格式：
 *         - gyro_bias[3]: 12 bytes
 *         - accel_bias[3]: 12 bytes
 *         - gyro_scale[9]: 36 bytes
 *         - accel_scale[9]: 36 bytes
 *         - reserved[24]: 96 bytes
 *         总计：192 bytes (不含 CRC32)
 */
static msg_error_t h_set_imu_config(const msg_packet_t *pkt)
{
    imu_suspend();
    
    /* 检查数据长度 */
    const uint16_t expected_len = sizeof(imu_config_t) - sizeof(uint32_t);
    if (pkt->length < expected_len) {
        return msg_server_send_response(pkt, 0x01, NULL, 0);  /* 参数错误 */
    }
    
    /* 解析参数 */
    imu_config_t config;
    memset(&config, 0, sizeof(config));
    
    const uint8_t *p = pkt->payload;
    
    memcpy(config.gyro_bias, p, 12);
    p += 12;
    memcpy(config.accel_bias, p, 12);
    p += 12;
    memcpy(config.gyro_scale, p, 36);
    p += 36;
    memcpy(config.accel_scale, p, 36);
    p += 36;
    memcpy(config.reserved, p, 96);
    
    /* 写入 Flash */
    HAL_StatusTypeDef st = flash_write_imu_config(&config);
    if (st != HAL_OK) {
        return msg_server_send_response(pkt, 0x03, NULL, 0);  /* 写入失败 */
    }
    
    /* 重新加载配置到内存 */
    flash_read_or_init_imu_config(&g_imu_config);
    
    /* 刷新暂停时间 */
    imu_suspend();
    
    return msg_server_send_response(pkt, 0x00, NULL, 0);
}

/**
 * @brief  读取 IMU 配置参数命令处理 (0x0071)
 * @note   如果 Flash 中没有有效配置，返回默认配置（全0偏置，单位矩阵缩放）
 */
static msg_error_t h_get_imu_config(const msg_packet_t *pkt)
{
    imu_suspend();
    
    imu_config_t config;
    
    HAL_StatusTypeDef st = flash_read_imu_config(&config);
    if (st != HAL_OK) {
        /* Flash 中没有有效配置，返回当前内存中的配置（已初始化为默认值） */
        memcpy(&config, &g_imu_config, sizeof(imu_config_t));
    }
    
    /* 返回数据（不含 CRC32） */
    const uint16_t resp_len = sizeof(imu_config_t) - sizeof(uint32_t);
    return msg_server_send_response(pkt, 0x00, (const uint8_t*)&config, resp_len);
}

/**
 * @brief  设置标定模式命令处理 (0x0072)
 * @note   数据格式：mode(1) - 0=正常模式, 1=标定模式
 */
static msg_error_t h_set_calibration_mode(const msg_packet_t *pkt)
{
    imu_suspend();
    
    if (pkt->length < 1) {
        return msg_server_send_response(pkt, 0x01, NULL, 0);
    }
    
    uint8_t mode = pkt->payload[0];
    
    if (mode > 1) {
        return msg_server_send_response(pkt, 0x01, NULL, 0);
    }
    
    imu_set_calibration_mode(mode);
    
    /* 延迟恢复 IMU */
    HAL_Delay(100);
    imu_resume();
    
    return msg_server_send_response(pkt, 0x00, NULL, 0);
}

/**
 * @brief  查询标定模式命令处理 (0x0073)
 */
static msg_error_t h_get_calibration_mode(const msg_packet_t *pkt)
{
    imu_suspend();
    
    uint8_t mode = imu_get_calibration_mode();
    
    imu_resume();
    
    return msg_server_send_response(pkt, 0x00, &mode, 1);
}

/* ==================== 注册命令处理器 ==================== */

void imu_commands_register(void)
{
    msg_server_register_command(CMD_HEARTBEAT, h_heartbeat);
    msg_server_register_command(CMD_SET_IMU_CONFIG, h_set_imu_config);
    msg_server_register_command(CMD_GET_IMU_CONFIG, h_get_imu_config);
    msg_server_register_command(CMD_SET_CALIBRATION_MODE, h_set_calibration_mode);
    msg_server_register_command(CMD_GET_CALIBRATION_MODE, h_get_calibration_mode);
}

