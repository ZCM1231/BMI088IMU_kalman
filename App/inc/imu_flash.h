/**
 ******************************************************************************
 * @file    imu_flash.h
 * @brief   IMU Flash 存储操作（适配 STM32H7）
 * @note    H7 Flash 扇区大小: 128KB (与 G4 的 2KB 页不同)
 ******************************************************************************
 */

#ifndef __IMU_FLASH_H__
#define __IMU_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ==================== H7 Flash 配置 ==================== */

/* STM32H723: 1MB Flash, 8个扇区, 每个128KB */
#define H7_FLASH_SECTOR_SIZE      0x20000U    /* 128KB */
#define H7_FLASH_SECTOR_COUNT     8

/* IMU 配置存储位置: 使用最后一个扇区 (Sector 7) */
#define IMU_CONFIG_SECTOR         FLASH_SECTOR_7
#define IMU_CONFIG_ADDR           0x080E0000U  /* Sector 7 起始地址 */
#define IMU_CONFIG_SIZE           H7_FLASH_SECTOR_SIZE

/* Flash 编程单位: H7 需要 256-bit (32字节) 对齐 */
#define H7_FLASH_WORD_SIZE        32

/* ==================== IMU 配置结构体 ==================== */

/**
 * @brief IMU 校准配置结构体
 * @note  与 imu_module 项目保持一致，便于上位机兼容
 */
typedef struct {
    float gyro_bias[3];              /* 陀螺仪偏置 [x, y, z] (deg/s) */
    float accel_bias[3];             /* 加速度计偏置 [x, y, z] (g) */
    float gyro_scale[3][3];          /* 陀螺仪缩放矩阵 3x3 */
    float accel_scale[3][3];         /* 加速度计缩放矩阵 3x3 */
    float reserved[24];              /* 保留字段，用于未来扩展 */
    uint32_t crc32;                  /* CRC32 校验和 */
} imu_config_t;

/* ==================== 公共 API ==================== */

/**
 * @brief  从 Flash 读取 IMU 配置
 * @param  config: 配置结构体指针
 * @return HAL_OK=成功, HAL_ERROR=CRC校验失败或无效
 */
HAL_StatusTypeDef flash_read_imu_config(imu_config_t *config);

/**
 * @brief  将 IMU 配置写入 Flash
 * @param  config: 配置结构体指针
 * @return HAL_OK=成功, HAL_ERROR=写入失败
 * @note   会自动计算并填充 CRC32
 */
HAL_StatusTypeDef flash_write_imu_config(const imu_config_t *config);

/**
 * @brief  读取或初始化 IMU 配置
 * @param  config: 配置结构体指针
 * @note   如果 Flash 中无有效配置，则初始化为默认值并写入
 */
void flash_read_or_init_imu_config(imu_config_t *config);

/**
 * @brief  计算 CRC32
 * @param  data: 数据指针
 * @param  size: 数据长度
 * @return CRC32 值
 */
uint32_t calculate_crc32(const uint8_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_FLASH_H__ */

