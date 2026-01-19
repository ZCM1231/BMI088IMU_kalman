/**
 ******************************************************************************
 * @file    imu_flash.c
 * @brief   IMU Flash 存储操作实现（适配 STM32H7）
 * @note    参考 cecu 项目 ota_app_flash_ops.c 实现
 *          H7 Flash 特性:
 *          - 扇区大小: 128KB (Sector)
 *          - 编程对齐: 256-bit (32字节)
 *          - 编程类型: FLASH_TYPEPROGRAM_FLASHWORD
 *          - 第三个参数是指向32字节对齐缓冲区的指针
 ******************************************************************************
 */

#include "imu_flash.h"
#include <string.h>

/* ==================== CRC32 计算 ==================== */

/**
 * @brief  计算 CRC32 (与 imu_module 一致)
 */
uint32_t calculate_crc32(const uint8_t *data, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xEDB88320U;
            } else {
                crc = (crc >> 1);
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ==================== H7 Flash 底层操作 ==================== */

/**
 * @brief  擦除指定扇区 (参考 cecu flash_erase_area)
 * @param  sector: 扇区号 (FLASH_SECTOR_0 - FLASH_SECTOR_7)
 * @return HAL 状态
 */
static HAL_StatusTypeDef h7_flash_erase_sector(uint32_t sector)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;
    
    /* 关闭中断，防止 Flash 操作期间出错 */
    __disable_irq();
    
    /* 解锁 Flash */
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
    
    /* 配置擦除参数 */
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7V - 3.6V */
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Sector = sector;
    erase_init.NbSectors = 1;
    
    /* 执行擦除 */
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    
    /* 锁定 Flash */
    HAL_FLASH_Lock();
    
    /* 恢复中断 */
    __enable_irq();
    
    return status;
}

/**
 * @brief  写入数据到 Flash (H7 需要 32 字节对齐)
 * @param  addr: 目标地址 (必须 32 字节对齐)
 * @param  data: 数据指针
 * @param  size: 数据长度
 * @return HAL 状态
 * @note   参考 cecu flash_write_data 实现
 *         H7 的 HAL_FLASH_Program 第三个参数是 (uint64_t)(uintptr_t)buf32
 */
static HAL_StatusTypeDef h7_flash_write(uint32_t addr, const uint8_t *data, uint32_t size)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    /* H7 Flash 写入需要 32 字节对齐的缓冲区 */
    __attribute__((aligned(32))) uint8_t buf32[H7_FLASH_WORD_SIZE];
    
    /* 关闭中断，防止 Flash 操作期间出错 */
    __disable_irq();
    
    /* 解锁 Flash */
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2);
    
    /* 按 32 字节块写入 */
    uint32_t offset = 0;
    while (offset < size) {
        uint32_t chunk = (size - offset) > H7_FLASH_WORD_SIZE ? H7_FLASH_WORD_SIZE : (size - offset);
        
        /* 准备一个 Flash Word (填充 0xFF) */
        memset(buf32, 0xFF, sizeof(buf32));
        memcpy(buf32, data + offset, chunk);
        
        /* 写入一个 Flash Word (256-bit) 
         * 关键：第三个参数是 (uint64_t)(uintptr_t)buf32 */
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + offset, (uint64_t)(uintptr_t)buf32);
        if (status != HAL_OK) {
            break;
        }
        
        offset += H7_FLASH_WORD_SIZE;
    }
    
    /* 锁定 Flash */
    HAL_FLASH_Lock();
    
    /* 恢复中断 */
    __enable_irq();
    
    return status;
}

/* ==================== IMU 配置读写 ==================== */

/**
 * @brief  从 Flash 读取 IMU 配置
 */
HAL_StatusTypeDef flash_read_imu_config(imu_config_t *config)
{
    if (config == NULL) {
        return HAL_ERROR;
    }
    
    /* 直接从 Flash 读取 */
    memcpy(config, (void*)IMU_CONFIG_ADDR, sizeof(imu_config_t));
    
    /* 验证 CRC32 */
    uint32_t calculated_crc = calculate_crc32((const uint8_t*)config, 
                                              sizeof(imu_config_t) - sizeof(uint32_t));
    
    if (calculated_crc == config->crc32) {
        return HAL_OK;
    }
    
    return HAL_ERROR;
}

/**
 * @brief  将 IMU 配置写入 Flash
 */
HAL_StatusTypeDef flash_write_imu_config(const imu_config_t *config)
{
    if (config == NULL) {
        return HAL_ERROR;
    }
    
    /* 复制配置并计算 CRC32 */
    imu_config_t cfg = *config;
    cfg.crc32 = calculate_crc32((const uint8_t*)&cfg, 
                                sizeof(imu_config_t) - sizeof(uint32_t));
    
    /* 擦除配置扇区 */
    if (h7_flash_erase_sector(IMU_CONFIG_SECTOR) != HAL_OK) {
        return HAL_ERROR;
    }
    
    /* 写入配置 */
    return h7_flash_write(IMU_CONFIG_ADDR, (const uint8_t*)&cfg, sizeof(imu_config_t));
}

/**
 * @brief  读取或初始化 IMU 配置
 */
void flash_read_or_init_imu_config(imu_config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    /* 尝试读取现有配置 */
    if (flash_read_imu_config(config) == HAL_OK) {
        /* 配置有效 */
        return;
    }
    
    /* 配置无效，初始化默认值（不自动写入Flash，避免启动时硬件错误） */
    memset(config, 0, sizeof(imu_config_t));
    
    /* 默认陀螺仪偏置为 0 */
    config->gyro_bias[0] = 0.0f;
    config->gyro_bias[1] = 0.0f;
    config->gyro_bias[2] = 0.0f;
    
    /* 默认加速度计偏置为 0 */
    config->accel_bias[0] = 0.0f;
    config->accel_bias[1] = 0.0f;
    config->accel_bias[2] = 0.0f;
    
    /* 默认缩放矩阵为单位矩阵 */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            config->gyro_scale[i][j] = (i == j) ? 1.0f : 0.0f;
            config->accel_scale[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    /* 注意：不在启动时自动写入 Flash
     * 仅当上位机发送配置命令时才写入 */
}
