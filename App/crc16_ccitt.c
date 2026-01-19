/**
 ******************************************************************************
 * @file    crc16_ccitt.c
 * @brief   CRC-16-CCITT 校验实现
 * @note    多项式: 0x1021, 初始值: 0xFFFF
 ******************************************************************************
 */

#include "crc16_ccitt.h"

/**
 * @brief  CRC-16-CCITT 在线计算
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @return CRC16 值
 */
uint16_t crc16_ccitt_calc(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    const uint16_t poly = 0x1021;
    
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= ((uint16_t)data[i] << 8);
        
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return crc;
}

