#ifndef __CRC16_CCITT_H__
#define __CRC16_CCITT_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  CRC-16-CCITT 计算
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @return CRC16 值
 * @note   多项式: 0x1021, 初始值: 0xFFFF
 */
uint16_t crc16_ccitt_calc(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRC16_CCITT_H__ */

