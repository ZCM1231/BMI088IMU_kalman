/**
 ******************************************************************************
 * @file    port_usb.h
 * @brief   USB 端口适配层（中断驱动）
 * @note    从 imu_module 移植，适配 STM32H7
 ******************************************************************************
 */

#ifndef __PORT_USB_H__
#define __PORT_USB_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 USB 端口适配层
 */
void port_usb_init(void);

/**
 * @brief  USB 中断回调 - 在 USB CDC 接收中断中调用
 * @param  data: 接收到的数据指针
 * @param  len: 数据长度
 * @note   此函数在中断上下文中执行，只做快速数据缓存
 */
void port_usb_on_receive(const uint8_t *data, uint32_t len);

/**
 * @brief  主循环中处理 USB 接收数据
 * @note   从环形缓冲区读取数据，解析帧并注入 msg_server
 */
void port_usb_process(void);

/**
 * @brief  USB 发送函数
 * @param  data: 要发送的数据指针
 * @param  len: 数据长度
 * @return HAL 状态
 */
HAL_StatusTypeDef port_usb_send(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_USB_H__ */

