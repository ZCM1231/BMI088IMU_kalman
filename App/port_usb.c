/**
 ******************************************************************************
 * @file    port_usb.c
 * @brief   USB 端口适配层实现
 * @note    从 imu_module 移植，适配 STM32H7 (USB HS)
 ******************************************************************************
 */

#include "port_usb.h"
#include "msg_server.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* ==================== 环形缓冲区 ==================== */

#define USB_RX_BUFFER_SIZE 1024
static uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
static volatile uint16_t rx_write_idx = 0;  /* 中断写入位置 */
static uint16_t rx_read_idx = 0;            /* 主循环读取位置 */

/* ==================== 帧解析状态机 ==================== */

typedef enum {
    FRAME_STATE_IDLE = 0,
    FRAME_STATE_SOF1_FOUND,      /* 找到0xAA */
    FRAME_STATE_SOF2_FOUND,      /* 找到0x55 */
    FRAME_STATE_COLLECTING       /* 正在收集完整帧 */
} frame_state_t;

static frame_state_t s_frame_state = FRAME_STATE_IDLE;
static uint8_t s_frame_buffer[600];  /* 最大帧长 */
static uint16_t s_frame_len = 0;
static uint16_t s_frame_expected_len = 0;

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化 USB 端口适配层
 */
void port_usb_init(void)
{
    rx_write_idx = 0;
    rx_read_idx = 0;
    s_frame_state = FRAME_STATE_IDLE;
    s_frame_len = 0;
    s_frame_expected_len = 0;
    
    /* 注册 USB 发送函数到 msg_server */
    msg_server_register_port_sender(MSG_PORT_USB, port_usb_send);
    msg_server_set_input_enabled(MSG_PORT_USB, 1);
}

/**
 * @brief  USB 中断回调
 */
void port_usb_on_receive(const uint8_t *data, uint32_t len)
{
    /* 中断上下文：快速拷贝到环形缓冲区 */
    for (uint32_t i = 0; i < len; i++) {
        usb_rx_buffer[rx_write_idx] = data[i];
        rx_write_idx = (rx_write_idx + 1) % USB_RX_BUFFER_SIZE;
        
        /* 检查缓冲区溢出 */
        if (rx_write_idx == rx_read_idx) {
            /* 缓冲区满，丢弃最旧数据 */
            rx_read_idx = (rx_read_idx + 1) % USB_RX_BUFFER_SIZE;
        }
    }
}

/**
 * @brief  从环形缓冲区读取一个字节
 */
static uint8_t ring_buffer_read_byte(uint8_t *byte)
{
    if (rx_read_idx == rx_write_idx) {
        return 1;  /* 缓冲区空 */
    }
    
    *byte = usb_rx_buffer[rx_read_idx];
    rx_read_idx = (rx_read_idx + 1) % USB_RX_BUFFER_SIZE;
    return 0;
}

/**
 * @brief  帧解析状态机
 */
static void process_frame_byte(uint8_t byte)
{
    switch (s_frame_state) {
        case FRAME_STATE_IDLE:
            if (byte == 0xAA) {
                s_frame_state = FRAME_STATE_SOF1_FOUND;
                s_frame_buffer[0] = byte;
                s_frame_len = 1;
            }
            break;
            
        case FRAME_STATE_SOF1_FOUND:
            if (byte == 0x55) {
                s_frame_state = FRAME_STATE_SOF2_FOUND;
                s_frame_buffer[1] = byte;
                s_frame_len = 2;
            } else if (byte == 0xAA) {
                /* 可能是新帧开始 */
                s_frame_buffer[0] = byte;
                s_frame_len = 1;
            } else {
                s_frame_state = FRAME_STATE_IDLE;
                s_frame_len = 0;
            }
            break;
            
        case FRAME_STATE_SOF2_FOUND:
        case FRAME_STATE_COLLECTING:
            if (s_frame_len < sizeof(s_frame_buffer)) {
                s_frame_buffer[s_frame_len++] = byte;
            } else {
                /* 帧缓冲区溢出，重置 */
                s_frame_state = FRAME_STATE_IDLE;
                s_frame_len = 0;
                break;
            }
            
            /* 检查是否已收到长度字段（offset 14-15） */
            if (s_frame_len == 16) {
                /* 解析payload长度（大端序） */
                uint16_t payload_len = (uint16_t)(s_frame_buffer[14] << 8 | s_frame_buffer[15]);
                /* 计算期望的完整帧长 */
                s_frame_expected_len = 2 + 1 + 1 + 2 + 4 + 4 + 2 + payload_len + 2 + 2;
                
                if (s_frame_expected_len > sizeof(s_frame_buffer)) {
                    /* 帧长度异常，重置 */
                    s_frame_state = FRAME_STATE_IDLE;
                    s_frame_len = 0;
                    break;
                }
                
                s_frame_state = FRAME_STATE_COLLECTING;
            }
            
            /* 检查是否收集完整帧 */
            if (s_frame_state == FRAME_STATE_COLLECTING && s_frame_len >= s_frame_expected_len) {
                /* 验证帧尾 */
                if (s_frame_buffer[s_frame_len - 2] == 0x55 && 
                    s_frame_buffer[s_frame_len - 1] == 0xAA) {
                    /* 完整帧，注入msg_server */
                    msg_server_input_from_port(MSG_PORT_USB, s_frame_buffer, s_frame_len);
                }
                
                /* 重置状态机 */
                s_frame_state = FRAME_STATE_IDLE;
                s_frame_len = 0;
                s_frame_expected_len = 0;
            }
            break;
    }
}

/**
 * @brief  主循环中处理 USB 接收数据
 */
void port_usb_process(void)
{
    uint8_t byte;
    uint16_t processed_count = 0;
    
    /* 处理缓冲区中的数据，限制单次处理量 */
    while (ring_buffer_read_byte(&byte) == 0 && processed_count < 200) {
        process_frame_byte(byte);
        processed_count++;
    }
}

/**
 * @brief  USB 发送函数
 */
HAL_StatusTypeDef port_usb_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return HAL_ERROR;
    }
    
    /* H7 使用 USB HS */
    uint8_t result = CDC_Transmit_HS((uint8_t*)data, len);
    
    return (result == USBD_OK) ? HAL_OK : HAL_BUSY;
}

