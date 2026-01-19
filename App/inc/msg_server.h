/**
 ******************************************************************************
 * @file    msg_server.h
 * @brief   统一通信协议处理框架（消息服务器）
 * @note    从 imu_module 移植，适配 STM32H7
 ******************************************************************************
 */

#ifndef __MSG_SERVER_H__
#define __MSG_SERVER_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 端口定义 ==================== */

typedef enum {
    MSG_PORT_USB  = 0,
    MSG_PORT_MAX
} msg_port_t;

/* ==================== 错误码定义 ==================== */

typedef enum {
    MSG_OK                   = 0,
    MSG_ERR_INVALID_FRAME    = -1,
    MSG_ERR_CRC_MISMATCH     = -2,
    MSG_ERR_QUEUE_FULL       = -3,
    MSG_ERR_UNKNOWN_COMMAND  = -4,
    MSG_ERR_INVALID_ARG      = -5,
    MSG_ERR_DISABLED         = -6,
    MSG_ERR_BUSY             = -7,
    MSG_ERR_IO               = -8,
    MSG_ERR_NO_RESOURCE      = -10
} msg_error_t;

/* ==================== 协议帧格式常量 ==================== */

#define MSG_SOF_HI         0xAA
#define MSG_SOF_LO         0x55
#define MSG_TAIL_HI        0x55
#define MSG_TAIL_LO        0xAA

/* ==================== 包类型 ==================== */

typedef enum {
    MSG_TYPE_CMD   = 0x01,
    MSG_TYPE_RESP  = 0x02,
    MSG_TYPE_EVENT = 0x03,
} msg_type_t;

/* ==================== 统一消息包 ==================== */

/**
 * @brief 统一消息包（已解帧）
 * @note  target_id 编码：高4位=端口，低4位=设备ID
 */
typedef struct {
    uint8_t     port;          /* 来源端口 */
    uint8_t     pkt_type;      /* 包类型 */
    uint8_t     target_id;     /* 目标(端口,设备ID) 合成字节 */
    uint8_t     target_port;   /* 解析后的目标端口(高4位) */
    uint8_t     target_dev;    /* 解析后的目标设备ID(低4位) */
    uint16_t    cmd;           /* 命令/功能码 */
    uint32_t    total_pkts;    /* 总数据包数 */
    uint32_t    seq_idx;       /* 当前数据包序号（1..N） */
    const uint8_t *payload;    /* 数据指针 */
    uint16_t    length;        /* 数据长度 */
} msg_packet_t;

/* ==================== 函数指针类型 ==================== */

typedef msg_error_t (*msg_command_handler_t)(const msg_packet_t *pkt);
typedef HAL_StatusTypeDef (*msg_port_send_fn)(const uint8_t *data, uint16_t len);

/* ==================== 公共 API ==================== */

/**
 * @brief  初始化消息服务器
 */
void msg_server_init(void);

/**
 * @brief  端口输入统一入口
 * @param  port: 端口号
 * @param  data: 帧数据指针
 * @param  len: 帧长度
 * @return 错误码
 */
msg_error_t msg_server_input_from_port(msg_port_t port, const uint8_t *data, uint16_t len);

/**
 * @brief  注册命令处理器
 * @param  cmd: 命令码
 * @param  handler: 处理函数
 * @return 错误码
 */
msg_error_t msg_server_register_command(uint16_t cmd, msg_command_handler_t handler);

/**
 * @brief  注册端口发送函数
 * @param  port: 端口号
 * @param  sender: 发送函数指针
 */
void msg_server_register_port_sender(msg_port_t port, msg_port_send_fn sender);

/**
 * @brief  设置端口输入使能
 * @param  port: 端口号
 * @param  enabled: 1=使能, 0=禁用
 */
void msg_server_set_input_enabled(msg_port_t port, uint8_t enabled);

/**
 * @brief  获取队列大小
 * @return 当前队列中的消息数
 */
uint8_t msg_server_queue_size(void);

/**
 * @brief  发送响应
 * @param  req: 原始请求包
 * @param  status: 状态码
 * @param  resp_data: 响应数据
 * @param  resp_len: 响应数据长度
 * @return 错误码
 */
msg_error_t msg_server_send_response(const msg_packet_t *req,
                                     uint8_t status,
                                     const uint8_t *resp_data,
                                     uint16_t resp_len);

/**
 * @brief  主循环轮询处理
 * @note   从队列取包并分发到处理器
 */
void msg_server_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __MSG_SERVER_H__ */

