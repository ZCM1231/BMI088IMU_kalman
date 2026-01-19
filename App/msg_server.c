/**
 ******************************************************************************
 * @file    msg_server.c
 * @brief   统一通信协议处理框架（消息服务器）实现
 * @note    从 imu_module 移植，适配 STM32H7
 ******************************************************************************
 */

#include "msg_server.h"
#include "crc16_ccitt.h"
#include <string.h>
#include <stdbool.h>

/* ==================== 可调参数 ==================== */

#define MSG_QUEUE_DEPTH              16      /* 入队环形队列槽位 */
#define MSG_FRAME_MAX_LEN            600     /* 最大帧长防御 */
#define MSG_MAX_HANDLER_SLOTS        16      /* 命令处理器最大条目 */
#define MSG_RX_MIN_LEN               (2+1+1+2+4+4+2+2+2) /* 最小帧：无载荷 */
#define MSG_MAX_TOTAL_PKTS           4096    /* total_pkts 合法上限 */
#define MSG_QUEUE_DROP_OLDEST        1       /* 队列满时：1=丢最旧，0=拒绝新包 */

/* ==================== 本机设备ID ==================== */

uint8_t g_device_id = 0x01;  /* 默认设备ID，可在初始化时修改 */

/* ==================== 入队队列结构 ==================== */

typedef struct {
    uint8_t  buf[MSG_FRAME_MAX_LEN];
    uint16_t len;
    uint8_t  port;
} rx_item_t;

static rx_item_t   s_queue[MSG_QUEUE_DEPTH];
static volatile uint8_t s_q_head = 0, s_q_tail = 0, s_q_cnt = 0;

/* ==================== 命令处理器表 ==================== */

typedef struct {
    uint16_t               cmd;
    msg_command_handler_t  handler;
} handler_slot_t;

static handler_slot_t s_handlers[MSG_MAX_HANDLER_SLOTS];
static uint8_t        s_handler_cnt = 0;

/* ==================== 端口发送/输入使能 ==================== */

static msg_port_send_fn s_port_senders[MSG_PORT_MAX] = {0};
static uint8_t s_port_input_enabled[MSG_PORT_MAX] = {1}; /* USB 默认开 */

/* ==================== 临界区 ==================== */

#define MSG_ENTER_CRITICAL() __disable_irq()
#define MSG_EXIT_CRITICAL()  __enable_irq()

/* ==================== 工具函数 ==================== */

static void write_u16_be(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static void write_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static msg_command_handler_t find_handler(uint16_t cmd) {
    for (uint8_t i = 0; i < s_handler_cnt; ++i) {
        if (s_handlers[i].cmd == cmd) {
            return s_handlers[i].handler;
        }
    }
    return NULL;
}

/* ==================== 框架初始化 ==================== */

void msg_server_init(void)
{
    memset((void*)s_handlers, 0, sizeof(s_handlers));
    s_handler_cnt = 0;

    MSG_ENTER_CRITICAL();
    s_q_head = s_q_tail = s_q_cnt = 0;
    MSG_EXIT_CRITICAL();
}

/* ==================== 注册/配置接口 ==================== */

msg_error_t msg_server_register_command(uint16_t cmd, msg_command_handler_t handler)
{
    if (handler == NULL) {
        return MSG_ERR_INVALID_ARG;
    }
    if (s_handler_cnt >= MSG_MAX_HANDLER_SLOTS) {
        return MSG_ERR_NO_RESOURCE;
    }

    /* 检查是否已存在，覆盖 */
    for (uint8_t i = 0; i < s_handler_cnt; ++i) {
        if (s_handlers[i].cmd == cmd) {
            s_handlers[i].handler = handler;
            return MSG_OK;
        }
    }
    
    s_handlers[s_handler_cnt].cmd = cmd;
    s_handlers[s_handler_cnt].handler = handler;
    s_handler_cnt++;
    return MSG_OK;
}

void msg_server_register_port_sender(msg_port_t port, msg_port_send_fn sender)
{
    if ((int)port >= 0 && port < MSG_PORT_MAX) {
        s_port_senders[port] = sender;
    }
}

void msg_server_set_input_enabled(msg_port_t port, uint8_t enabled)
{
    if ((int)port >= 0 && port < MSG_PORT_MAX) {
        s_port_input_enabled[port] = (enabled ? 1 : 0);
    }
}

uint8_t msg_server_queue_size(void)
{
    return s_q_cnt;
}

/* ==================== 入队 ==================== */

static msg_error_t enqueue_frame(msg_port_t port, const uint8_t *data, uint16_t len)
{
    if (len > MSG_FRAME_MAX_LEN) {
        return MSG_ERR_INVALID_FRAME;
    }

    MSG_ENTER_CRITICAL();
    if (s_q_cnt >= MSG_QUEUE_DEPTH) {
#if MSG_QUEUE_DROP_OLDEST
        s_q_head = (uint8_t)((s_q_head + 1) % MSG_QUEUE_DEPTH);
        s_q_cnt--;
#else
        MSG_EXIT_CRITICAL();
        return MSG_ERR_QUEUE_FULL;
#endif
    }
    rx_item_t *slot = &s_queue[s_q_tail];
    memcpy(slot->buf, data, len);
    slot->len = len;
    slot->port = (uint8_t)port;
    s_q_tail = (uint8_t)((s_q_tail + 1) % MSG_QUEUE_DEPTH);
    s_q_cnt++;
    MSG_EXIT_CRITICAL();
    return MSG_OK;
}

/* ==================== 输入校验与设备ID过滤 ==================== */

msg_error_t msg_server_input_from_port(msg_port_t port, const uint8_t *data, uint16_t len)
{
    if ((int)port < 0 || port >= MSG_PORT_MAX) {
        return MSG_ERR_INVALID_ARG;
    }
    if (!s_port_input_enabled[port]) {
        return MSG_ERR_DISABLED;
    }
    if (data == NULL || len < MSG_RX_MIN_LEN) {
        return MSG_ERR_INVALID_FRAME;
    }

    /* 包头/包尾检查 */
    if (!(data[0] == MSG_SOF_HI && data[1] == MSG_SOF_LO)) {
        return MSG_ERR_INVALID_FRAME;
    }
    if (!(data[len - 2] == MSG_TAIL_HI && data[len - 1] == MSG_TAIL_LO)) {
        return MSG_ERR_INVALID_FRAME;
    }

    /* 字段偏移 */
    const uint16_t off_tgt    = 3;
    const uint16_t off_total  = 6;
    const uint16_t off_seq    = 10;
    const uint16_t off_len    = 14;

    /* 最小可读长度判断 */
    if (len < (off_len + 2 + 2 + 2)) {
        return MSG_ERR_INVALID_FRAME;
    }

    /* 数据区长度与总帧长一致性 */
    uint16_t payload_len = (uint16_t)(data[off_len] << 8 | data[off_len + 1]);
    uint16_t expect_len  = (uint16_t)(2 + 1 + 1 + 2 + 4 + 4 + 2 + payload_len + 2 + 2);
    if (len != expect_len) {
        return MSG_ERR_INVALID_FRAME;
    }

    /* TOTAL/SEQ 合法性 */
    uint32_t total_pkts = (uint32_t)data[off_total] |
                          ((uint32_t)data[off_total+1] << 8) |
                          ((uint32_t)data[off_total+2] << 16) |
                          ((uint32_t)data[off_total+3] << 24);
    uint32_t seq_idx    = (uint32_t)data[off_seq] |
                          ((uint32_t)data[off_seq+1] << 8) |
                          ((uint32_t)data[off_seq+2] << 16) |
                          ((uint32_t)data[off_seq+3] << 24);
    if (total_pkts == 0 || total_pkts > MSG_MAX_TOTAL_PKTS) {
        return MSG_ERR_INVALID_FRAME;
    }
    if (seq_idx == 0 || seq_idx > total_pkts) {
        return MSG_ERR_INVALID_FRAME;
    }

    /* 设备ID过滤 */
    uint8_t target_id = data[off_tgt];
    uint8_t tgt_dev  = (uint8_t)(target_id & 0x0F);
    if (tgt_dev != g_device_id && tgt_dev != 0x0F) {
        return MSG_OK;  /* 静默丢弃非本设备的包 */
    }

    /* CRC16 校验 */
    uint16_t crc_recv = (uint16_t)(data[len - 4] << 8 | data[len - 3]);
    uint16_t crc_calc = crc16_ccitt_calc(&data[0], (uint32_t)(len - 4));
    if (crc_calc != crc_recv) {
        return MSG_ERR_CRC_MISMATCH;
    }

    return enqueue_frame(port, data, len);
}

/* ==================== Poll 分发 ==================== */

void msg_server_poll(void)
{
    while (s_q_cnt > 0) {
        /* 出队 */
        MSG_ENTER_CRITICAL();
        rx_item_t *slot = &s_queue[s_q_head];
        uint16_t local_len = slot->len;
        uint8_t  from_port = slot->port;
        uint8_t  local_frame[MSG_FRAME_MAX_LEN];
        if (local_len > MSG_FRAME_MAX_LEN) {
            s_q_head = (uint8_t)((s_q_head + 1) % MSG_QUEUE_DEPTH);
            s_q_cnt--;
            MSG_EXIT_CRITICAL();
            continue;
        }
        memcpy(local_frame, slot->buf, local_len);
        s_q_head = (uint8_t)((s_q_head + 1) % MSG_QUEUE_DEPTH);
        s_q_cnt--;
        MSG_EXIT_CRITICAL();

        /* 解析偏移 */
        const uint16_t off_type    = 2;
        const uint16_t off_tgt     = 3;
        const uint16_t off_cmd     = 4;
        const uint16_t off_total   = 6;
        const uint16_t off_seq     = 10;
        const uint16_t off_len     = 14;
        const uint16_t off_payload = 16;

        msg_packet_t pkt;
        pkt.port       = from_port;
        pkt.pkt_type   = local_frame[off_type];
        pkt.target_id  = local_frame[off_tgt];
        pkt.target_port= (uint8_t)((pkt.target_id >> 4) & 0x0F);
        pkt.target_dev = (uint8_t)(pkt.target_id & 0x0F);
        pkt.cmd        = (uint16_t)(local_frame[off_cmd] << 8 | local_frame[off_cmd+1]);
        pkt.total_pkts = (uint32_t)local_frame[off_total] |
                         ((uint32_t)local_frame[off_total+1] << 8) |
                         ((uint32_t)local_frame[off_total+2] << 16) |
                         ((uint32_t)local_frame[off_total+3] << 24);
        pkt.seq_idx    = (uint32_t)local_frame[off_seq] |
                         ((uint32_t)local_frame[off_seq+1] << 8) |
                         ((uint32_t)local_frame[off_seq+2] << 16) |
                         ((uint32_t)local_frame[off_seq+3] << 24);
        uint16_t payload_len = (uint16_t)(local_frame[off_len] << 8 | local_frame[off_len+1]);
        pkt.payload    = &local_frame[off_payload];
        pkt.length     = payload_len;

        /* 路由到处理器 */
        msg_command_handler_t handler = find_handler(pkt.cmd);
        if (handler) {
            handler(&pkt);
        }
    }
}

/* ==================== 发送响应 ==================== */

msg_error_t msg_server_send_response(const msg_packet_t *req,
                                     uint8_t status,
                                     const uint8_t *resp_data,
                                     uint16_t resp_len)
{
    if (!req) {
        return MSG_ERR_INVALID_ARG;
    }

    uint16_t payload_len = (uint16_t)(1 + resp_len);
    uint16_t frame_len = (uint16_t)(2 + 1 + 1 + 2 + 4 + 4 + 2 + payload_len + 2 + 2);
    if (frame_len > MSG_FRAME_MAX_LEN) {
        return MSG_ERR_INVALID_FRAME;
    }

    uint8_t frame[MSG_FRAME_MAX_LEN];
    uint8_t *p = frame;
    p[0] = MSG_SOF_HI;
    p[1] = MSG_SOF_LO;
    p[2] = MSG_TYPE_RESP;
    p[3] = req->target_id;
    p[4] = (uint8_t)(req->cmd >> 8);
    p[5] = (uint8_t)req->cmd;
    write_u32_le(&p[6], 1);
    write_u32_le(&p[10], 1);
    write_u16_be(&p[14], payload_len);
    p[16] = status;
    if (resp_len && resp_data) {
        memcpy(&p[17], resp_data, resp_len);
    }
    uint16_t crc = crc16_ccitt_calc(frame, (uint32_t)(frame_len - 4));
    frame[frame_len - 4] = (uint8_t)(crc >> 8);
    frame[frame_len - 3] = (uint8_t)crc;
    frame[frame_len - 2] = MSG_TAIL_HI;
    frame[frame_len - 1] = MSG_TAIL_LO;

    /* 发送：回原端口 */
    msg_port_t send_port = (msg_port_t)req->port;
    HAL_StatusTypeDef hs = HAL_ERROR;
    msg_port_send_fn fn = (send_port < MSG_PORT_MAX) ? s_port_senders[send_port] : NULL;
    if (fn) {
        hs = fn(frame, frame_len);
    }
    return (hs == HAL_OK) ? MSG_OK : MSG_ERR_IO;
}

