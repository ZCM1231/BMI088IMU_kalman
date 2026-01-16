/**
  ******************************************************************************
  * @file    imu_temp_ctrl.c
  * @brief   IMU温度控制 (无OS版本)
  * @details 通过PID控制IMU温度，保持恒温以提高测量精度
  *          通过USB发送IMU数据到上位机
  ******************************************************************************
  */

#include "imu_temp_ctrl.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
static uint16_t CRC16_Calculate(const uint8_t *data, uint16_t len);

/* Private variables ---------------------------------------------------------*/
float gyro[3], accel[3], temp;
uint8_t forceStop = 0;

static float out = 0;
static float err = 0;
static float err_l = 0;
static float err_ll = 0;

static volatile uint8_t imu_data_ready = 0;
static uint16_t imu_timestamp = 0;

/* EKF滤波器实例 */
AttitudeEKF_t ekf;
static uint8_t ekf_initialized = 0;

/* EKF时间步长 (1ms = 0.001s) */
#define EKF_DT  0.001f

/**
  * @brief  IMU温控初始化
  * @retval None
  */
void IMU_TempCtrl_Init(void)
{
    /* 启动PWM输出 (TIM3 CH4用于加热控制) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    /* 初始化BMI088 */
    while(BMI088_init())
    {
        ;
    }
}

/**
  * @brief  IMU温控主循环处理
  * @note   在主循环中调用此函数
  * @retval None
  */
void IMU_TempCtrl_Loop(void)
{
    /* 检查数据就绪标志 */
    if (imu_data_ready == 0)
    {
        return;
    }
    
    /* 清除标志 */
    imu_data_ready = 0;
    
    /* 读取IMU数据 */
    BMI088_read(gyro, accel, &temp);
    
    /* PID计算 */
    err_ll = err_l;
    err_l = err;
    err = DES_TEMP - temp;
    out = KP * err + KI * (err + err_l + err_ll) + KD * (err - err_l);
    
    /* 输出限幅 */
    if (out > MAX_OUT) out = MAX_OUT;
    if (out < 0) out = 0.f;
    
    /* 强制停止检查 */
    if (forceStop == 1)
    {
        out = 0.0f;
    }
    
    /* 设置PWM占空比 */
    htim3.Instance->CCR4 = (uint16_t)out;
    
    /* ========== EKF姿态解算 ========== */
    /* 准备EKF输入数据 */
    Vec3_t gyro_vec = {gyro[0], gyro[1], gyro[2]};  // rad/s
    Vec3_t accel_vec = {accel[0], accel[1], accel[2]};  // m/s² (BMI088已转换)
    
    /* 首次运行时初始化EKF */
    if (!ekf_initialized)
    {
        EKF_Init(&ekf, &accel_vec);
        ekf_initialized = 1;
    }
    else
    {
        /* ====== 开始计时 ====== */
        uint32_t start_cycles = DWT_CYCCNT;
        
        /* EKF预测步骤 (使用陀螺仪) */
        EKF_Predict(&ekf, &gyro_vec, EKF_DT);
        
        /* EKF更新步骤 (使用加速度计) */
        EKF_Update(&ekf, &accel_vec);
        
        /* ====== 结束计时并更新统计 ====== */
        uint32_t end_cycles = DWT_CYCCNT;
        uint32_t elapsed = end_cycles - start_cycles;
        
        // 更新统计数据
        ekf_timing.last_cycles = elapsed;
        ekf_timing.count++;
        ekf_timing.total_cycles += elapsed;
        
        if (elapsed < ekf_timing.min_cycles) {
            ekf_timing.min_cycles = elapsed;
        }
        if (elapsed > ekf_timing.max_cycles) {
            ekf_timing.max_cycles = elapsed;
        }
        
        // 转换为微秒 (方便观测)
        ekf_timing.last_us = (float)elapsed * 1000000.0f / (float)CPU_FREQ_HZ;
        ekf_timing.min_us = (float)ekf_timing.min_cycles * 1000000.0f / (float)CPU_FREQ_HZ;
        ekf_timing.max_us = (float)ekf_timing.max_cycles * 1000000.0f / (float)CPU_FREQ_HZ;
        ekf_timing.avg_us = (float)ekf_timing.total_cycles / (float)ekf_timing.count * 1000000.0f / (float)CPU_FREQ_HZ;
    }
    
    /* 构建USB数据包 */
    IMU_USBPacket_t packet;
    memset(&packet, 0, sizeof(packet));
    
    packet.header = IMU_PACKET_HEADER;  // 0x9527
    
    /* IMU裸数据 */
    /* 加速度: m/s² -> g (除以重力加速度) */
    packet.accel[0] = accel[0] / 9.80665f;
    packet.accel[1] = accel[1] / 9.80665f;
    packet.accel[2] = accel[2] / 9.80665f;
    /* 陀螺仪: rad/s (无需转换) */
    packet.gyro[0] = gyro[0];
    packet.gyro[1] = gyro[1];
    packet.gyro[2] = gyro[2];
    
    /* 四元数 - 来自EKF解算结果 */
    packet.quaternion[0] = ekf.x.q.w;  // w
    packet.quaternion[1] = ekf.x.q.x;  // x
    packet.quaternion[2] = ekf.x.q.y;  // y
    packet.quaternion[3] = ekf.x.q.z;  // z
    
    /* 时间戳 */
    packet.timestamp = imu_timestamp++;
    
    /* 零偏 - 来自EKF估计 */
    packet.bias[0] = ekf.x.b.x;
    packet.bias[1] = ekf.x.b.y;
    packet.bias[2] = ekf.x.b.z;
    
    /* 温度原始值 - 转换为ICM42688格式: temp = raw/132.48 + 25, 所以 raw = (temp-25)*132.48 */
    packet.temperature = (uint16_t)((temp - 25.0f) * 132.48f);
    
    /* 状态位 */
    packet.status = 0x00;
    
    /* 计算CRC (不包含CRC字段本身) */
    packet.crc = CRC16_Calculate((uint8_t*)&packet, sizeof(packet) - 2);
    
    /* 通过USB发送 */
    CDC_Transmit_HS((uint8_t*)&packet, sizeof(packet));
}

/**
  * @brief  设置数据就绪标志
  * @note   在GPIO外部中断回调中调用
  * @retval None
  */
void IMU_SetDataReadyFlag(void)
{
    imu_data_ready = 1;
}

/**
  * @brief  GPIO外部中断回调
  * @param  GPIO_Pin: 触发中断的GPIO引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GYRO_INT_Pin)
    {
        /* 使用陀螺仪中断触发数据读取 (1000Hz, 1ms一次) */
        IMU_SetDataReadyFlag();
    }
    else if(GPIO_Pin == ACC_INT_Pin)
    {
        /* 加速度计中断 (800Hz, 1.25ms一次) - 暂不使用 */
    }
}

/**
  * @brief  CRC16-Modbus计算 (与App工程一致)
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval CRC16值
  */
static uint16_t CRC16_Calculate(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

