/**
 * @file    unitree_motor.h
 * @author  Copilot
 * @date    2025-12-19
 * @version 1.3.0
 * @brief   宇树电机 GO-M8010-6 驱动层 (仿 DJI 驱动风格)
 *
 * 使用说明:
 * 1. 在应用层初始化时注册 RX Event 回调:
 *    HAL_UART_RegisterRxEventCallback(&huart1, Unitree_RxEventCallback);
 *    HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, Unitree_TxCpltCallback); // 必须注册发送完成回调以切换 RS485 方向
 *
 * 2. 初始化电机对象:
 *    Unitree_Init(&hUnitree, (Unitree_Config_t){.huart = &huart1, .id = 1, ...});
 *
 * 3. 设置控制指令 (不发送):
 *    __Unitree_SET_CMD(&hUnitree, mode, tau, omega, pos, kp, kw);
 *
 * 4. 周期性发送指令:
 *    Unitree_SendCommand(&huart1);
 */

#ifndef UNITREE_MOTOR_H
#define UNITREE_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// 协议常量定义
#define UNITREE_POS_SCALE (32768.0f / 6.2832f)
#define UNITREE_SPD_SCALE (256.0f / 6.2832f)
#define UNITREE_TOR_SCALE 256.0f
#define UNITREE_KP_SCALE  (32768 / 25.6f)
#define UNITREE_KW_SCALE  (32768 / 25.6f)
// 协议结构体 (1字节对齐)
#pragma pack(1)

typedef struct
{
    uint8_t id : 4;
    uint8_t status : 3;
    uint8_t none : 1;
} RIS_Mode_t;

typedef struct
{
    int16_t tor_des; // q8
    int16_t spd_des; // q7
    int32_t pos_des; // q15
    uint16_t k_pos;  // q15
    uint16_t k_spd;  // q15
} RIS_Comd_t;

typedef struct
{
    int16_t torque; // q8
    int16_t speed;  // q7
    int32_t pos;    // q15
    int8_t temp;
    uint8_t MError : 3;
    uint16_t force : 12;
    uint8_t none : 1;
} RIS_Fbk_t;

typedef struct
{
    uint8_t head[2];
    RIS_Mode_t mode;
    RIS_Comd_t comd;
    uint16_t CRC16;
} ControlData_t;

typedef struct
{
    uint8_t head[2];
    RIS_Mode_t mode;
    RIS_Fbk_t fbk;
    uint16_t CRC16;
} MotorData_t;


#pragma pack()

// 电机反馈数据结构体
typedef struct
{
    MotorData_t motor_recv_data; // 电机反馈数据结构体
    int hex_len;                 // 接收数据长度，通常为78字节
    long long resv_time;         // 接收时间，单位为微秒（us）
    int correct;                 // 数据校验结果，1表示成功，0表示失败
    unsigned char motor_id;      // 电机ID
    unsigned char mode;          // 电机工作模式
    int Temp;                    // 电机温度
    unsigned char MError;        // 电机错误码
    float T;                     // 当前实际扭矩
    float W;                     // speed 当前实际速度
    float Pos;                   // 当前实际位置
    float footForce;             // 传感器力反馈，单位：12位
} MOTOR_recv;


// 电机配置结构体
typedef struct
{
    UART_HandleTypeDef* huart;     // 串口句柄
    GPIO_TypeDef* rs485_gpio_port; // RS485 控制端口
    uint16_t rs485_de_pin;         // RS485 DE 引脚
    uint16_t rs485_re_pin;         // RS485 RE 引脚
    uint8_t id;                    // 电机 ID (0-14)
    bool reverse;                  // 是否反转
    float reduction_rate;          // 减速比 (输出端/电机端)
} Unitree_Config_t;

// 电机对象结构体
typedef struct UnitreeMotor
{
    Unitree_Config_t config;

    // 控制输入 (物理单位)
    uint8_t mode; // 0:锁定, 1:FOC闭环, 2:校准
    float T;      // 前馈扭矩 (Nm)
    float W;      // 目标速度 (rad/s)
    float Pos;    // 目标位置 (rad)
    float K_P;    // 刚度系数 (0.0 - 1.0)
    float K_W;    // 阻尼系数 (0.0 - 1.0)

    // 反馈输出 (物理单位)
    struct
    {
        float torque;              // 实际扭矩 (Nm)
        float speed;               // 实际速度 (rad/s)
        float pos;                 // 实际位置 (rad)
        int8_t temp;               // 温度 (℃)
        uint8_t error;             // 错误码
        bool connected;            // 连接状态
        uint32_t rx_count;         // 接收计数
        uint32_t error_count_crc;  // 错误计数
        uint32_t error_count_head; // 帧错误计数
    } feedback;


    // 内部缓冲区
    ControlData_t tx_buffer;
    MOTOR_recv rx_buffer;

    // 状态标志
    volatile bool waiting_for_reply; // 标记是否正在等待回复
} UnitreeMotor;

/**
 * @brief 初始化宇树电机
 * @param motor 电机对象指针
 * @param config 配置参数
 */
void Unitree_Init(UnitreeMotor* motor, Unitree_Config_t config);

/**
 * @brief 设置控制指令 (不发送)
 * @param motor 电机对象指针
 * @param mode 模式 (0:锁定, 1:FOC闭环, 2:校准)
 * @param torque 前馈扭矩 (Nm)
 * @param speed 目标速度 (rad/s)
 * @param pos 目标位置 (rad)
 * @param kp 刚度系数 (0.0 - 1.0)
 * @param kw 阻尼系数 (0.0 - 1.0)
 */
void Unitree_SetCmd(UnitreeMotor* motor, uint8_t mode, float torque, float speed, float pos, float kp, float kw);

/**
 * @brief 宏定义别名，符合用户习惯
 */
#define __Unitree_SET_CMD(_MOTOR, _MODE, _TORQUE, _SPEED, _POS, _KP, _KW) \
    Unitree_SetCmd(_MOTOR, _MODE, _TORQUE, _SPEED, _POS, _KP, _KW)

/**
 * @brief 发送控制指令 (通过 UART 句柄查找电机并发送)
 * @param huart 串口句柄
 */
void Unitree_SendCommand(UnitreeMotor* motor);

/**
 * @brief 串口发送完成回调 (需在 HAL_UART_TxCpltCallback 中调用)
 * @param huart 串口句柄
 */
void Unitree_TxCpltCallback(UART_HandleTypeDef* huart);

/**
 * @brief 串口接收事件回调 (需在 HAL_UARTEx_RxEventCallback 中调用)
 * @param huart 串口句柄
 * @param Size 接收到的数据长度
 */
void Unitree_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif // UNITREE_MOTOR_H
