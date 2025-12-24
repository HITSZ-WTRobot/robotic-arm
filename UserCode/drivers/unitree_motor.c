/**
 * @file    unitree_motor.c
 * @author  Copilot
 * @date    2025-12-19
 * @version 1.3.0
 * @brief   宇树电机 GO-M8010-6 驱动实现 (仿 DJI 驱动风格)
 */

#include "unitree_motor.h"
#include <string.h>
#include "libs\crc_ccitt.h"

// 全局单电机实例指针
static UnitreeMotor* g_motor = NULL;

// 辅助宏：限制范围
#define SATURATE(_IN, _MIN, _MAX) \
    {                             \
        if (_IN < _MIN)           \
            _IN = _MIN;           \
        else if (_IN > _MAX)      \
            _IN = _MAX;           \
    }

/**
 * @brief 切换 RS485 为发送模式
 */
static inline void RS485_Set_TX(UnitreeMotor* motor)
{
    if (motor->config.rs485_gpio_port)
    {
        // 高电平发送
        HAL_GPIO_WritePin(motor->config.rs485_gpio_port, motor->config.rs485_de_pin, GPIO_PIN_SET);
    }
}

/**
 * @brief 切换 RS485 为接收模式
 */
static inline void RS485_Set_RX(UnitreeMotor* motor)
{
    if (motor->config.rs485_gpio_port)
    {
        // 低电平接收
        HAL_GPIO_WritePin(motor->config.rs485_gpio_port, motor->config.rs485_de_pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief 数据打包
 */
static void Unitree_PackData(UnitreeMotor* motor)
{
    ControlData_t* tx = &motor->tx_buffer;

    tx->head[0] = 0xFE;
    tx->head[1] = 0xEE;

    SATURATE(motor->config.id, 0, 15);
    SATURATE(motor->mode, 0, 7);
    SATURATE(motor->K_P, 0.0f, 25.599f);
    SATURATE(motor->K_W, 0.0f, 25.599f);
    SATURATE(motor->T, -127.99f, 127.99f);
    SATURATE(motor->W, -804.00f, 804.00f);
    SATURATE(motor->Pos, -411774.0f, 411774.0f);

    tx->mode.id     = motor->config.id;
    tx->mode.status = motor->mode;
    tx->mode.none   = 0;

    float kp_sat = motor->K_P;
    float kw_sat = motor->K_W;
    // 应用减速比和反转配置
    float ratio = motor->config.reduction_rate;
    float dir   = motor->config.reverse ? -1.0f : 1.0f;

    tx->comd.tor_des = (int16_t)(motor->T * dir / ratio * UNITREE_TOR_SCALE);
    tx->comd.spd_des = (int16_t)(motor->W * dir * ratio * UNITREE_SPD_SCALE);
    tx->comd.pos_des = (int32_t)(motor->Pos * dir * ratio * UNITREE_POS_SCALE);
    tx->comd.k_pos   = (uint16_t)(kp_sat * UNITREE_KP_SCALE);
    tx->comd.k_spd   = (uint16_t)(kw_sat * UNITREE_KW_SCALE);

    tx->CRC16 = crc_ccitt(0, (uint8_t*)tx, 15);
}

/**
 * @brief 数据解包
 */
static void Unitree_UnpackData(UnitreeMotor* motor)
{
    MOTOR_recv* motor_r = &motor->recv;

    // 校验包头
    if (motor_r->motor_recv_data.head[0] != 0xFD || motor_r->motor_recv_data.head[1] != 0xEE)
    {
        motor_r->correct = 0;
        return;
    }

    // 校验 CRC
    if (motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t*)&motor_r->motor_recv_data, 14))
    {
        motor_r->correct = 0;
        return;
    }

    motor_r->motor_id = motor_r->motor_recv_data.mode.id;
    motor_r->mode     = motor_r->motor_recv_data.mode.status;
    motor_r->Temp     = motor_r->motor_recv_data.fbk.temp;
    motor_r->MError   = motor_r->motor_recv_data.fbk.MError;

    // 应用减速比和反转
    float ratio = motor->config.reduction_rate;
    float dir   = motor->config.reverse ? -1.0f : 1.0f;

    motor_r->T   = (float)motor_r->motor_recv_data.fbk.torque / UNITREE_TOR_SCALE * ratio * dir;
    motor_r->W   = (float)motor_r->motor_recv_data.fbk.speed / UNITREE_SPD_SCALE / ratio * dir;
    motor_r->Pos = (float)motor_r->motor_recv_data.fbk.pos / UNITREE_POS_SCALE / ratio * dir;

    motor_r->footForce = (float)motor_r->motor_recv_data.fbk.force;
    motor_r->correct   = 1;
    motor_r->total_count++;
}

void Unitree_Init(UnitreeMotor* motor, Unitree_Config_t config)
{
    if (!motor || !config.huart)
        return;

    memset(motor, 0, sizeof(UnitreeMotor));
    motor->config = config;

    // 默认参数
    if (motor->config.reduction_rate == 0.0f)
    {
        motor->config.reduction_rate = 6.33f;
    }

    // 注册为全局单实例
    g_motor = motor;
}

void Unitree_SetCmd(UnitreeMotor* motor, uint8_t mode, float torque, float speed, float pos, float kp, float kw)
{
    if (!motor)
        return;

    motor->mode = mode;
    motor->T    = torque;
    motor->W    = speed;
    motor->Pos  = pos;
    motor->K_P  = kp;
    motor->K_W  = kw;

    // 预先打包数据，准备发送
    Unitree_PackData(motor);
}

void Unitree_SendCommand(UART_HandleTypeDef* huart)
{
    // 检查全局电机实例是否匹配该串口
    if (!g_motor || g_motor->config.huart != huart)
    {
        return;
    }

    // 检查是否正在等待回复 (避免重入或冲突)
    if (g_motor->waiting_for_reply)
    {
        return;
    }

    // 切换发送模式
    RS485_Set_TX(g_motor);

    // 标记为等待回复
    g_motor->waiting_for_reply = true;

    // 启动 DMA 发送
    // 注意：必须在发送完成中断(TxCplt)中切换回接收模式
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(g_motor->config.huart, (uint8_t*)&g_motor->tx_buffer, sizeof(ControlData_t));

    if (status != HAL_OK)
    {
        g_motor->waiting_for_reply = false; // 发送失败，清除标志
        RS485_Set_RX(g_motor);              // 恢复接收模式以防万一
    }
}

void Unitree_TxCpltCallback(UART_HandleTypeDef* huart)
{
    // 仅处理注册的全局电机实例
    if (g_motor && g_motor->config.huart == huart)
    {
        // 发送完成，立即切换为接收模式
        RS485_Set_RX(g_motor);

        // 启动 DMA 接收 (使用 Idle 中断)
        // 注意：这里启动接收，等待电机回复
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)&g_motor->recv.motor_recv_data, sizeof(MotorData_t));
    }
}

void Unitree_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    // 仅处理注册的全局电机实例
    if (g_motor && g_motor->config.huart == huart && g_motor->waiting_for_reply)
    {
        // 记录接收到的长度
        g_motor->recv.hex_len = Size;

        // 检查数据长度是否符合预期
        // if (Size == sizeof(MotorData_t))
        // {
        Unitree_UnpackData(g_motor);
        // }
        // 清除等待标志，释放总线
        g_motor->waiting_for_reply = false;
    }
}
