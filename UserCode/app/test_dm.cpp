#include "app.h"
#include "bsp/can_driver.h"
#include "can.h"
#include "cmsis_os2.h"
#include "controller/arm_ctrl.h" // 引入控制器头文件
#include "drivers/DJI.h"
#include "drivers/DM.h"
#include "drivers/unitree_motor.h"
#include "interfaces/arm_motor_if.h"
#include "libs/pid_motor.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"


#define UART_PC_HANDLER huart5

extern UART_HandleTypeDef UART_PC_HANDLER;
extern UART_HandleTypeDef huart1;
static uint8_t uart3_rx_buf[64];
static volatile bool uart3_rx_flag    = false;
static volatile uint16_t uart3_rx_len = 0;


DJI_t dji_motor_driver;
DJI_t dji_gripper;
DM_t dm_motor1;
XGZP6847D pressure_sensor(&hi2c1, 200.0f);

Arm::MotorCtrl* joint1_motor  = NULL;
Arm::MotorCtrl* joint2_motor  = NULL;
Arm::MotorCtrl* gripper_motor = NULL;

// 控制器实例
Arm::Controller* robot_arm = NULL;

#ifdef __cplusplus
extern "C" {

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart->Instance == UART_PC_HANDLER.Instance)
    {
        uart3_rx_len  = Size;
        uart3_rx_flag = true;
    }
}

// 定时器周期 (秒)
static const float CONTROL_PERIOD = 0.001f; // 1ms

void TIM_Callback(TIM_HandleTypeDef* htim)
{
    // 周期性更新控制回路 (1kHz)

    // 发送 CAN 指令 (DJI)
    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_5_8);
    // DM 的发送
    // DM_Vel_SendSetCmd(DM_t *hdm, const float value_vel)
}

void Init(void* argument)
{
    DM_CAN_FilterInit(&hcan2, 14);
    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DM_CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    DM_Config_t config = {
        .hcan        = &hcan2,
        .id0         = 1,
        .POS_MAX_RAD = 3.1416f,
        .VEL_MAX_RAD = 40.0f * 3.1416f / 180.0f,
        .T_MAX       = 2.0f,
        .mode        = DM_MODE_VEL,
        .motor_type  = DM_J10010L,
    };
    DM_Init(&dm_motor1, &config);
    // 启动定时器
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);

    osThreadExit();
}

void MotorCtrl(void* argument)
{
    // 等待系统稳定
    osDelay(4000);
    for (;;)
    {
        DM_Vel_SendSetCmd(&dm_motor1, 1.0f);
        osDelay(5000);
        DM_Vel_SendSetCmd(&dm_motor1, -1.0f);
        osDelay(5000);
    }
}

} // extern "C"
#endif
