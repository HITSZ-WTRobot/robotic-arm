#include "app.h"


#include "stdio.h"

#ifdef __cplusplus
extern "C" {

UnitreeMotor* g_motor = NULL;

DJI_t dji_3508;

// 实例化电机
Arm::Motor DJI(&dji_3508, 1000.0f, 5); // DJI: 需传力矩系数
Arm::Motor Unitree(g_motor, 1, 5);     // Unitree: 需传ID

/**
 * 定时器回调函数，用于定时进行 PID 计算和 CAN 指令发送
 * @param htim
 */
void TIM_Callback(TIM_HandleTypeDef* htim)
{
    DJI.update();
    Unitree.update();

    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_5_8);
}

void Motor_Control_Init()
{

    DJI_CAN_FilterInit(&hcan1, 0);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    // HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, DJI_CAN_Fifo1ReceiveCallback);

    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

    DJI_Init(&dji_3508, (DJI_Config_t){
                            .auto_zero  = false,      //< 是否在启动时自动清零角度
                            .motor_type = M3508_C620, //< 电机类型
                            .hcan       = &hcan1,     //< 电机挂载在的 CAN 句柄
                            .id1        = 1,          //< 电调 ID (1~8)
                        });


    DJI.setPID(4.0f, 0.01f, 0.00f, 1000.0f,        // 位置环 PID 参数
               30.0f, 0.001f, 5.00f, 8000.0f);     // 速度环 PID 参数
    Unitree.setPID(4.0f, 0.01f, 0.00f, 10.0f,      // 位置环 PID 参数
                   30.0f, 0.001f, 5.00f, 127.99f); // 速度环 PID 参数

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);
}

void Init(void* argument)
{
    g_motor = Unitree_Create_Motor();
    if (Unitree_init(g_motor, &UART_UNITREE_HANDLER, ID) != HAL_OK)
    {
        Error_Handler();
    }

    Motor_Control_Init(); // 初始化电机
    osThreadExit();
}

void MotorCtrl(void* argument)
{
    // 设置宇树电机目标位置

    for (;;)
    {
        Unitree.setTarget(10, 0.0f);

        osDelay(1000);

        Unitree.setTarget(-10, 0.0f);

        osDelay(1000);
    }
}
}


#endif // __cplusplus
