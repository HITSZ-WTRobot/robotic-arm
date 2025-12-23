#include "app.h"
// #include "libs/pid_pd.h"
#include "cmsis_os2.h"
#include "stdio.h"
#include "usart.h"


#ifdef __cplusplus
extern "C" {

UnitreeMotor g_motor;

DJI_t dji_3508;

// 实例化电机
Arm::MotorCtrl Unitree(&g_motor,
                       Arm::ControlMode::PositionPD_VelocityFF, // 控制模式
                       1);

/**
 * 定时器回调函数，用于定时进行 PID 计算和 CAN 指令发送
 * @param htim
 */
void TIM_Callback(TIM_HandleTypeDef* htim)
{
    // Unitree.update();

    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_5_8);
    Unitree_SendCommand(&huart1); // 发送宇树电机命令
}

void Motor_Control_Init()
{

    Unitree.SetCtrlParam((PD_Config_t){
                             .Kp             = 10.0f,
                             .Kd             = 0.3f,
                             .abs_output_max = 2000.0f,
                         },
                         (MotorPID_Config_t){
                             .Kp             = 50.0f,
                             .Ki             = 0.2f,
                             .Kd             = 5.0f,
                             .abs_output_max = 1000.0f,
                         }); // 设置 PID 参数
}

void Init(void* argument)
{
    HAL_UART_RegisterRxEventCallback(&huart1, Unitree_RxEventCallback);
    HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, Unitree_TxCpltCallback);

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);
    Unitree_Init(&g_motor, (Unitree_Config_t){
                               .huart           = &huart1,
                               .rs485_gpio_port = GPIOA,
                               .rs485_de_pin    = GPIO_PIN_8,
                               .id              = 2, // ID 为 0
                               .reduction_rate  = 1.0f,
                           });
    Unitree_SetCmd(&g_motor,
                   1,    // Mode 1: FOC Closed Loop
                   0.0f, // T
                   1.0f, // W
                   0.0f, // Pos
                   0.0f, // K_P
                   0.1f  // K_W
    );
    Unitree_SendCommand(&huart1); // 发送初始命令
    // Motor_Control_Init(); // 初始化电机
    osThreadExit();
}

void MotorCtrl(void* argument)
{
    // 设置宇树电机目标位置

    for (;;)
    {
        // Unitree.setTarget(10, 0.0f);

        // osDelay(1000);

        // Unitree.setTarget(-10, 0.0f);

        // osDelay(1000);
        osDelay(10);
    }
}
}


#endif // __cplusplus
