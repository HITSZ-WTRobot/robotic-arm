#include "app.h"
#include "cmsis_os2.h"
#include "stdio.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {

UnitreeMotor g_motor;

// 实例化电机，使用 VelocityPID 模式
// 参数: 驱动句柄, 控制模式, 力矩系数(Unitree默认为1.0)
Arm::MotorCtrl Unitree(&g_motor,
                       Arm::ControlMode::VelocityPID,
                       1.0f);

/**
 * 定时器回调函数，用于定时进行 PID 计算和 CAN 指令发送
 * @param htim
 */
void TIM_Callback(TIM_HandleTypeDef* htim)
{
    // 更新控制回路 (计算 PID 输出)
    Unitree.update();

    // 发送宇树电机命令 (通过串口发送)
    Unitree_SendCommand(&g_motor);
}

void Motor_Control_Init()
{
    // 设置速度环 PID 参数
    // Kp, Ki, Kd, MaxOutput
    // 请根据实际电机响应进行调节
    Unitree.SetCtrlParam((MotorPID_Config_t){
        .Kp             = 0.005f,   // 比例系数0.032f
        .Ki             = 1e-5f, // 积分系数0.00005f
        .Kd             = 0.0000f,   // 微分系数0.005
        .abs_output_max = 5.0f,   // 输出限幅 (Nm)
    });
}

void Init(void* argument)
{
    // 注册串口接收回调 (用于接收电机反馈)
    HAL_UART_RegisterRxEventCallback(&huart1, Unitree_RxEventCallback);
    // 注册串口发送完成回调 (用于 RS485 方向切换)
    HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, Unitree_TxCpltCallback);

    // 注册定时器回调并启动 (用于周期性控制)
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);

    // 初始化宇树电机驱动
    Unitree_Init(&g_motor, (Unitree_Config_t){
                               .huart           = &huart1,
                               .rs485_gpio_port = GPIOA,
                               .rs485_de_pin    = GPIO_PIN_8,
                               .id              = 2,     // ID 为 2
                               .reduction_rate  = 6.33f, // 减速比 6.33
                           });

    // 初始化 PID 参数
    Motor_Control_Init();

    // 退出初始化任务
    osThreadExit();
}

// void MotorCtrl(void* argument)
// {
//     for (;;)
//     {
//         // 设置目标速度 (单位: degree/s)
//         // 360 degree/s = 60 RPM
//         Unitree.setTarget(3.6f);

//         osDelay(5000);

//         // 停止
//         Unitree.setTarget(0.0f);

//         osDelay(5000);


//     }
// }

    float current_vel         = 0.0f;

void MotorCtrl(void* argument)
{
    const float ramp_step     = 3.0f;
    const uint32_t ramp_delay = 2;

    for (;;)
    {
        // 设置目标速度 (单位: degree/s)
        // 360 degree/s = 60 RPM（10倍关系--------------3.6 ~ 36degree/s
        // 平滑加速
        // while (current_vel < 180.0f)
        // {
        //     current_vel += ramp_step;
        //     if (current_vel > 180.0f)
        //         current_vel = 180.0f;
            Unitree.setTarget(current_vel);
        //     osDelay(ramp_delay);
        // }

        osDelay(1);

        // // 平滑减速停止
        // while (current_vel > 0.0f)
        // {
        //     current_vel -= ramp_step;
        //     if (current_vel < 0.0f)
        //         current_vel = 0.0f;
        //     Unitree.setTarget(current_vel);
        //     osDelay(ramp_delay);
        // }

        // osDelay(2000);
    }
}
}

#endif
