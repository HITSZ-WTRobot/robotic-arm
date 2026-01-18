#include <cstdio>
#include <cstring>
#include "app.h"
#include "cmsis_os2.h"
#include "controller/arm_ctrl.h" // 引入控制器头文件
#include "drivers/DJI.h"
#include "drivers/unitree_motor.h"
#include "interfaces/arm_motor_if.h"
#include "libs/pid_motor.h"
#include "stm32f407xx.h"
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
UnitreeMotor unitree_motor_driver;
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
    if (robot_arm != NULL)
    {
        robot_arm->update(CONTROL_PERIOD);
    }

    // 发送 CAN 指令 (DJI)
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_5_8);
    // Unitree 的发送
    Unitree_SendCommand(&unitree_motor_driver);
}

void Init(void* argument)
{

    // 初始化 DJI 驱动
    DJI_CAN_FilterInit(&hcan1, 0);
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    DJI_Init(&dji_motor_driver, (DJI_Config_t){
                                    .auto_zero  = true,
                                    .reverse    = false,
                                    .motor_type = M3508_C620,
                                    .hcan       = &hcan1,
                                    .id1        = 6,
                                });
    DJI_Init(&dji_gripper, (DJI_Config_t){
                               .auto_zero  = true,
                               .reverse    = false,
                               .motor_type = M2006_C610,
                               .hcan       = &hcan1,
                               .id1        = 1,
                           });

    // 检查 DJI 电机连接 (等待反馈)
    // uint32_t start_tick = HAL_GetTick();
    // while (dji_motor_driver.feedback_count == 0 || dji_gripper.feedback_count == 0)
    // {
    //     if (HAL_GetTick() - start_tick > 3000)
    //     {
    //         // 超时处理: 可以在这里报错或死循环
    //         break;
    //     }
    //     osDelay(10);
    // }

    // 注册 UART 回调
    HAL_UART_RegisterRxEventCallback(&huart1, Unitree_RxEventCallback);
    HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, Unitree_TxCpltCallback);

    // 初始化 Unitree 驱动
    Unitree_Init(&unitree_motor_driver, (Unitree_Config_t){
                                            .huart           = &huart1,
                                            .rs485_gpio_port = GPIOA,
                                            .rs485_de_pin    = GPIO_PIN_8,
                                            .id              = 2,     // ID 为 2
                                            .reverse         = true,  // 反转方向
                                            .reduction_rate  = 6.33f, // 减速比 6.33
                                        });
    Unitree_SetCmd(&unitree_motor_driver, 1, 0, 0, 0, 0, 0);
    Unitree_SendCommand(&unitree_motor_driver);
     osDelay(10);
    // if (unitree_motor_driver.feedback.rx_count == 0)
    // {
    //     // 超时处理: 可以在这里报错或死循环
    // }
    // 2. 实例化电机接口
    // 大臂 (Unitree)
    static Arm::MotorCtrl m1(&unitree_motor_driver, Arm::ControlMode::PositionPD_VelocityFF, 1);
    m1.SetCtrlParam((PD_Config_t){
                        .Kp             = 3.0f,
                        .Kd             = 0.1f,
                        .abs_output_max = 5.0f,
                    },
                    (MotorPID_Config_t){
                        .Kp             = 0.02f,
                        .Ki             = 0.00305f, // 积分项
                        .Kd             = 0.0f,     //
                        .abs_output_max = 15.0f,    // 降低一半处理
                    });                             // 设置 PID
    joint1_motor = &m1;

    // 小臂 (DJI)
    static Arm::MotorCtrl m2(&dji_motor_driver, Arm::ControlMode::PositionPD_VelocityFF, (3591.0f / (187 * 100)) * (16384.0f / (20.0f * 0.3f)));
    m2.SetCtrlParam((PD_Config_t){
                        .Kp             = 8.0f,
                        .Kd             = 0.2f,
                        .abs_output_max = 2000.0f,
                    },
                    (MotorPID_Config_t){
                        .Kp             = 45.0f,
                        .Ki             = 0.1f,
                        .Kd             = 5.0f,
                        .abs_output_max = 5000.0f,
                    });
    joint2_motor = &m2;
    // 吸盘 (DJI)

    static Arm::MotorCtrl m3(&dji_gripper, Arm::ControlMode::PositionPD_VelocityFF, 10000.0f / (10.0f * 0.18f));
    m3.SetCtrlParam((PD_Config_t){
                        .Kp             = 1.5f,
                        .Kd             = 5.0f,
                        .abs_output_max = 2000.0f,
                    },
                    (MotorPID_Config_t){
                        .Kp             = 50.0f,
                        .Ki             = 0.7f,
                        .Kd             = 13.0f,
                        .abs_output_max = 5000.0f,
                    });
    gripper_motor = &m3;

    // 检测初始化是否成功
    uint32_t start_tick = HAL_GetTick();
    while (!joint1_motor->isConnected() || !joint2_motor->isConnected() || !gripper_motor->isConnected())
    {
        // 再发送一次命令？
        if (HAL_GetTick() - start_tick > 6000)
        {
            Error_Handler(); // 超时处理
        }
        osDelay(10);
    }


    // 3. 配置机械臂参数
    Arm::Controller::Config arm_cfg;
    arm_cfg.l1          = 0.346f;  // 大臂长 0.3m
    arm_cfg.l2          = 0.382f;  // 小臂长 0.25m
    arm_cfg.l3          = 0.093f;  // 吸盘长 0.25m
    arm_cfg.lc1         = 0.171f;  // 大臂质心
    arm_cfg.lc2         = 0.176f;  // 小臂质心
    arm_cfg.lc3         = 0.057f;  // 吸盘质心 (估算值)
    arm_cfg.m1          = 1.2243f; // 大臂质量 kg
    arm_cfg.m2          = 0.675f;  // 小臂质量 kg
    arm_cfg.m3          = 0.6764f; // 吸盘质量
    arm_cfg.g           = 9.81f;
    arm_cfg.reduction_1 = 1.0f;                       // 大臂减速比 (例如 Unitree Go1 减速比)
    arm_cfg.reduction_2 = 100 * 187 * 1.5f / 3591.0f; // 小臂减速比 (例如 M3508 减速比)
    // arm_cfg.reduction_3 = 1.5f;                       // 吸盘关节减速比 (M2006)
    arm_cfg.reduction_3 = 2.7f; // 吸盘关节减速比 (M2006)
    // 关节零位偏移 (Degree)
    // 假设上电时大臂垂直地面 (90度)，小臂水平 (0度)
    // 如果电机上电位置为 0，则 offset_1 = 90
    arm_cfg.offset_1 = 0.0f;
    arm_cfg.offset_2 = 162.0f;
    arm_cfg.offset_3 = -90.0f;
    // arm_cfg.offset_2 = 90.0f;
    // arm_cfg.offset_3 = 0.0f;

    // 运动学限制 (Degree)
    arm_cfg.j1_max_vel  = 360.0f;
    arm_cfg.j1_max_acc  = 10.0f;
    arm_cfg.j1_max_jerk = 500.0f;
    arm_cfg.j2_max_vel  = 50.0f;
    arm_cfg.j2_max_acc  = 120.0f;
    arm_cfg.j2_max_jerk = 500.0f;
    arm_cfg.j3_max_vel  = 3600.0f;
    arm_cfg.j3_max_acc  = 360.0f;
    arm_cfg.j3_max_jerk = 500.0f;

    // 4. 实例化并初始化控制器
    static Arm::Controller ctrl(*joint1_motor, *joint2_motor, *gripper_motor, arm_cfg);
    robot_arm = &ctrl;
    osDelay(50); // 等待电机反馈更新
    Unitree_SetCmd(&unitree_motor_driver, 1, 0, 0, 0, 0, 0);
    Unitree_SendCommand(&unitree_motor_driver);
    osDelay(2000);
    robot_arm->init(); // 读取当前位置，防止跳变

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);

    osThreadExit();
}


// --- 轨迹数据 ---
// Process1
static const float process1_traj[][3] = {
    {3.141593f, -2.827433f, 0.000000f},
    {3.000361f, -2.688597f, -0.001970f},
    {2.865372f, -2.566211f, 0.001671f},
    {2.728288f, -2.448021f, 0.007646f},
    {2.525032f, -2.274048f, -0.003308f},
    {2.317031f, -2.106882f, 0.001148f},
    {2.184397f, -1.989168f, -0.012160f},
    {2.051005f, -1.871886f, -0.010713f},
    {1.928072f, -1.738172f, -0.028643f},
    {1.732155f, -1.558632f, -0.014999f},
    {1.600856f, -1.442883f, 0.006201f},
    {1.400429f, -1.286767f, 0.001225f},
    {1.275329f, -1.161723f, 0.008580f},
    {1.087409f, -0.988449f, -0.016237f},
    {0.957967f, -0.868879f, -0.020701f},
    {0.776756f, -0.681631f, -0.020544f},
    {0.711219f, -0.624120f, -0.018811f},
    {0.527850f, -0.483824f, -0.010778f},
    {0.398864f, -0.427611f, -0.043028f},
    {0.234325f, -0.273950f, -0.033185f},
    {0.125850f, -0.142338f, -0.053368f},
    {0.000000f, 0.000000f, 0.000000f}
};
static const int process1_len = sizeof(process1_traj)/sizeof(process1_traj[0]);
// Process2
static const float process2_traj[][3] = {
    {0.000000f, 0.000000f, 0.000000f},
    {0.046095f, 0.146695f, 0.002017f},
    {0.073150f, 0.220581f, 0.005211f},
    {0.117630f, 0.357943f, 0.029283f},
    {0.139245f, 0.432597f, 0.042472f},
    {0.182444f, 0.580913f, 0.044316f},
    {0.151884f, 0.791065f, 0.058143f},
    {0.097523f, 1.004226f, 0.136580f},
    {0.165343f, 1.159596f, 0.129640f},
    {0.209266f, 1.293336f, 0.107277f},
    {0.304751f, 1.484513f, 0.080762f},
    {0.390983f, 1.535119f, 0.000000f},
    {0.488183f, 1.671205f, 0.000000f},
    {0.384871f, 1.822862f, 0.000000f}
};
static const int process2_len = sizeof(process2_traj)/sizeof(process2_traj[0]);
// Process3
static const float process3_traj[][3] = {
    {0.384871f, 1.822862f, 0.000000f},
    {0.263114f, 1.981565f, 0.003342f},
    {0.171269f, 2.104145f, 0.000000f},
    {0.174243f, 1.937333f, -0.009588f},
    {0.147297f, 1.778970f, -0.033879f},
    {0.104708f, 1.551748f, -0.064548f},
    {0.054087f, 1.325416f, -0.071155f},
    {0.031988f, 1.170462f, -0.050527f},
    {0.022380f, 0.937288f, -0.036419f},
    {-0.056023f, 0.745292f, -0.022871f},
    {-0.059208f, 0.584665f, -0.026611f},
    {-0.067586f, 0.367524f, -0.059808f},
    {-0.152035f, 0.218699f, -0.038715f},
    {0.000000f, 0.000000f, 0.000000f}
};
static const int process3_len = sizeof(process3_traj)/sizeof(process3_traj[0]);

float q1, q2, q3;
void MotorCtrl(void* argument)
{
    // 等待系统稳定
    osDelay(4000);

    // 自动轨迹播放（弧度转角度）
    const float RAD2DEG = 180.0f / 3.1415926f;
    if (robot_arm) {
        // 播放 Process1
        for (int i = 0; i < process1_len; ++i) {
            robot_arm->setJointTarget(
                process1_traj[i][0] * RAD2DEG,
                process1_traj[i][1] * RAD2DEG,
                process1_traj[i][2] * RAD2DEG);
            osDelay(50); // 50ms 间隔，约20Hz
        }
        // 播放 Process2
        for (int i = 0; i < process2_len; ++i) {
            robot_arm->setJointTarget(
                process2_traj[i][0] * RAD2DEG,
                process2_traj[i][1] * RAD2DEG,
                process2_traj[i][2] * RAD2DEG);
            osDelay(50);
        }
        // 播放 Process3
        for (int i = 0; i < process3_len; ++i) {
            robot_arm->setJointTarget(
                process3_traj[i][0] * RAD2DEG,
                process3_traj[i][1] * RAD2DEG,
                process3_traj[i][2] * RAD2DEG);
            osDelay(50);
        }
    }

    // 播放完毕后，恢复串口控制
    // 开启空闲中断接收 (DMA)
    HAL_UARTEx_ReceiveToIdle_DMA(&UART_PC_HANDLER, uart3_rx_buf, sizeof(uart3_rx_buf));

    char tx_buf[64];
    int vacuum_state = 0;
    for (;;)
    {
        // 1. 处理接收到的数据
        if (uart3_rx_flag)
        {
            // 确保字符串结束
            if (uart3_rx_len < sizeof(uart3_rx_buf))
                uart3_rx_buf[uart3_rx_len] = 0;
            else
                uart3_rx_buf[sizeof(uart3_rx_buf) - 1] = 0;

            // 解析 CSV: 45.5, 30.0, 0.0, 1
            // 注意: sscanf 会自动跳过空白字符，但逗号需要显式匹配
            if (sscanf((char*)uart3_rx_buf, "%f, %f, %f, %d", &q1, &q2, &q3, &vacuum_state) == 4)
            {
                if (robot_arm)
                {
                    robot_arm->setJointTarget(q1, q2, q3);
                }
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, (GPIO_PinState)!vacuum_state);
                // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (GPIO_PinState)vacuum_state);
            }

            uart3_rx_flag = false;
            // 重新开启接收
            HAL_UARTEx_ReceiveToIdle_DMA(&UART_PC_HANDLER, uart3_rx_buf, sizeof(uart3_rx_buf));
        }

        // 2. 发送当前状态反馈
        if (robot_arm)
        {
            float cur_q1, cur_q2, cur_q3;
            robot_arm->getJointAngles(cur_q1, cur_q2, cur_q3);

            // float pressure_kpa = pressure_sensor.readPressure() / 1000.0f;

            float pressure_kpa = 100.0f;
            int len            = sprintf(tx_buf, "%.2f, %.2f, %.2f, %.2f\n", cur_q1, cur_q2, cur_q3, pressure_kpa);

            HAL_UART_Transmit_DMA(&UART_PC_HANDLER, (uint8_t*)tx_buf, len);
        }

        osDelay(50); // 20Hz 刷新率
    }
}

} // extern "C"
#endif
