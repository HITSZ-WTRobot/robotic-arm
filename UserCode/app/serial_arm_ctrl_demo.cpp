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
                                    .id1        = 2,
                                });
    DJI_Init(&dji_gripper, (DJI_Config_t){
                               .auto_zero  = true,
                               .reverse    = false,
                               .motor_type = M2006_C610,
                               .hcan       = &hcan1,
                               .id1        = 1,
                           });

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
    arm_cfg.reduction_3 = 2.7f;                       // 吸盘关节减速比 (M2006)

    // 关节零位偏移 (Degree)
    // 假设上电时大臂垂直地面 (90度)，小臂水平 (0度)
    // 如果电机上电位置为 0，则 offset_1 = 90
    arm_cfg.offset_1 = 0.0f;
    arm_cfg.offset_2 = 162.0f;
    arm_cfg.offset_3 = -90.0f;

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
static const float action_init2get[][3] = {
    {0.000000f, 2.827433f, -1.570796f},
    {0.149272f, 2.708163f, -1.535406f},
    {0.210992f, 2.638466f, -1.462390f},
    {0.351969f, 2.525328f, -1.391390f},
    {0.490123f, 2.409258f, -1.329995f},
    {0.627619f, 2.293327f, -1.273215f},
    {0.763559f, 2.180060f, -1.204847f},
    {0.893702f, 2.058548f, -1.138770f},
    {1.026454f, 1.936687f, -1.076546f},
    {1.157744f, 1.818645f, -1.020221f},
    {1.284473f, 1.693275f, -0.969958f},
    {1.400403f, 1.548976f, -0.882830f},
    {1.595668f, 1.373530f, -0.773428f},
    {1.661732f, 1.314833f, -0.652823f},
    {1.787060f, 1.188164f, -0.564372f},
    {1.892786f, 1.034662f, -0.435481f},
    {2.089038f, 0.888197f, -0.381912f},
    {2.219644f, 0.797044f, -0.314549f},
    {2.381298f, 0.612779f, -0.250119f},
    {2.552461f, 0.489465f, -0.115213f},
    {2.624715f, 0.485858f, 0.000000f},
    {2.766650f, 0.305219f, 0.000000f},
    {2.938274f, 0.160281f, 0.000000f},
    {3.141593f, -0.000000f, 0.000000f}
};
static const int action_init2get_len = sizeof(action_init2get)/sizeof(action_init2get[0]);
// Process2
static const float action_get2put[][3] = {
    {3.141593f, -0.000000f, -1.570796f},
    {3.102457f, -0.095799f, -1.434089f},
    {3.086675f, -0.184754f, -1.277085f},
    {3.055273f, -0.295949f, -1.152601f},
    {3.025892f, -0.389281f, -1.008077f},
    {3.010740f, -0.502128f, -0.874304f},
    {2.989157f, -0.606948f, -0.735549f},
    {2.963501f, -0.749939f, -0.521466f},
    {2.890438f, -0.907487f, -0.352853f},
    {2.858316f, -0.956106f, -0.293872f},
    {2.793328f, -1.120093f, -0.136842f},
    {2.761982f, -1.238270f, 0.075933f},
    {2.773546f, -1.365256f, 0.195676f},
    {2.750610f, -1.535119f, 0.355306f},
    {2.653410f, -1.671205f, 0.588591f},
    {2.756722f, -1.822862f, 0.636937f}
};
static const int action_get2put_len = sizeof(action_get2put)/sizeof(action_get2put[0]);
// Process3
static const float action_put2get[][3] = {
    {2.756722f, -1.822862f, 0.636937f},
    {2.824672f, -1.880269f, 0.629419f},
    {2.958277f, -2.013794f, 0.681043f},
    {2.970324f, -2.104145f, 0.704618f},
    {2.986054f, -2.028544f, 0.671219f},
    {3.005234f, -1.799302f, 0.583837f},
    {3.011206f, -1.720496f, 0.558266f},
    {3.038244f, -1.491268f, 0.481924f},
    {3.031102f, -1.329515f, 0.435469f},
    {3.043924f, -1.187249f, 0.381992f},
    {3.088608f, -0.986776f, 0.295018f},
    {3.070166f, -0.904455f, 0.264346f},
    {3.063479f, -0.704245f, 0.169698f},
    {3.068580f, -0.523316f, 0.104484f},
    {3.027958f, -0.309280f, 0.048032f},
    {3.053214f, -0.240542f, 0.037357f},
    {3.131146f, -0.093789f, 0.022022f},
    {3.141593f, 0.000000f, 0.000000f}
};
static const int action_put2get_len = sizeof(action_put2get)/sizeof(action_put2get[0]);

// 弧度转角度
static const float RAD2DEG = 180.0f / 3.1415926f;

// 播放轨迹 helper
static void Arm_Action(Arm::Controller* arm, const float traj[][3], int len)
{
    if (!arm)
        return;
    for (int i = 0; i < len; ++i)
    {
        arm->setJointTarget(
            traj[i][0] * RAD2DEG,
            traj[i][1] * RAD2DEG,
            traj[i][2] * RAD2DEG);
        while (!arm->isArrived())
        {
            osDelay(10);
        }
    }
}

float q1, q2, q3;
void MotorCtrl(void* argument)
{
    // 等待系统稳定
    osDelay(4000);

    if (robot_arm)
    {
        Arm_Action(robot_arm, action_init2get, action_init2get_len);
        osDelay(5000);
        Arm_Action(robot_arm, action_get2put, action_get2put_len);
        osDelay(5000);
        Arm_Action(robot_arm, action_put2get, action_put2get_len);
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
