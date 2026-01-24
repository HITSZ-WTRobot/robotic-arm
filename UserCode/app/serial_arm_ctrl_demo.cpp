#include <cstdio>
#include <cstring>
#include "app.h"
#include "cmsis_os2.h"
#include "controller/arm_ctrl.h" // 引入控制器头文件
#include "drivers/DJI.h"
#include "drivers/DM.h" // 引入达妙驱动
#include "drivers/unitree_motor.h"
#include "interfaces/arm_motor_mit.h" // 使用新的 MIT 接口
#include "libs/pid_motor.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "trajectory.h"

#define UART_PC_HANDLER huart1

extern UART_HandleTypeDef UART_PC_HANDLER;
// extern UART_HandleTypeDef huart1;
static uint8_t uart3_rx_buf[64];
static volatile bool uart3_rx_flag    = false;
static volatile uint16_t uart3_rx_len = 0;
static volatile bool is_animating     = false;
static volatile int requested_action  = -1; // -1: none, >=0: action id


DJI_t dji_motor_driver;
DJI_t dji_gripper;
// UnitreeMotor unitree_motor_driver;
DM_t dm_motor_driver; // 替换为达妙电机

XGZP6847D pressure_sensor(&hi2c1, 200.0f);

Arm::MotorCtrl* joint1_motor  = NULL;
Arm::MotorCtrl* joint2_motor  = NULL;
Arm::MotorCtrl* gripper_motor = NULL;

// 控制器实例
Arm::Controller* robot_arm = NULL;

// 定义在全局，方便调试器观测
static const float RATIO_2 = (3591.0f / (187 * 100)) * (16384.0f / (20.0f * 0.3f));
static const float RATIO_3 = 16384.0f / (20.0f * 0.3f);

static Arm::MotorCtrl m1(&dm_motor_driver);
static Arm::MotorCtrl m2(&dji_motor_driver, RATIO_2);
static Arm::MotorCtrl m3(&dji_gripper, RATIO_3);

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
}

void StatusFeedbackTask(void* argument)
{
    char tx_buf[64];
    // pressure_sensor is global

    osDelay(4000);

    // 开启空闲中断接收 (DMA)
    HAL_UARTEx_ReceiveToIdle_DMA(&UART_PC_HANDLER, uart3_rx_buf, sizeof(uart3_rx_buf));

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

            float cmd_q1, cmd_q2, cmd_q3;
            int vacuum_state  = 0;
            int payload_state = 0;
            int action_id     = -1;
            int args_count    = 0;

            if (sscanf((char*)uart3_rx_buf, "ACT, %d", &action_id) == 1)
            {
                if (!is_animating)
                {
                    requested_action = action_id;
                }
            }
            // 解析 CSV: 45.5, 30.0, 0.0, 1, 1 (可选 payload)
            // 注意: sscanf 会自动跳过空白字符，但逗号需要显式匹配
            else if ((args_count = sscanf((char*)uart3_rx_buf, "%f, %f, %f, %d, %d", &cmd_q1, &cmd_q2, &cmd_q3, &vacuum_state, &payload_state)) >= 4)
            {
                if (robot_arm)
                {
                    // 若不在播放动画，则允许关节控制
                    if (!is_animating)
                    {
                        robot_arm->setJointTarget(cmd_q1, cmd_q2, cmd_q3);
                    }

                    // 如果只收到4个参数，默认payload跟随vacuum (兼容旧协议)
                    if (args_count == 4)
                        payload_state = vacuum_state;

                    if (payload_state)
                        robot_arm->setPayload(0.6, 0.5);
                    else
                        robot_arm->setPayload(0.0, 0.0);
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (GPIO_PinState)!vacuum_state);
                }
            }

            uart3_rx_flag = false;
            // 重新开启接收
            HAL_UARTEx_ReceiveToIdle_DMA(&UART_PC_HANDLER, uart3_rx_buf, sizeof(uart3_rx_buf));
        }

        if (robot_arm)
        {
            float cur_q1, cur_q2, cur_q3;
            robot_arm->getJointAngles(cur_q1, cur_q2, cur_q3);

            // float pressure_kpa = pressure_sensor.readPressure() / 1000.0f;
            float pressure_kpa = 100.0f;
            int len            = sprintf(tx_buf, "%.2f, %.2f, %.2f, %.2f, %d\n", cur_q1, cur_q2, cur_q3, pressure_kpa, is_animating ? 1 : 0);

            // 使用 UART_PC_HANDLER 发送，注意这里不需要互斥，因为只有本任务在使用 TX
            HAL_UART_Transmit_DMA(&UART_PC_HANDLER, (uint8_t*)tx_buf, len);
        }
        osDelay(50); // 20Hz 刷新率
    }
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
                                    .id1        = 1,
                                });
    DJI_Init(&dji_gripper, (DJI_Config_t){
                               .auto_zero  = true,
                               .reverse    = true,
                               .motor_type = M3508_C620,
                               .hcan       = &hcan1,
                               .id1        = 2,
                           });
    // 初始化达妙驱动 (DM-J10010L, CAN2)
    DM_CAN_FilterInit(&hcan2, 14);
    HAL_CAN_RegisterCallback(&hcan2, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DM_CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    DM_Config_t config = {
        .hcan        = &hcan2,
        .id0         = 1,
        .POS_MAX_RAD = 12.5f,
        .VEL_MAX_RAD = 25.0f,
        .T_MAX       = 200.0f,
        .mode        = DM_MODE_MIT,
        .motor_type  = DM_J10010L,
        .reverse     = true,
    };
    DM_Init(&dm_motor_driver, &config);
    // 使能电机 (进入 MIT 模式)
    // DM_MIT_SendSetCmd(&dm_motor_driver, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    // osDelay(100);

    // 为关节1 使用达妙电机, MIT 模式
    // MIT 参数: Kp, Kd, Ki, I_limit
    // 初始参数: Kp=30, Kd=1.0, Ki=5.0 (积分项用于消除稳态误差), I_lim=2.0
    // static Arm::MotorCtrl m1(&dm_motor_driver); // Moved to global
    m1.SetMitParams(150.0f, 3.5f, 0.0f, 2.0f);
    joint1_motor = &m1;

    // 小臂 (DJI)
    // 转换 PD(Kp=8, Kd=0.2) + PID_Vel(Kp=45, Ki=0.1) -> MIT Impendance (Physical Nm)
    // Torque Ratio = IQ / Nm
    // float ratio2 = (3591.0f / (187 * 100)) * (16384.0f / (20.0f * 0.3f)); // Moved to global
    // Old IQ Gains:
    // Kp_iq = 45 * 8 = 360
    // Kd_iq = 45 * (1 + 0.2) = 54
    // Ki_iq = 0.1 * 8 = 0.8 (Integral of Pos Error)
    // New Nm Gains = Gain_iq / ratio
    // static Arm::MotorCtrl m2(&dji_motor_driver, RATIO_2); // Moved to global
    m2.SetMitParams(0.8f, 0.03f, 0.01f, 1.0f); // I_limit updated directly to Nm ~2000/ratio? No, keep conservative 5Nm
    joint2_motor = &m2;
    // 吸盘 (DJI)

    // 转换 PD(Kp=1.5, Kd=5) + PID_Vel(Kp=50, Ki=0.7) -> MIT Impendance
    // Torque Ratio
    // float ratio3 = 16384.0f / (20.0f * 0.3f); // Moved to global
    // static Arm::MotorCtrl m3(&dji_gripper, RATIO_3); // Moved to global
    m3.SetMitParams(0.6f, 0.03f, 0.01f, 2.0f); // Limit 2Nm
    gripper_motor = &m3;

    // 检测初始化是否成功
    // uint32_t start_tick = HAL_GetTick();
    // while (!joint1_motor->isConnected() || !joint2_motor->isConnected() || !gripper_motor->isConnected())
    // {
    //     // 再发送一次命令？
    //     if (HAL_GetTick() - start_tick > 6000)
    //     {
    //         Error_Handler(); // 超时处理
    //     }
    //     osDelay(10);
    // }


    // 3. 配置机械臂参数
    Arm::Controller::Config arm_cfg;
    arm_cfg.l1          = 0.346f;   // 大臂长 0.3m
    arm_cfg.l2          = 0.382f;   // 小臂长 0.25m
    arm_cfg.l3          = 0.093f;   // 吸盘长 0.25m
    arm_cfg.lc1         = 0.171f;   // 大臂质心
    arm_cfg.lc2         = 0.23769f; // 小臂质心
    arm_cfg.lc3         = 0.057f;   // 吸盘质心 (估算值)
    arm_cfg.m1          = 1.2243f;  // 大臂质量 kg
    arm_cfg.m2          = 0.909f;   // 小臂质量 kg
    arm_cfg.m3          = 0.6764f;  // 吸盘质量
    arm_cfg.g           = 9.81f;
    arm_cfg.reduction_1 = 1.0f;                       // 大臂减速比 (例如 Unitree Go1 减速比)
    arm_cfg.reduction_2 = 100 * 187 * 1.5f / 3591.0f; // 小臂减速比 (例如 M3508 减速比)
    arm_cfg.reduction_3 = 1.5f;                       // 吸盘关节减速比 (M2006)

    // 关节零位偏移 (Degree)
    // 假设上电时大臂垂直地面 (90度)，小臂水平 (0度)
    // 如果电机上电位置为 0，则 offset_1 = 90
    arm_cfg.offset_1 = 0.0f;
    arm_cfg.offset_2 = -164.0f;
    arm_cfg.offset_3 = 90.0f;

    // 回程差 (Backlash) 补偿 (Degree)
    arm_cfg.backlash_1 = 0.0f; // 直驱电机通常无间隙
    arm_cfg.backlash_2 = 6.0f; // 减速组会有一定间隙
    arm_cfg.backlash_3 = 3.0f;

    // 运动学限制 (Degree)
    arm_cfg.j1_max_vel  = 50.0f;
    arm_cfg.j1_max_acc  = 50.0f;
    arm_cfg.j1_max_jerk = 500.0f;
    arm_cfg.j2_max_vel  = 20.0f;
    arm_cfg.j2_max_acc  = 120.0f;
    arm_cfg.j2_max_jerk = 500.0f;
    arm_cfg.j3_max_vel  = 3600.0f;
    arm_cfg.j3_max_acc  = 30.0f;
    arm_cfg.j3_max_jerk = 500.0f;

    // 4. 实例化并初始化控制器
    static Arm::Controller ctrl(*joint1_motor, *joint2_motor, *gripper_motor, arm_cfg);
    robot_arm = &ctrl;
    osDelay(2000);
    robot_arm->init(); // 读取当前位置，防止跳变

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);

    osThreadExit();
}


float q1, q2, q3;
void MotorCtrl(void* argument)
{
    // 等待系统稳定
    osDelay(4000);

    /*
    if (robot_arm)
    {
        is_animating = true;
        Arm_PlayAction(robot_arm, TRAJ_INIT2GET);
        osDelay(5000);
        is_animating = false;
        // Arm_PlayAction(robot_arm, TRAJ_GET2PUT);
        // osDelay(5000);
        // Arm_PlayAction(robot_arm, TRAJ_PUT2GET);
    }
    */

    for (;;)
    {
        if (requested_action >= 0 && robot_arm)
        {
            if (requested_action < ACTION_TOTAL_COUNT)
            {
                is_animating = true;
                Arm_PlayAction(robot_arm, requested_action);
                is_animating = false;
            }
            else
            {
                // Invalid action ID
            }
            requested_action = -1;
        }
        osDelay(100);
    }
}

} // extern "C"
#endif
