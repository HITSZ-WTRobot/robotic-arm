#include <cstdio>
#include <cstring>
#include "app.h"
#include "cmsis_os2.h"
#include "controller/dual_arm_ctrl.h" // 切换为双臂控制器
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
DJI_t dji_motor_driver_slave; // 新增：小臂从电机
DJI_t dji_gripper;
DJI_t dji_gripper_slave; // 新增：吸盘从电机
UnitreeMotor unitree_motor_driver;
XGZP6847D pressure_sensor(&hi2c1, 200.0f);

// 全局变量定义
Arm::MotorCtrl* joint1_motor         = NULL;
Arm::MotorCtrl* joint2_motor_master  = NULL; // 小臂主
Arm::MotorCtrl* joint2_motor_slave   = NULL; // 小臂从
Arm::MotorCtrl* gripper_motor_master = NULL; // 吸盘主
Arm::MotorCtrl* gripper_motor_slave  = NULL; // 吸盘从

// 控制器实例
Arm::DualArmController* robot_arm = NULL;

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
    // 初始化小臂从电机 (M3508)
    // 假设：对向安装需要反转 (reverse = true)，ID为 7
    DJI_Init(&dji_motor_driver_slave, (DJI_Config_t){
                                          .auto_zero  = false, // 从电机不需要 auto_zero，跟随主电机
                                          .reverse    = true,  // 对向安装通常需要反向
                                          .motor_type = M3508_C620,
                                          .hcan       = &hcan1,
                                          .id1        = 7, // 需修改：根据实际 ID 设置
                                      });

    DJI_Init(&dji_gripper, (DJI_Config_t){
                               .auto_zero  = true,
                               .reverse    = false,
                               .motor_type = M2006_C610,
                               .hcan       = &hcan1,
                               .id1        = 1,
                           });
    // 初始化吸盘从电机 (M2006)
    // 假设：对向安装需要反转 (reverse = true)，ID为 2
    DJI_Init(&dji_gripper_slave, (DJI_Config_t){
                                     .auto_zero  = false,
                                     .reverse    = true, // 对向安装通常需要反向
                                     .motor_type = M2006_C610,
                                     .hcan       = &hcan1,
                                     .id1        = 2, // 需修改：根据实际 ID 设置
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
    // 大臂 (Unitree Go1)
    static Arm::MotorCtrl m1(&unitree_motor_driver, Arm::ControlMode::PositionPD_VelocityFF, 1.0f); // 减速比已经在 Driver 中处理，或者这里填 1.0 如果 MotorCtrl 不处理
    // 注意: 如果 UnitreeMotor Driver 内部已经乘了减速比，这里 MotorCtrl 可能应当设为 1.0
    // 假设 MotorCtrl 是通用层，通常需要传入减速比来转换 轴前馈 -> 电机前馈。
    // 这里先填 1.0，因为 Unitree 的控制协议通常直接是对关节进行控制(如果配置了)，或者对电机进行控制。
    // 之前代码里 Init Unitree 时传入了 reduction_rate = 6.33f。
    // 如果 MotorCtrl 再次乘以 6.33，可能重叠。
    // 但鉴于之前的代码 m2 是 (3591... * ...)，显然 MotorCtrl 负责部分减速比。
    // 让我们保持 consistency: Unitree Driver 可能比较特殊。
    // 稳妥起见，设为 1.0，依靠 DualArmController 的 reduction_1 和 Driver 的 reduction_rate。

    m1.SetCtrlParam((PD_Config_t){
                        .Kp             = 0.0f,
                        .Kd             = 0.0f,
                        .abs_output_max = 20.0f,
                    },
                    (MotorPID_Config_t){
                        .Kp             = 0.0f,
                        .Ki             = 0.0f,
                        .Kd             = 0.0f,
                        .abs_output_max = 20.0f,
                    });
    joint1_motor = &m1;

    // 小臂 Master (DJI)
    static Arm::MotorCtrl m2_master(&dji_motor_driver, Arm::ControlMode::PositionPD_VelocityFF, (3591.0f / (187 * 100)) * (16384.0f / (20.0f * 0.3f)));
    m2_master.SetCtrlParam((PD_Config_t){
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
    joint2_motor_master = &m2_master;

    // 小臂 Slave (DJI)
    static Arm::MotorCtrl m2_slave(&dji_motor_driver_slave, Arm::ControlMode::PositionPD_VelocityFF, (3591.0f / (187 * 100)) * (16384.0f / (20.0f * 0.3f)));
    m2_slave.SetCtrlParam((PD_Config_t){
                              .Kp             = 8.0f,
                              .Kd             = 0.2f,
                              .abs_output_max = 2000.0f,
                          },
                          (MotorPID_Config_t){
                              .Kp             = 45.0f,
                              .Ki             = 0.1f, // 如果 Slave 负载稍轻，可适当减小 Ki
                              .Kd             = 5.0f,
                              .abs_output_max = 5000.0f,
                          });
    joint2_motor_slave = &m2_slave;


    // 吸盘 Master (DJI)
    static Arm::MotorCtrl m3_master(&dji_gripper, Arm::ControlMode::PositionPD_VelocityFF, 10000.0f / (10.0f * 0.18f));
    m3_master.SetCtrlParam((PD_Config_t){
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
    gripper_motor_master = &m3_master;

    // 吸盘 Slave (DJI)
    static Arm::MotorCtrl m3_slave(&dji_gripper_slave, Arm::ControlMode::PositionPD_VelocityFF, 10000.0f / (10.0f * 0.18f));
    m3_slave.SetCtrlParam((PD_Config_t){
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
    gripper_motor_slave = &m3_slave;

    // 检测初始化是否成功
    uint32_t start_tick = HAL_GetTick();
    while (!joint1_motor->isConnected() || !joint2_motor_master->isConnected() || !joint2_motor_slave->isConnected() ||
           !gripper_motor_master->isConnected() || !gripper_motor_slave->isConnected())
    {
        // ... (同上)
        if (HAL_GetTick() - start_tick > 6000)
        {
            Error_Handler(); // 超时处理
        }
        osDelay(10);
    }


    // 3. 配置机械臂参数
    Arm::DualArmController::Config arm_cfg;
    arm_cfg.l1 = 0.346f; // 大臂长 0.3m
    // ... (物理参数保持不变)
    arm_cfg.l2          = 0.382f;
    arm_cfg.l3          = 0.093f;
    arm_cfg.lc1         = 0.171f;
    arm_cfg.lc2         = 0.176f;
    arm_cfg.lc3         = 0.057f;
    arm_cfg.m1          = 1.2243f;
    arm_cfg.m2          = 0.675f;
    arm_cfg.m3          = 0.6764f;
    arm_cfg.g           = 9.81f;
    arm_cfg.reduction_1 = 1.0f;
    arm_cfg.reduction_2 = 100 * 187 * 1.5f / 3591.0f;
    arm_cfg.reduction_3 = 2.7f;
    arm_cfg.offset_1    = 0.0f;
    arm_cfg.offset_2    = 162.0f;
    arm_cfg.offset_3    = -90.0f;

    // 运动学限制
    arm_cfg.j1_max_vel  = 360.0f;
    arm_cfg.j1_max_acc  = 10.0f;
    arm_cfg.j1_max_jerk = 500.0f;
    arm_cfg.j2_max_vel  = 50.0f;
    arm_cfg.j2_max_acc  = 120.0f;
    arm_cfg.j2_max_jerk = 500.0f;
    arm_cfg.j3_max_vel  = 3600.0f;
    arm_cfg.j3_max_acc  = 360.0f;
    arm_cfg.j3_max_jerk = 500.0f;

    // --- 新增: 同步纠偏参数 ---
    // 关键参数:
    // gain = 2.0: 每 1度误差产生 2度/秒的修正速度。既不慢也不至于太猛。
    // deadzone = 0.2: 忽略小于0.2度的误差，防止在死压状态下微震荡。
    arm_cfg.sync_gain_2     = 2.0f;
    arm_cfg.sync_deadzone_2 = 0.2f;

    arm_cfg.sync_gain_3     = 2.0f;
    arm_cfg.sync_deadzone_3 = 0.2f;

    // 4. 实例化并初始化控制器
    static Arm::DualArmController ctrl(*joint1_motor,
                                       *joint2_motor_master, *joint2_motor_slave,
                                       *gripper_motor_master, *gripper_motor_slave,
                                       arm_cfg);
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

float q1, q2, q3;
void MotorCtrl(void* argument)
{
    // 等待系统稳定
    osDelay(4000);

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
