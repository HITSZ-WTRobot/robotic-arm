#include "app.h"
#include "controller/arm_ctrl.h" // 引入控制器头文件
#include "drivers/DJI.h"
#include "drivers/Unitree_user.h"
#include "interfaces/arm_motor_if.h"


DJI_t dji_motor_driver;
DJI_t dji_gripper;
UnitreeMotor* unitree_motor_driver = NULL;

Arm::Motor* joint1_motor  = NULL;
Arm::Motor* joint2_motor  = NULL;
Arm::Motor* gripper_motor = NULL;

// 控制器实例
Arm::Controller* robot_arm = NULL;

#ifdef __cplusplus
extern "C" {

// 定时器周期 (秒)
static const float CONTROL_PERIOD = 0.001f; // 1ms

void TIM_Callback(TIM_HandleTypeDef* htim)
{
    // 5. 周期性更新控制回路 (1kHz)
    if (robot_arm != NULL)
    {
        robot_arm->update(CONTROL_PERIOD);
    }

    // 发送 CAN 指令 (DJI)
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);

    // Unitree 的发送通常在 update 内部或单独处理，取决于您的驱动实现
    // 如果 Unitree 需要单独发送函数，请在这里调用，例如:
    // Unitree_SendCmd(unitree_motor_driver);
}

void Init(void* argument)
{

    // 初始化 DJI 驱动
    DJI_CAN_FilterInit(&hcan1, 0);
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    DJI_Init(&dji_motor_driver, (DJI_Config_t){
                                    .auto_zero  = false,
                                    .reverse    = true,
                                    .motor_type = M3508_C620,
                                    .hcan       = &hcan1,
                                    .id1        = 3,
                                });
    DJI_Init(&dji_gripper, (DJI_Config_t){
                               .auto_zero  = false,
                               .motor_type = M2006_C610,
                               .hcan       = &hcan1,
                               .id1        = 2,
                           });
    // 初始化 Unitree 驱动
    unitree_motor_driver = Unitree_Create_Motor();
    Unitree_init(unitree_motor_driver, &UART_UNITREE_HANDLER, 1);

    // 2. 实例化电机接口
    // 大臂 (Unitree)
    static Arm::Motor m1(unitree_motor_driver, 1, 5);
    m1.setPID(4.0f, 0.01f, 0.0f, 2000.0f,
              30.0f, 0.001f, 5.0f, 8000.0f); // 设置 PID
    joint1_motor = &m1;

    // 小臂 (DJI)
    static Arm::Motor m2(&dji_motor_driver, 16384.0f / 20.0f, 5); // 假设力矩系数
    m2.setPID(8.0f, 0.0f, 0.2f, 2000.0f,
              45.0f, 0.1f, 5.0f, 8000.0f);
    joint2_motor = &m2;
    // 吸盘 (DJI)

    static Arm::Motor m3(&dji_gripper, 1000.0f, 5);
    m3.setPID(4.0f, 0.0f, 0.1f, 1000.0f,
              20.0f, 0.05f, 2.0f, 4000.0f);
    gripper_motor = &m3;

    // 3. 配置机械臂参数
    Arm::Controller::Config arm_cfg;
    arm_cfg.l1          = 0.3f;  // 大臂长 0.3m
    arm_cfg.l2          = 0.25f; // 小臂长 0.25m
    arm_cfg.lc1         = 0.15f; // 大臂质心
    arm_cfg.lc2         = 0.12f; // 小臂质心
    arm_cfg.m1          = 1.5f;  // 大臂质量 kg
    arm_cfg.m2          = 0.8f;  // 小臂质量 kg
    arm_cfg.m3          = 0.2f;  // 吸盘质量
    arm_cfg.g           = 9.81f;
    arm_cfg.reduction_1 = 1.0f;               // 大臂减速比 (例如 Unitree Go1 减速比)
    arm_cfg.reduction_2 = 5.728209412419938f; // 小臂减速比 (例如 M3508 减速比)
    arm_cfg.reduction_3 = 36.0f;              // 吸盘关节减速比 (M2006)

    // 关节零位偏移 (Degree)
    // 假设上电时大臂垂直地面 (90度)，小臂水平 (0度)
    // 如果电机上电位置为 0，则 offset_1 = 90
    arm_cfg.offset_1 = 0.0f;
    arm_cfg.offset_2 = 10.0f;
    arm_cfg.offset_3 = 0.0f;

    // 运动学限制 (Degree)
    arm_cfg.j1_max_vel  = 5.0f;
    arm_cfg.j1_max_acc  = 5.0f;
    arm_cfg.j1_max_jerk = 500.0f;
    arm_cfg.j2_max_vel  = 5.0f;
    arm_cfg.j2_max_acc  = 5.0f;
    arm_cfg.j2_max_jerk = 500.0f;
    arm_cfg.j3_max_vel  = 5.0f;
    arm_cfg.j3_max_acc  = 5.0f;
    arm_cfg.j3_max_jerk = 500.0f;

    // 4. 实例化并初始化控制器
    static Arm::Controller ctrl(*joint1_motor, *joint2_motor, *gripper_motor, arm_cfg);
    robot_arm = &ctrl;

    robot_arm->init(); // 读取当前位置，防止跳变

    // 启动定时器
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);

    osThreadExit();
}

void MotorCtrl(void* argument)
{
    // 等待系统稳定
    osDelay(1000);

    for (;;)
    {
        // 移动到位置 A: 大臂 45度，小臂 -30度
        if (robot_arm)
        {
            robot_arm->setJointTarget(0.0f, 0.0f, 0.0f);
        }

        // 等待运动完成 (简单延时，或轮询 isArrived)
        osDelay(4000);

        // 移动到位置 B
        if (robot_arm)
        {
            robot_arm->setJointTarget(0.0f, 10.0f, 0.0f);
        }

        osDelay(4000);
    }
}

} // extern "C"
#endif
