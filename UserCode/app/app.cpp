#include "app.h"
#ifdef __cplusplus
extern "C" {

UnitreeMotor *g_motor = NULL;

float target_angle = 0.0f;

// 使用时需要修改为对应的电机ID，串口句柄，kp，kd参数，与RE，DE引脚设置（在Unitree_user.h）

// 前馈力矩计算
// y = 0.002 * x^3 - 0.0496 * x^2 + 0.0611 * x + 1.8155
// y = ((0.002 * x - 0.0496) * x + 0.0611) * x + 1.8155
float tau_Cal(float angle)
{
    float tau = 0.002f;
    tau       = tau * angle - 0.0496f;
    tau       = tau * angle + 0.0611f;
    tau       = tau * angle + 1.8155f;
    return tau;
}

// 电机初始化任务

void Init(void *argument)
{
    g_motor = Unitree_Create_Motor();
    if (Unitree_init(g_motor, &UART_UNITREE_HANDLER, ID) != HAL_OK) {
        Error_Handler();
    }
    osThreadExit();
}

void MotorCtrl(void *argument)
{
    for (;;) {
        Unitree_UART_tranANDrev(g_motor,
                                ID,
                                1,                     // mode
                                tau_Cal(target_angle), // tau
                                1.0f,                  // W
                                0.0f,                  // Pos
                                0.0f,                  // K_P
                                0.1f                   // K_W
        );
        osDelayUntil(10);
    }
}
}
#endif // __cplusplus