#include "arm_motor_if.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float DEG_TO_RAD  = M_PI / 180.0f;
static constexpr float RPM_TO_RADS = 2.0f * M_PI / 60.0f;

namespace Arm
{

    // ============================================================================
    // Motor Implementation
    // ============================================================================

    // DJI Constructor
    Motor::Motor(DJI_t* driver, float torque_ratio, uint32_t pos_vel_ratio) :
        type_(MotorType::DJI), driver_(driver), id_(0),
        torque_ratio_(torque_ratio), pos_vel_ratio_(pos_vel_ratio), update_count_(0)
    {
        std::memset(&pos_pid_, 0, sizeof(MotorPID_t));
        std::memset(&vel_pid_, 0, sizeof(MotorPID_t));
        if (pos_vel_ratio_ == 0)
            pos_vel_ratio_ = 1;
    }

    // Unitree Constructor
    Motor::Motor(::UnitreeMotor* driver, uint8_t id, uint32_t pos_vel_ratio) :
        type_(MotorType::Unitree), driver_(driver), id_(id),
        torque_ratio_(1.0f), pos_vel_ratio_(pos_vel_ratio), update_count_(0)
    {
        std::memset(&pos_pid_, 0, sizeof(MotorPID_t));
        std::memset(&vel_pid_, 0, sizeof(MotorPID_t));
        if (pos_vel_ratio_ == 0)
            pos_vel_ratio_ = 1;
    }

    void Motor::setPID(float pos_kp, float pos_ki, float pos_kd, float pos_max_out,
                       float vel_kp, float vel_ki, float vel_kd, float vel_max_out)
    {
        MotorPID_Config_t pos_config;
        pos_config.Kp             = pos_kp;
        pos_config.Ki             = pos_ki;
        pos_config.Kd             = pos_kd;
        pos_config.abs_output_max = pos_max_out;
        MotorPID_Init(&pos_pid_, pos_config);

        MotorPID_Config_t vel_config;
        vel_config.Kp             = vel_kp;
        vel_config.Ki             = vel_ki;
        vel_config.Kd             = vel_kd;
        vel_config.abs_output_max = vel_max_out;
        MotorPID_Init(&vel_pid_, vel_config);
    }

    void Motor::setTarget(float angle, float torque_ff)
    {
        target_angle_ = angle;
        feedforward_  = torque_ff;
    }

    void Motor::update()
    {
        // 1. 更新反馈数据
        updateFeedback();

        // 2. 位置环计算 (降频)
        update_count_++;
        if (update_count_ >= pos_vel_ratio_)
        {
            pos_pid_.ref = target_angle_;
            pos_pid_.fdb = current_angle_;
            MotorPID_Calculate(&pos_pid_);
            update_count_ = 0;
        }

        // 3. 速度环计算 (目标速度 = 位置环输出)
        vel_pid_.ref = pos_pid_.output;
        vel_pid_.fdb = current_velocity_;
        MotorPID_Calculate(&vel_pid_);

        // 4. 合成最终输出 (速度环输出 + 前馈力矩)
        // torque_ratio_ 用于将 Nm 转换为电机控制单位 (DJI: IQ, Unitree: Nm)
        float ff_output    = feedforward_ * torque_ratio_;
        float final_output = vel_pid_.output + ff_output;

        // 5. 发送控制指令
        outputControl(final_output);
    }

    void Motor::updateFeedback()
    {
        if (!driver_)
            return;

        if (type_ == MotorType::DJI)
        {
            DJI_t* dji        = static_cast<DJI_t*>(driver_);
            current_angle_    = dji->abs_angle * DEG_TO_RAD;
            current_velocity_ = dji->velocity * RPM_TO_RADS;
        }
        else if (type_ == MotorType::Unitree)
        {
            ::UnitreeMotor* unitree = static_cast<::UnitreeMotor*>(driver_);
            current_angle_          = unitree->m_data.Pos;
            current_velocity_       = unitree->m_data.W;
        }
    }

    void Motor::outputControl(float output)
    {
        if (!driver_)
            return;

        if (type_ == MotorType::DJI)
        {
            DJI_t* dji = static_cast<DJI_t*>(driver_);
            __DJI_SET_IQ_CMD(dji, output);
        }
        else if (type_ == MotorType::Unitree)
        {
            ::UnitreeMotor* unitree = static_cast<::UnitreeMotor*>(driver_);
            Unitree_UART_tranANDrev(unitree,
                                    id_,
                                    10,            // Mode 10: FOC Closed Loop
                                    output,        // T (Calculated Torque)
                                    0.0f,          // W (Target Velocity)
                                    target_angle_, // Pos (Target Position)
                                    0.0f,          // K_P (Stiffness = 0)
                                    0.0f           // K_W (Damping = 0)
            );
        }
    }

} // namespace Arm
