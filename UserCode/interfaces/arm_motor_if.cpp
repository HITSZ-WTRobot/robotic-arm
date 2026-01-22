#include "arm_motor_if.h"
#include <cmath>
#include <cstdint>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float DEG_TO_RAD  = M_PI / 180.0f;
static constexpr float RAD_TO_DEG  = 180.0f / M_PI;
static constexpr float RPM_TO_DEGS = 6.0f; // 360 / 60

namespace Arm
{

    // ============================================================================
    // MotorCtrl Implementation
    // ============================================================================

    // DJI Constructor
    MotorCtrl::MotorCtrl(DJI_t* driver, ControlMode mode, float torque_ratio) :
        type_(MotorType::DJI), mode_(mode), enable_(true), driver_(driver),
        torque_ratio_(torque_ratio), pos_vel_ratio_(1), update_count_(0)
    {
        // pos_pid_ and pos_pd_ share memory in union, so clearing the larger one is enough
        std::memset(&pos_pid_, 0, sizeof(MotorPID_t));
        std::memset(&vel_pid_, 0, sizeof(MotorPID_t));
    }

    // Unitree Constructor
    MotorCtrl::MotorCtrl(::UnitreeMotor* driver, ControlMode mode, float torque_ratio) :
        type_(MotorType::Unitree), mode_(mode), enable_(true), driver_(driver),
        torque_ratio_(torque_ratio), pos_vel_ratio_(1), update_count_(0)
    {
        std::memset(&pos_pid_, 0, sizeof(MotorPID_t));
        std::memset(&vel_pid_, 0, sizeof(MotorPID_t));
    }

    // DM Constructor
    MotorCtrl::MotorCtrl(DM_t* driver, ControlMode mode) :
        type_(MotorType::DM), mode_(mode), enable_(true), driver_(driver),
        torque_ratio_(1.0f), pos_vel_ratio_(1), update_count_(0)
    {
        std::memset(&pos_pid_, 0, sizeof(MotorPID_t));
        std::memset(&vel_pid_, 0, sizeof(MotorPID_t));
    }

    void MotorCtrl::SetCtrlParam(const MotorPID_Config_t& pos_config, const MotorPID_Config_t& vel_config, uint32_t pos_vel_ratio)
    {
        MotorPID_Init(&pos_pid_, pos_config);
        MotorPID_Init(&vel_pid_, vel_config);
        pos_vel_ratio_ = (pos_vel_ratio == 0) ? 1 : pos_vel_ratio;
    }

    void MotorCtrl::SetCtrlParam(const PD_Config_t& pos_config, const MotorPID_Config_t& vel_config)
    {
        PD_Init(&pos_pd_, &pos_config);
        MotorPID_Init(&vel_pid_, vel_config);
        pos_vel_ratio_ = 1; // PD control usually runs at same frequency or doesn't use the ratio logic in this implementation
    }

    void MotorCtrl::SetCtrlParam(const MotorPID_Config_t& vel_config)
    {
        std::memset(&pos_pid_, 0, sizeof(MotorPID_t));
        MotorPID_Init(&vel_pid_, vel_config);
        pos_vel_ratio_ = 1;
    }

    void MotorCtrl::SetMitParams(float kp, float kd, float ki, float i_limit)
    {
        mit_kp_      = kp;
        mit_kd_      = kd;
        mit_ki_      = ki;
        mit_i_limit_ = i_limit;
        mit_sum_err_ = 0.0f;
    }

    void MotorCtrl::setEnable(bool enable)
    {
        enable_ = enable;
    }

    void MotorCtrl::setTarget(float primary_ref, float secondary_ref, float torque_ff)
    {
        primary_ref_   = primary_ref;
        secondary_ref_ = secondary_ref;
        torque_ff_     = torque_ff;
    }

    void MotorCtrl::update(float dt)
    {
        // 1. 更新反馈数据
        updateFeedback();

        if (!enable_)
        {
            outputControl(0.0f); // 设置为0力矩
            return;
        }

        float vel_target   = 0.0f;
        float final_output = 0.0f;

        switch (mode_)
        {
        case ControlMode::MIT:
            // MIT 模式下的积分项计算 (可分离微调)
            // 误差累积 (Pos Ref - Current Angle)
            if (mit_ki_ > 1e-6f)
            {
                // 计算位置误差 (primary_ref_ is angle in degrees)
                float pos_err = primary_ref_ - current_angle_;
                // 积分
                mit_sum_err_ += pos_err * dt;
                // 限幅
                if (mit_i_limit_ > 0.0f)
                {
                    if (mit_sum_err_ > mit_i_limit_) mit_sum_err_ = mit_i_limit_;
                    if (mit_sum_err_ < -mit_i_limit_) mit_sum_err_ = -mit_i_limit_;
                }
            }
            // 这里的 final_output 仅用于表示积分+前馈的总力矩
            // 实际 MIT 发送将在 switch 后特殊处理，或者直接在这里处理完 return
            {
                // 计算积分力矩
                float i_torque = mit_sum_err_ * mit_ki_;
                // 总前馈力矩 = 重力补偿(torque_ff_) + 积分微调(i_torque)
                float total_feedforward = torque_ff_ + i_torque;

                if (type_ == MotorType::DM)
                {
                    DM_t* dm = static_cast<DM_t*>(driver_);
                    // 转换单位: Angle(Deg)->Rad, Vel(Deg/s)->Rad/s
                    float p_des = primary_ref_ * DEG_TO_RAD;
                    float v_des = secondary_ref_ * DEG_TO_RAD;
                    DM_MIT_SendSetCmd(dm, p_des, v_des, mit_kp_, mit_kd_, total_feedforward);
                }
            }
            return; // MIT 模式直接处理完毕返回

        case ControlMode::PositionVelocityPID:
            // 2. 位置环计算 (降频)
            update_count_++;
            if (update_count_ >= pos_vel_ratio_)
            {
                pos_pid_.ref = primary_ref_; // Target Angle
                pos_pid_.fdb = current_angle_;
                MotorPID_Calculate(&pos_pid_);
                update_count_ = 0;
            }
            // 速度环目标 = 位置环输出
            vel_target = pos_pid_.output;

            // 3. 速度环计算
            vel_pid_.ref = vel_target;
            vel_pid_.fdb = current_velocity_;
            MotorPID_Calculate(&vel_pid_);

            final_output = vel_pid_.output;
            break;

        case ControlMode::PositionPD_VelocityFF:
            // 使用 PD 控制器，无降频
            pos_pd_.ref = primary_ref_; // Target Angle
            pos_pd_.fdb = current_angle_;
            PD_Calculate(&pos_pd_);

            // 速度环目标 = 位置环输出 + 前馈速度
            vel_target = pos_pd_.output + secondary_ref_;

            // 3. 速度环计算
            vel_pid_.ref = vel_target;
            vel_pid_.fdb = current_velocity_;
            MotorPID_Calculate(&vel_pid_);

            final_output = vel_pid_.output;
            break;

        case ControlMode::VelocityPID:
            // 速度环目标 = 用户设定值 (primary_ref)
            vel_target = primary_ref_;

            vel_pid_.ref = vel_target;
            vel_pid_.fdb = current_velocity_;
            MotorPID_Calculate(&vel_pid_);

            final_output = vel_pid_.output;
            break;

        case ControlMode::DirectTorque:
            final_output = primary_ref_; // primary_ref is torque
            break;

        case ControlMode::None:
        default:
            final_output = 0.0f;
            break;
        }

        // 4. 合成最终输出 (PID输出 + 前馈力矩)
        // torque_ratio_ 用于将 Nm 转换为电机控制单位 (DJI: IQ, Unitree: Nm)

        ctrl_value = final_output + torque_ff_ * torque_ratio_;
        // float ctrl_value = torque_ff_ * torque_ratio_;
        // 5. 发送控制指令
        outputControl(ctrl_value);
    }

    void MotorCtrl::updateFeedback()
    {
        if (!driver_)
            return;

        if (type_ == MotorType::DJI)
        {
            DJI_t* dji        = static_cast<DJI_t*>(driver_);
            current_angle_    = dji->abs_angle;
            current_velocity_ = dji->velocity * RPM_TO_DEGS;
        }
        else if (type_ == MotorType::Unitree)
        {
            ::UnitreeMotor* unitree = static_cast<::UnitreeMotor*>(driver_);
            current_angle_          = unitree->feedback.pos * RAD_TO_DEG;
            current_velocity_       = unitree->feedback.speed * RAD_TO_DEG;
        }
        else if (type_ == MotorType::DM)
        {
            DM_t* dm          = static_cast<DM_t*>(driver_);
            current_angle_    = dm->abs_angle;
            current_velocity_ = dm->vel * RPM_TO_DEGS; // DM.c 中 vel 单位是 rpm
        }
    }

    void MotorCtrl::outputControl(float output)
    {
        if (!driver_)
            return;
        
        if (type_ == MotorType::DM)
        {
            // 对于非MIT模式的普通控制 (如 DirectTorque -> DM Vel Mode or Pos Mode? No, DirectTorque implies Torque)
            // 如果 DM 工作在非 MIT 模式，outputControl 很难通用，因为 DM 的 Pos/Vel 接口语义不同
            // 这里假设只有 MIT 模式被主动使用。如果必须支持 Enable=False (0力矩)，可以使用 MIT 模式设 Kp=Kd=0 发送
             if (output == 0.0f) {
                 // 停机/禁能逻辑
                 DM_t* dm = static_cast<DM_t*>(driver_);
                 // 发送 MIT 模式 0 力矩 0 刚度
                 DM_MIT_SendSetCmd(dm, 0, 0, 0, 0, 0);
             }
             else {
                
             }
             return;
        }

        if (type_ == MotorType::DJI)
        {
            DJI_t* dji = static_cast<DJI_t*>(driver_);

            // 安全限幅，防止 int16_t 溢出导致反转
            float max_iq = 0.0f;
            if (dji->motor_type == M3508_C620)
                max_iq = (float)DJI_M3508_C620_IQ_MAX / 2;
            else if (dji->motor_type == M2006_C610)
                max_iq = (float)DJI_M2006_C610_IQ_MAX / 2;
            else
                max_iq = 10000.0f; // 默认安全值

            if (output > max_iq)
                output = max_iq;
            if (output < -max_iq)
                output = -max_iq;

            __DJI_SET_IQ_CMD(dji, output);
        }
        else if (type_ == MotorType::Unitree)
        {
            ::UnitreeMotor* unitree = static_cast<::UnitreeMotor*>(driver_);

            float target_pos_rad = 0.0f;
            if (mode_ == ControlMode::PositionVelocityPID || mode_ == ControlMode::PositionPD_VelocityFF)
            {
                target_pos_rad = primary_ref_ * DEG_TO_RAD;
            }
            else
            {
                target_pos_rad = current_angle_ * DEG_TO_RAD;
            }

            Unitree_SetCmd(unitree,
                           1,              // Mode 1: FOC Closed Loop
                           output,         // T (Calculated Torque)
                           0.0f,           // W (Target Velocity)
                           target_pos_rad, // Pos (Target Position)
                           0.0f,           // K_P (Stiffness = 0)
                           0.0f            // K_W (Damping = 0)
            );
        }
    }
    bool MotorCtrl::isConnected() const
    {
        if (!driver_) return false;
        if (type_ == MotorType::DJI)
        {
            const DJI_t* dji = static_cast<const DJI_t*>(driver_);
            return dji->feedback_count > 0;
        }
        else if (type_ == MotorType::Unitree)
        {
            const ::UnitreeMotor* unitree = static_cast<const ::UnitreeMotor*>(driver_);
            return unitree->feedback.rx_count > 0;
        }
        else if (type_ == MotorType::DM)
        {
            const DM_t* dm = static_cast<const DM_t*>(driver_);
            return dm->feedback_count > 0;
        }
        return false;
    }

} // namespace Arm
