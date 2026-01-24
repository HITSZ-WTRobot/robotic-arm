#include "arm_motor_mit.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include "drivers/DM.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace Arm
{
    static constexpr float DEG_TO_RAD  = M_PI / 180.0f;
    static constexpr float RAD_TO_DEG  = 180.0f / M_PI;
    static constexpr float RPM_TO_DEGS = 360.0f / 60.0f;

    // ============================================================================
    // Constructors
    // ============================================================================

    MotorCtrl::MotorCtrl(DJI_t* driver, float torque_ratio) :
        type_(MotorType::DJI), enable_(true), driver_(driver),
        torque_ratio_(torque_ratio), kp_(0), kd_(0), ki_(0), i_limit_(0), i_enable_(true), sum_err_(0),
        current_angle_(0), current_velocity_(0),
        target_angle_(0), target_velocity_(0), target_torque_ff_(0),
        is_connected_(false), last_rx_count_(0), rx_timeout_(0)
    {
    }

    MotorCtrl::MotorCtrl(UnitreeMotor* driver, float torque_ratio) :
        type_(MotorType::Unitree), enable_(true), driver_(driver),
        torque_ratio_(torque_ratio), kp_(0), kd_(0), ki_(0), i_limit_(0), i_enable_(true), sum_err_(0),
        current_angle_(0), current_velocity_(0),
        target_angle_(0), target_velocity_(0), target_torque_ff_(0),
        is_connected_(false), last_rx_count_(0), rx_timeout_(0)
    {
    }

    MotorCtrl::MotorCtrl(DM_t* driver) :
        type_(MotorType::DM), enable_(true), driver_(driver),
        torque_ratio_(1.0f), kp_(0), kd_(0), ki_(0), i_limit_(0), i_enable_(true), sum_err_(0),
        current_angle_(0), current_velocity_(0),
        target_angle_(0), target_velocity_(0), target_torque_ff_(0),
        is_connected_(false), last_rx_count_(0), rx_timeout_(0)
    {
    }

    // ============================================================================
    // Parameter Setup
    // ============================================================================

    void MotorCtrl::SetMitParams(float kp, float kd, float ki, float i_limit)
    {
        kp_      = kp;
        kd_      = kd;
        ki_      = ki;
        i_limit_ = i_limit;
        sum_err_ = 0.0f; // Reset integral on param change
    }

    void MotorCtrl::SetIntegralEnable(bool enable)
    {
        i_enable_ = enable;
        if (!enable)
        {
            sum_err_ = 0.0f;
        }
    }

    void MotorCtrl::ResetIntegral()
    {
        sum_err_ = 0.0f;
    }

    // ============================================================================
    // Control Loop
    // ============================================================================

    void MotorCtrl::setTarget(float primary_ref, float secondary_ref, float torque_ff)
    {
        target_angle_     = primary_ref;   // Degree
        target_velocity_  = secondary_ref; // Degree/s
        target_torque_ff_ = torque_ff;     // Nm
    }

    void MotorCtrl::setEnable(bool enable)
    {
        enable_ = enable;
        if (!enable)
            ResetIntegral();
    }

    void MotorCtrl::update(float dt)
    {
        // 1. 获取反馈
        updateFeedback();

        // 1.1 检查连接状态 (超时保护)
        uint32_t current_rx_count = 0;
        if (driver_)
        {
            if (type_ == MotorType::DJI)
                current_rx_count = static_cast<DJI_t*>(driver_)->feedback_count;
            else if (type_ == MotorType::Unitree)
                current_rx_count = static_cast<UnitreeMotor*>(driver_)->feedback.rx_count;
            else if (type_ == MotorType::DM)
                current_rx_count = static_cast<DM_t*>(driver_)->feedback_count;
        }

        // 准备控制参数 (正常模式 / 断联保护模式)
        // 断联保护逻辑：
        // 如果断联，我们只通过 outputControl 输出前馈力矩（重力补偿），防止下坠。
        // 不再进行 MIT 计算，因为 feedback 不准。

        if (current_rx_count != last_rx_count_)
        {
            last_rx_count_ = current_rx_count;
            rx_timeout_    = 0.0f;
            is_connected_  = true;
        }
        else
        {
            rx_timeout_ += dt;
            if (rx_timeout_ > TIMEOUT_THRESHOLD)
            {
                is_connected_ = false;
            }
        }

        // 如果未启用，停止输出
        if (!enable_)
        {
            outputControl(0.0f);
            return;
        }

        if (!is_connected_)
        {
            // 断联期间：
            // 对于非 Native MIT 电机 (DJI/Unitree)，MCU 需要实时反馈来计算力矩。
            // 如果反馈丢失，控制回路失效，必须降级为只发送前馈 (重力补偿)。
            if (type_ != MotorType::DM)
            {
                float safe_ff = target_torque_ff_;
                total_torque_ = safe_ff;
                outputControl(total_torque_);
                return;
            }

            // 对于 DM (Native MIT) 电机：
            // 电机内部闭环，即便 MCU 收不到反馈 (RX断)，只要指令 (TX) 能到达，
            // 电机仍能基于内部真实位置正确控制。
            // 策略：继续发送正常目标 (Blind Transmit)，依靠电机内部 Watchdog 处理全断情况。
            // 不 return，继续向下执行发送逻辑。
        }

        // --- 已连接 ---

        // 2. 计算积分项 (Integral Term)
        // 误差 = 目标位置 - 当前位置
        // 注意：如果断联 (DM case)，current_angle_ 是旧值，pos_err 不准。
        // 所以下方的积分累积必须 && is_connected_
        float pos_err         = target_angle_ - current_angle_;
        float torque_integral = 0.0f;

        if (i_enable_ && is_connected_ && ki_ > 1e-6f)
        {
            sum_err_ += pos_err * dt;

            // 积分输出限幅
            torque_integral = sum_err_ * ki_;
            if (i_limit_ > 0.0f)
            {
                if (torque_integral > i_limit_)
                {
                    torque_integral = i_limit_;
                    sum_err_        = torque_integral / ki_;
                }
                else if (torque_integral < -i_limit_)
                {
                    torque_integral = -i_limit_;
                    sum_err_        = torque_integral / ki_;
                }
            }
        }
        else
        {
            if (!i_enable_)
                sum_err_ = 0.0f;
            torque_integral = 0.0f;
        }

        // 3. 执行控制策略
        float total_ff = target_torque_ff_ + torque_integral;

        if (type_ == MotorType::DM)
        {
            // --- 达妙 (Native MIT) ---
            DM_t* dm = static_cast<DM_t*>(driver_);

            // 使用原始目标
            DM_MIT_SendSetCmd(dm, target_angle_, target_velocity_, kp_, kd_, total_ff);
        }
        else

        {
            // --- DJI / Unitree (Manual MIT) ---
            float vel_err = target_velocity_ - current_velocity_;

            // 使用原始误差
            float torque_mit = (kp_ * pos_err) + (kd_ * vel_err);

            total_torque_ = torque_mit + total_ff;

            outputControl(total_torque_);
        }
    }

    // ============================================================================
    // Low Level
    // ============================================================================

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
            UnitreeMotor* uni = static_cast<UnitreeMotor*>(driver_);
            current_angle_    = uni->feedback.pos * RAD_TO_DEG;
            current_velocity_ = uni->feedback.speed * RAD_TO_DEG;
        }
        else if (type_ == MotorType::DM)
        {
            DM_t* dm          = static_cast<DM_t*>(driver_);
            current_angle_    = dm->abs_angle;
            current_velocity_ = dm->vel * RPM_TO_DEGS; // DM.c 中 vel 单位是 rpm
        }
    }

    void MotorCtrl::outputControl(float torque_nm)
    {
        if (!driver_)
            return;

        if (type_ == MotorType::DM)
        {
            // 特殊情况: 只有在 enable=false 时才会调到这里传 0
            // 正常控制走 update 中的 MIT Send
            if (torque_nm == 0.0f)
            {
                DM_t* dm = static_cast<DM_t*>(driver_);
                DM_MIT_SendSetCmd(dm, 0, 0, 0, 0, 0);
            }
            return;
        }

        if (type_ == MotorType::DJI)
        {
            DJI_t* dji = static_cast<DJI_t*>(driver_);

            // Torque (Nm) -> Current (IQ)
            // torque_ratio_ = IQ_MAX / Torque_Rating or similar
            // e.g. M3508: 16384 / ~3Nm approx? Users sets this ratio.

            float iq_float = torque_nm * torque_ratio_;
            iq_float_      = iq_float; //
            // Clamp
            float max_iq = 16000.0f; // Default Safe constant
            if (dji->motor_type == M3508_C620)
                max_iq = 16384.0f;
            if (dji->motor_type == M2006_C610)
                max_iq = 10000.0f;

            if (iq_float > max_iq)
                iq_float = max_iq;
            if (iq_float < -max_iq)
                iq_float = -max_iq;

            __DJI_SET_IQ_CMD(dji, (int16_t)iq_float);
        }
        else if (type_ == MotorType::Unitree)
        {
            UnitreeMotor* uni = static_cast<UnitreeMotor*>(driver_);

            // Unitree FOC Mode (Mode 1), Pure Torque control
            // Kp=0, Kd=0, Pos=0, Vel=0, T = torque_nm
            // Unitree driver handles Nm natively

            Unitree_SetCmd(uni,
                           1,         // Mode 1
                           torque_nm, // T
                           0,         // W
                           0,         // Pos
                           0,         // Kp
                           0          // Kw
            );
        }
    }

    bool MotorCtrl::isConnected() const
    {
        return is_connected_;
    }

} // namespace Arm
