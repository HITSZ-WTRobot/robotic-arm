#pragma once

#include <cstdint>
#include "drivers/DJI.h"
#include "drivers/DM.h"
#include "drivers/unitree_motor.h"

namespace Arm
{

    enum class MotorType
    {
        DJI,
        Unitree,
        DM
    };

    /**
     * @brief 机械臂关节电机统一接口 (纯 MIT 模式 + 可控积分项)
     *
     * 设计理念:
     * 所有电机(DJI/Unitree/DM)均统一抽象为阻抗控制模型:
     * Torque = Kp * (Pos_des - Pos_fdb) + Kd * (Vel_des - Vel_fdb) + Torque_ff + Torque_integral
     *
     * - 对于 DM 电机: 利用其原生 MIT 模式，仅需在外部计算 Integral 项并叠加到 T_ff 发送。
     * - 对于 DJI/Unitree: 在本类中完整实现上述公式计算出 Torque，再转换为电流/力矩指令发送。
     */
    class MotorCtrl
    {
    public:
        /**
         * @brief 构造函数
         */
        MotorCtrl(DJI_t* driver, float torque_ratio);
        MotorCtrl(UnitreeMotor* driver, float torque_ratio = 1.0f);
        MotorCtrl(DM_t* driver);

        ~MotorCtrl() = default;

        /**
         * @brief 设置 MIT 参数 (核心配置接口)
         * @param kp 刚度系数 (Stiffness)
         * @param kd 阻尼系数 (Damping)
         * @param ki 积分系数 (Integral Gain)
         * @param i_limit 积分限幅 (Integral Limit)
         */
        void SetMitParams(float kp, float kd, float ki, float i_limit);

        /**
         * @brief 启用/禁用积分项 (运行时可动态开关)
         */
        void SetIntegralEnable(bool enable);

        /**
         * @brief 重置积分累积误差
         */
        void ResetIntegral();

        /**
         * @brief 设置控制目标
         * @param primary_ref   目标位置 (Degree)
         * @param secondary_ref 目标速度 (Degree/s) - 前馈速度
         * @param torque_ff     前馈力矩 (Nm)
         */
        void setTarget(float primary_ref, float secondary_ref = 0.0f, float torque_ff = 0.0f);

        void setEnable(bool enable);
        bool isEnabled() const { return enable_; }

        // 周期性更新 (建议 1kHz)
        void update(float dt);

        float getAngle() const { return current_angle_; }
        float getVelocity() const { return current_velocity_; }
        bool isConnected() const;

    private:
        void updateFeedback();
        void outputControl(float torque);

        MotorType type_;
        bool enable_;
        void* driver_;

        float torque_ratio_; // Nm -> IQ (DJI) or Scale (Unitree)

        // MIT Parameters
        float kp_;
        float kd_;
        float ki_;
        float i_limit_;

        // Integral State
        bool i_enable_;
        float sum_err_;

        // State Feedback
        float current_angle_;    // Degree
        float current_velocity_; // Degree/s

        // Control Targets
        float target_angle_;     // Degree
        float target_velocity_;  // Degree/s
        float target_torque_ff_; // Nm

        float total_torque_; // Nm

        float iq_float_; // 临时变量

        // Connection Protection
        bool is_connected_                       = false;
        uint32_t last_rx_count_                  = 0;
        float rx_timeout_                        = 0.0f;
        static constexpr float TIMEOUT_THRESHOLD = 0.1f; // 100ms
    };

} // namespace Arm
