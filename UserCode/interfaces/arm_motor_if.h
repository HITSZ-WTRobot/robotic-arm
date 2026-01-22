#pragma once

#include <cstdint>
#include "drivers/DJI.h"
#include "drivers/unitree_motor.h"
#include "drivers/DM.h"
#include "libs/pid_motor.h"
#include "libs/pid_pd.h"

namespace Arm
{

    enum class MotorType
    {
        DJI,
        Unitree,
        DM // Add DM Motor Type
    };

    enum class ControlMode
    {
        None,
        PositionVelocityPID,   // 1. 位置-速度串级 PID
        VelocityPID,           // 2. 纯速度环 PID
        PositionPD_VelocityFF, // 3. 位置 PD + 速度前馈
        DirectTorque,          // 4. 直接力矩/电流控制
        MIT                    // 5. 达妙 MIT 模式 (Kp, Kd, Pos, Vel, Tff + K_i积分微调)
    };

    /**
     * @brief 机械臂关节电机统一接口 (支持 DJI, Unitree, DM)
     */
    class MotorCtrl
    {
    public:
        /**
         * @brief 构造 DJI 电机实例
         * @param driver DJI电机驱动句柄 (DJI_t*)
         * @param mode 控制模式
         * @param torque_ratio 力矩转电流系数 (IQ/Nm)
         */
        MotorCtrl(DJI_t* driver, ControlMode mode, float torque_ratio);

        /**
         * @brief 构造 Unitree 电机实例
         * @param driver 宇树电机驱动句柄 (UnitreeMotor*)
         * @param mode 控制模式
         * @param torque_ratio 力矩系数 (通常为1.0, 但为了统一接口保留)
         */
        MotorCtrl(::UnitreeMotor* driver, ControlMode mode, float torque_ratio = 1.0f);

        /**
         * @brief 构造 DM 电机实例
         * @param driver 达妙电机驱动句柄 (DM_t*)
         * @param mode 控制模式 (推荐 MIT)
         */
        MotorCtrl(DM_t* driver, ControlMode mode);

        ~MotorCtrl() = default;

        /**
         * @brief 设置双环/串级 PID 参数 (用于 PositionVelocityPID 或 PositionPD_VelocityFF)
         * @param pos_config 位置环 PID 配置
         * @param vel_config 速度环 PID 配置
         * @param pos_vel_ratio 位置环与速度环的计算频率比
         */
        void SetCtrlParam(const MotorPID_Config_t& pos_config, const MotorPID_Config_t& vel_config, uint32_t pos_vel_ratio);

        /**
         * @brief 设置 PD + PID 参数 (用于 PositionPD_VelocityFF)
         * @param pos_config 位置环 PD 配置
         * @param vel_config 速度环 PID 配置
         */
        void SetCtrlParam(const PD_Config_t& pos_config, const MotorPID_Config_t& vel_config);

        /**
         * @brief 设置速度环 PID 参数 (用于 VelocityPID)
         * @param vel_config 速度环 PID 配置
         */
        void SetCtrlParam(const MotorPID_Config_t& vel_config);

        /**
         * @brief 设置 MIT 模式参数 (用于 DM 电机)
         * @param kp 刚度系数
         * @param kd 阻尼系数
         * @param ki 积分微调系数 (如果不使用积分项，设为0)
         * @param i_limit 积分限幅
         */
        void SetMitParams(float kp, float kd, float ki = 0.0f, float i_limit = 0.0f);

        /**
         * @brief 统一设置控制目标
         *
         * @param primary_ref   主目标值
         *                      - 位置相关模式/MIT: 目标角度 (Degree)
         *                      - 速度模式:     目标速度 (Degree/s)
         *                      - 力矩模式:     目标力矩 (Nm)
         * @param secondary_ref 次级目标值 (前馈项)
         *                      - PositionPD_VelocityFF/MIT: 前馈速度 (Degree/s)
         *                      - 其他模式: 通常为 0
         * @param torque_ff     前馈力矩 (Nm)
         *                      - 所有模式下，该值都会直接叠加到最终输出中
         */
        void setTarget(float primary_ref, float secondary_ref = 0.0f, float torque_ff = 0.0f);

        /**
         * @brief 启用/禁用电机控制
         * @param enable true: 启用; false: 禁用 (输出 0 力矩)
         */
        void setEnable(bool enable);

        /**
         * @brief 获取当前启用状态
         */
        bool isEnabled() const { return enable_; }

        /**
         * @brief 更新控制回路 (需周期性调用)
         * @param dt 控制周期 (秒), 默认 0.001f
         */
        void update(float dt = 0.001f);

        /**
         * @brief 获取当前电机角度，单位：度 (Degree)
         */
        float getAngle() const { return current_angle_; }

        /**
         * @brief 获取当前电机速度，单位：度每秒 (Degree/s)
         */
        float getVelocity() const { return current_velocity_; }

        /**
         * @brief 检查电机是否初始化成功（已连接）
         * @return true: 已连接/收到反馈; false: 未连接
         */
        bool isConnected() const;

    private:
        void updateFeedback();
        void outputControl(float output);

        MotorType type_;
        ControlMode mode_;
        bool enable_;
        void* driver_; // 泛型指针，指向 DJI_t 或 UnitreeMotor
        float torque_ratio_;
        uint32_t pos_vel_ratio_ = 1;
        uint32_t update_count_  = 0;

        // MIT 模式参数
        float mit_kp_      = 0.0f;
        float mit_kd_      = 0.0f;
        float mit_ki_      = 0.0f;
        float mit_i_limit_ = 0.0f;
        float mit_sum_err_ = 0.0f; // 积分项累积误差

        union
        {
            MotorPID_t pos_pid_;
            PD_t pos_pd_;
        };
        MotorPID_t vel_pid_;

        // 状态数据
        float current_angle_    = 0.0f;
        float current_velocity_ = 0.0f;

        // 控制目标
        float primary_ref_   = 0.0f;
        float secondary_ref_ = 0.0f;
        float torque_ff_     = 0.0f;

        float ctrl_value; // 最终控制输出值
    };

} // namespace Arm
