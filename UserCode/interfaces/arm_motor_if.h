#pragma once

#include <cstdint>
#include "drivers/DJI.h"
#include "drivers/Unitree_user.h"
#include "libs/pid_motor.h"
#include "libs/pid_pd.h"

namespace Arm
{

    enum class MotorType
    {
        DJI,
        Unitree
    };

    enum class ControlMode
    {
        None,
        PositionVelocityPID,   // 1. 位置-速度串级 PID
        VelocityPID,           // 2. 纯速度环 PID
        PositionPD_VelocityFF, // 3. 位置 PD + 速度前馈
        DirectTorque           // 4. 直接力矩/电流控制
    };

    /**
     * @brief 机械臂关节电机统一接口 (支持 DJI 和 Unitree)
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
         * @param id 电机ID
         * @param mode 控制模式
         * @param torque_ratio 力矩系数 (通常为1.0, 但为了统一接口保留)
         */
        MotorCtrl(::UnitreeMotor* driver, uint8_t id, ControlMode mode, float torque_ratio = 1.0f);

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
         * @brief 统一设置控制目标
         *
         * @param primary_ref   主目标值
         *                      - 位置相关模式: 目标角度 (Degree)
         *                      - 速度模式:     目标速度 (Degree/s)
         *                      - 力矩模式:     目标力矩 (Nm)
         * @param secondary_ref 次级目标值 (前馈项)
         *                      - PositionPD_VelocityFF 模式: 前馈速度 (Degree/s)
         *                      - 其他模式: 通常为 0
         * @param torque_ff     前馈力矩 (Nm)
         *                      - 所有模式下，该值都会直接叠加到最终输出中
         */
        void setTarget(float primary_ref, float secondary_ref = 0.0f, float torque_ff = 0.0f);

        /**
         * @brief 更新控制回路 (需周期性调用)
         */
        void update();

        /**
         * @brief 获取当前电机角度，单位：度 (Degree)
         */
        float getAngle() const { return current_angle_; }

        /**
         * @brief 获取当前电机速度，单位：度每秒 (Degree/s)
         */
        float getVelocity() const { return current_velocity_; }

    private:
        void updateFeedback();
        void outputControl(float output);

        MotorType type_;
        ControlMode mode_;
        void* driver_; // 泛型指针，指向 DJI_t 或 UnitreeMotor
        uint8_t id_;   // 仅用于 Unitree
        float torque_ratio_;
        uint32_t pos_vel_ratio_ = 1;
        uint32_t update_count_  = 0;

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
    };

} // namespace Arm
