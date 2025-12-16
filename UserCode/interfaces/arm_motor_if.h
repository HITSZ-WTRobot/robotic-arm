#pragma once

#include <cstdint>
#include "drivers/DJI.h"
#include "drivers/Unitree_user.h"
#include "libs/pid_motor.h"

namespace Arm
{

    enum class MotorType
    {
        DJI,
        Unitree
    };

    enum class ControlMode
    {
        Position,
        Velocity
    };

    /**
     * @brief 机械臂关节电机统一接口 (支持 DJI 和 Unitree)
     */
    class Motor
    {
    public:
        /**
         * @brief 构造 DJI 电机实例
         * @param driver DJI电机驱动句柄 (DJI_t*)
         * @param torque_ratio 力矩转电流系数 (IQ/Nm)
         * @param pos_vel_ratio 位置环与速度环的计算频率比
         */
        Motor(DJI_t* driver, float torque_ratio, uint32_t pos_vel_ratio = 1);

        /**
         * @brief 构造 Unitree 电机实例
         * @param driver 宇树电机驱动句柄 (UnitreeMotor*)
         * @param id 电机ID
         * @param pos_vel_ratio 位置环与速度环的计算频率比
         */
        Motor(::UnitreeMotor* driver, uint8_t id, uint32_t pos_vel_ratio = 1);

        ~Motor() = default;

        /**
         * @brief 设置双环 PID 参数 (自动设置为位置模式)
         */
        void setPID(float pos_kp, float pos_ki, float pos_kd, float pos_max_out,
                    float vel_kp, float vel_ki, float vel_kd, float vel_max_out);

        /**
         * @brief 设置速度环 PID 参数 (自动设置为速度模式)
         */
        void setPID(float vel_kp, float vel_ki, float vel_kd, float vel_max_out);

        /**
         * @brief 设置控制目标
         * @param angle 目标角度 (degree)
         * @param torque_ff 前馈力矩 (Nm)
         */
        void setTarget(float angle, float torque_ff);

        /**
         * @brief 设置速度目标 (速度模式调试用)
         * @param velocity 目标速度 (degree/s)
         */
        void setVelocityTarget(float velocity);

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
        ControlMode mode_ = ControlMode::Position;
        void* driver_; // 泛型指针，指向 DJI_t 或 UnitreeMotor
        uint8_t id_;   // 仅用于 Unitree

        // 状态数据
        float current_angle_    = 0.0f;
        float current_velocity_ = 0.0f;

        // 控制目标
        float target_angle_    = 0.0f;
        float target_velocity_ = 0.0f;
        float feedforward_     = 0.0f;

        // 控制算法数据
        MotorPID_t pos_pid_;
        MotorPID_t vel_pid_;
        float torque_ratio_;
        uint32_t pos_vel_ratio_;
        uint32_t update_count_;
    };

} // namespace Arm
