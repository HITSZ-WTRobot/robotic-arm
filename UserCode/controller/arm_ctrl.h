#pragma once

#include "interfaces/arm_motor_if.h"

namespace Arm
{

    /**
     * @brief 机械臂控制器 (2-DOF + Gripper)
     */
    class Controller
    {
    public:
        /**
         * @brief 机械臂物理参数配置
         */
        struct Config
        {
            // 连杆长度 (m)
            float l1; // 大臂长度
            float l2; // 小臂长度

            // 连杆质心位置 (距离关节轴的距离, m)
            float lc1;
            float lc2;

            // 连杆质量 (kg)
            float m1;
            float m2;
            float m_gripper; // 夹爪质量 (视为小臂末端的点质量)

            // 重力加速度
            float g;
        };

        /**
         * @brief 构造函数
         * @param joint1 大臂电机 (Unitree)
         * @param joint2 小臂电机 (DJI 3508)
         * @param gripper 夹爪电机 (DJI 2006)
         * @param config 机械臂物理参数
         */
        Controller(Motor& joint1, Motor& joint2, Motor& gripper, const Config& config);

        /**
         * @brief 初始化控制器
         */
        void init();

        /**
         * @brief 设置关节目标角度 (关节空间控制)
         * @param q1 大臂角度 (rad)
         * @param q2 小臂角度 (rad)
         */
        void setJointTarget(float q1, float q2);

        /**
         * @brief 控制夹爪开合
         * @param open_width 开合宽度或角度 (根据具体夹爪实现定义)
         */
        void setGripper(float open_width);

        /**
         * @brief 更新控制回路 (需周期性调用, e.g. 1kHz)
         * @note 此函数会计算重力补偿并更新电机
         */
        void update();

        /**
         * @brief 正运动学求解 (FK)
         * @param x [out] 末端 X 坐标 (m)
         * @param y [out] 末端 Y 坐标 (m)
         */
        void getEndEffectorPose(float& x, float& y) const;

        /**
         * @brief 获取当前关节角度
         */
        void getJointAngles(float& q1, float& q2) const;

    private:
        /**
         * @brief 计算重力补偿力矩
         * @param q1 当前大臂角度
         * @param q2 当前小臂角度
         * @param tau1 [out] 大臂补偿力矩
         * @param tau2 [out] 小臂补偿力矩
         */
        void calculateGravityComp(float q1, float q2, float& tau1, float& tau2);

        Motor& joint1_;  // 大臂
        Motor& joint2_;  // 小臂
        Motor& gripper_; // 夹爪

        Config config_;

        float target_q1_;
        float target_q2_;
        float target_gripper_;
    };

} // namespace Arm
