#pragma once

#include "interfaces/arm_motor_if.h"
#include "libs/s_curve.h"

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
            float l3; // 吸盘长度

            // 连杆质心位置 (距离关节轴的距离, m)
            float lc1;
            float lc2;
            float lc3;

            // 连杆质量 (kg)
            float m1;
            float m2;
            float m3; // 吸盘关节+吸盘质量

            // 重力加速度
            float g;

            // 减速比 (电机转动圈数 / 关节转动圈数)
            // 例如: 减速比为 10 表示电机转 10 圈，关节转 1 圈
            float reduction_1;
            float reduction_2;
            float reduction_3;

            // 关节零位偏移 (Degree)
            // 关节角度 = (电机角度 / 减速比) + 偏移
            float offset_1;
            float offset_2;
            float offset_3;

            // 运动学限制 (Degree)
            float j1_max_vel;
            float j1_max_acc;
            float j1_max_jerk;

            float j2_max_vel;
            float j2_max_acc;
            float j2_max_jerk;

            float j3_max_vel;
            float j3_max_acc;
            float j3_max_jerk;
        };

        /**
         * @brief 构造函数
         * @param joint1 大臂电机 (Unitree)
         * @param joint2 小臂电机 (DJI 3508)
         * @param joint3 吸盘关节电机 (DJI 2006)
         * @param config 机械臂物理参数
         */
        Controller(MotorCtrl& joint1, MotorCtrl& joint2, MotorCtrl& joint3, const Config& config);

        /**
         * @brief 初始化控制器
         */
        void init();

        /**
         * @brief 设置关节目标角度 (关节空间控制)
         * @param q1 大臂目标角度 (degree)
         * @param q2 小臂目标角度 (degree)
         * @param q3 吸盘关节目标角度 (degree)
         * @param t1 [out] (可选) 大臂预计运动时间 (s)
         * @param t2 [out] (可选) 小臂预计运动时间 (s)
         * @param t3 [out] (可选) 吸盘关节预计运动时间 (s)
         */
        void setJointTarget(float q1, float q2, float q3, float* t1 = nullptr, float* t2 = nullptr, float* t3 = nullptr);

        /**
         * @brief 查询关节是否都已到达目标
         * @return true 已到达, false 运动中
         */
        bool isArrived() const;

        /**
         * @brief 更新控制回路 (需周期性调用)
         * @param dt 距离上一次调用的时间间隔 (s)
         * @note 此函数会计算重力补偿并更新电机
         */
        void update(float dt);

        /**
         * @brief 正运动学求解 (FK)
         * @param x [out] 末端 X 坐标 (m)
         * @param y [out] 末端 Y 坐标 (m)
         * @param phi [out] 末端姿态角 (rad)
         */
        void getEndEffectorPose(float& x, float& y, float& phi) const;

        /**
         * @brief 获取当前关节角度
         */
        void getJointAngles(float& q1, float& q2, float& q3) const;

    private:
        /**
         * @brief S型曲线轨迹规划器
         */
        struct SCurveTrajectory
        {
            SCurve_t curve;
            SCurve_t pending_curve;
            volatile bool update_needed;

            float current_time;
            bool running;

            // 当前状态缓存
            float cur_pos;
            float cur_vel;
            float cur_acc;

            SCurveTrajectory() :
                update_needed(false), current_time(0), running(false), cur_pos(0), cur_vel(0), cur_acc(0) {}

            void plan(float start_pos, float end_pos, float start_vel, float start_acc, float max_vel, float max_acc, float max_jerk);
            void step(float dt, float& pos, float& vel);
        };

        /**
         * @brief 计算重力补偿力矩
         * @param q1 当前大臂角度
         * @param q2 当前小臂角度
         * @param q3 当前吸盘关节角度
         * @param tau1 [out] 大臂补偿力矩
         * @param tau2 [out] 小臂补偿力矩
         * @param tau3 [out] 吸盘关节补偿力矩
         */
        void calculateGravityComp(float q1, float q2, float q3, float& tau1, float& tau2, float& tau3);

        MotorCtrl& joint1_; // 大臂
        MotorCtrl& joint2_; // 小臂
        MotorCtrl& joint3_; // 吸盘关节

        Config config_;

        SCurveTrajectory traj_q1_;
        SCurveTrajectory traj_q2_;
        SCurveTrajectory traj_q3_;

        float current_q1_ref_; // 当前规划的位置参考值
        float current_q2_ref_;
        float current_q3_ref_;

        // 电机上电初始位置 (弧度)
        float motor1_init_pos_;
        float motor2_init_pos_;
        float motor3_init_pos_;
    };

} // namespace Arm
