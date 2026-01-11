#pragma once

#include "interfaces/arm_motor_if.h"
#include "libs/s_curve.h"

namespace Arm
{

    /**
     * @brief 机械臂控制器 (针对双电机龙门架结构优化)
     * 支持:
     * 1. 关节 1: 单电机
     * 2. 关节 2: 双电机 (龙门架同步)
     * 3. 关节 3: 双电机 (龙门架同步)
     */
    class DualArmController
    {
    public:
        /**
         * @brief 机械臂物理参数配置
         */
        struct Config
        {
            // --- 机械臂本体参数 ---
            float l1; // 大臂长度
            float l2; // 小臂长度
            float l3; // 吸盘长度

            float lc1, lc2, lc3; // 质心位置
            float m1, m2, m3;    // 质量 (kg)
            float g;             // 重力加速度

            // --- 传动参数 ---
            // 减速比 (电机转动圈数 / 关节转动圈数)
            float reduction_1;
            float reduction_2;
            float reduction_3;

            // 关节零位偏移 (Degree)
            float offset_1;
            float offset_2;
            float offset_3;

            // --- 运动学限制 (用于 S 曲线规划) ---
            float j1_max_vel, j1_max_acc, j1_max_jerk;
            float j2_max_vel, j2_max_acc, j2_max_jerk;
            float j3_max_vel, j3_max_acc, j3_max_jerk;

            // --- 双电机同步参数 (Anti-Skewing) ---
            // 同步纠正增益 (1/s): 值越大纠在越快，但也越容易震荡
            // 建议范围: 1.0 ~ 5.0
            float sync_gain_2;
            float sync_gain_3;

            // 同步死区 (Degree): 只有偏差超过此值才进行纠正，防震荡
            // 建议范围: 0.1 ~ 0.5
            float sync_deadzone_2;
            float sync_deadzone_3;
        };

        /**
         * @brief 构造函数
         *
         * @param joint1        关节1电机 (单)
         * @param joint2_master 关节2主电机
         * @param joint2_slave  关节2从电机
         * @param joint3_master 关节3主电机
         * @param joint3_slave  关节3从电机
         * @param config        配置参数
         */
        DualArmController(MotorCtrl& joint1,
                          MotorCtrl& joint2_master, MotorCtrl& joint2_slave,
                          MotorCtrl& joint3_master, MotorCtrl& joint3_slave,
                          const Config& config);

        ~DualArmController() = default;

        /**
         * @brief 初始化控制器
         * 读取当前位置作为初始点，并启用电机
         */
        void init();

        /**
         * @brief 设置关节目标位置 (Degree)
         * 会自动触发 S 曲线规划
         *
         * @param q1 关节1目标角度
         * @param q2 关节2目标角度
         * @param q3 关节3目标角度
         * @param t1 (可选) 返回关节1规划时间
         * @param t2 (可选) 返回关节2规划时间
         * @param t3 (可选) 返回关节3规划时间
         */
        void setJointTarget(float q1, float q2, float q3, float* t1 = nullptr, float* t2 = nullptr, float* t3 = nullptr);

        /**
         * @brief 周期性更新 (请在定时器中以固定频率调用, 如 1kHz)
         *
         * @param dt 每次调用的时间间隔 (s)
         */
        void update(float dt);

        /**
         * @brief 获取当前关节角度 (Degree)
         * 对于双电机关节，返回主电机的角度
         */
        void getJointAngles(float& q1, float& q2, float& q3) const;

        /**
         * @brief 获取当前末端执行器坐标 (正运动学)
         */
        void getEndEffectorPose(float& x, float& y, float& phi) const;

        /**
         * @brief 检查是否所有关节都已到达目前位置 (轨迹规划结束)
         */
        bool isArrived() const;

    private:
        // --- 电机引用 ---
        MotorCtrl& j1_;
        MotorCtrl& j2_m_;
        MotorCtrl& j2_s_;
        MotorCtrl& j3_m_;
        MotorCtrl& j3_s_;

        Config config_;

        // --- 内部状态 ---
        // 初始位置 (电机轴角度)
        float j1_init_pos_   = 0;
        float j2_m_init_pos_ = 0;
        float j2_s_init_pos_ = 0;
        float j3_m_init_pos_ = 0;
        float j3_s_init_pos_ = 0;

        // 当前规划器输出的目标角度 (关节空间, Degree)
        float cur_q1_ref_ = 0;
        float cur_q2_ref_ = 0;
        float cur_q3_ref_ = 0;

        // 重力补偿系数缓存
        float G1_ = 0, G2_ = 0, G3_ = 0;


        // --- 内部辅助类: 轨迹规划器 ---
        struct SCurveTrajectory
        {
            SCurve_t curve;
            SCurve_t pending_curve;
            bool update_needed = false;
            bool running       = false;
            float current_time = 0.0f;

            // 保持当前状态用于下一次规划的起点
            float cur_pos = 0.0f;
            float cur_vel = 0.0f;
            float cur_acc = 0.0f;

            void plan(float start, float end, float start_vel, float start_acc, float max_vel, float max_acc, float max_jerk);
            void step(float dt, float& pos, float& vel);
        };

        SCurveTrajectory traj_q1_;
        SCurveTrajectory traj_q2_;
        SCurveTrajectory traj_q3_;

        // --- 辅助计算 ---
        void calculateGravityComp(float q1, float q2, float q3, float& tau1, float& tau2, float& tau3);

        /**
         * @brief 计算并应用同步纠偏
         *
         * @param master 主电机
         * @param slave  从电机
         * @param ref_speed 规划的前馈速度
         * @param reduction 减速比
         * @param gain      同步增益
         * @param deadzone  死区
         * @param output_v_m (输出) 主电机修正后的前馈速度
         * @param output_v_s (输出) 从电机修正后的前馈速度
         */
        void applySyncCorrection(const MotorCtrl& master, const MotorCtrl& slave,
                                 float master_init_pos, float slave_init_pos,
                                 float ref_speed, float reduction, float gain, float deadzone,
                                 float& output_v_m, float& output_v_s);
    };

} // namespace Arm
