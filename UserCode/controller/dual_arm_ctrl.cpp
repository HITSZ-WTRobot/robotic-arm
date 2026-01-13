#include "dual_arm_ctrl.h"
#include <cmath>
#include <cstdlib> // abs

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float DEG_TO_RAD = M_PI / 180.0f;

namespace Arm
{

    // ============================================================================
    // SCurveTrajectory Implementation (Helper)
    // ============================================================================

    void DualArmController::SCurveTrajectory::plan(float start, float end, float start_vel, float start_acc, float max_vel, float max_acc, float max_jerk)
    {
        SCurve_t temp_curve;
        SCurve_Result_t res = SCurve_Init(&temp_curve, start, end, start_vel, start_acc, max_vel, max_acc, max_jerk);
        if (res == S_CURVE_SUCCESS)
        {
            pending_curve = temp_curve;
            update_needed = true;
        }
    }

    void DualArmController::SCurveTrajectory::step(float dt, float& pos, float& vel)
    {
        if (update_needed)
        {
            curve         = pending_curve;
            current_time  = 0;
            running       = true;
            cur_pos       = curve.xs;
            cur_vel       = curve.vs;
            cur_acc       = curve.as;
            update_needed = false;
        }

        if (!running)
        {
            pos = cur_pos;
            vel = 0.0f;
            return;
        }

        current_time += dt;

        cur_pos = SCurve_CalcX(&curve, current_time);
        cur_vel = SCurve_CalcV(&curve, current_time);
        cur_acc = SCurve_CalcA(&curve, current_time);

        if (current_time >= curve.total_time)
        {
            running = false;
        }

        pos = cur_pos;
        vel = cur_vel;
    }

    // ============================================================================
    // DualArmController Implementation
    // ============================================================================

    DualArmController::DualArmController(MotorCtrl& joint1,
                                         MotorCtrl& joint2_master, MotorCtrl& joint2_slave,
                                         MotorCtrl& joint3_master, MotorCtrl& joint3_slave,
                                         const Config& config) :
        j1_(joint1),
        j2_m_(joint2_master), j2_s_(joint2_slave),
        j3_m_(joint3_master), j3_s_(joint3_slave),
        config_(config)
    {
        // 预计算重力补偿系数
        // G3 = m3 * g * lc3
        G3_ = config_.m3 * config_.g * config_.lc3;

        // G2 = (m2 * lc2 + m3 * l2) * g
        G2_ = (config_.m2 * config_.lc2 + config_.m3 * config_.l2) * config_.g;

        // G1 = (m1 * lc1 + m2 * l1 + m3 * l1) * g
        G1_ = (config_.m1 * config_.lc1 + config_.m2 * config_.l1 + config_.m3 * config_.l1) * config_.g;
    }

    void DualArmController::init()
    {
        // 1. 先禁用所有电机
        j1_.setEnable(false);
        j2_m_.setEnable(false);
        j2_s_.setEnable(false);
        j3_m_.setEnable(false);
        j3_s_.setEnable(false);

        // 2. 更新反馈以获取初始绝对位置
        j1_.update();
        j2_m_.update();
        j2_s_.update();
        j3_m_.update();
        j3_s_.update();

        // 3. 记录上电初始位置 (电机轴, Degree)
        j1_init_pos_   = j1_.getAngle();
        j2_m_init_pos_ = j2_m_.getAngle();
        j2_s_init_pos_ = j2_s_.getAngle(); // 记录从电机各自的零点
        j3_m_init_pos_ = j3_m_.getAngle();
        j3_s_init_pos_ = j3_s_.getAngle(); // 记录从电机各自的零点

        // 4. 计算当前关节角度并作为规划起点
        // 关节角度 = ((电机角度 - 初始角度) / 减速比) + 偏移
        // 初始化时刻 (电机-初始) 为0，所以关节角度 = 偏移
        float q1 = config_.offset_1;
        float q2 = config_.offset_2;
        float q3 = config_.offset_3;

        cur_q1_ref_ = q1;
        cur_q2_ref_ = q2;
        cur_q3_ref_ = q3;

        // 5. 初始化规划器状态
        traj_q1_.plan(q1, q1, 0, 0, config_.j1_max_vel, config_.j1_max_acc, config_.j1_max_jerk);
        traj_q2_.plan(q2, q2, 0, 0, config_.j2_max_vel, config_.j2_max_acc, config_.j2_max_jerk);
        traj_q3_.plan(q3, q3, 0, 0, config_.j3_max_vel, config_.j3_max_acc, config_.j3_max_jerk);

        // 假装已经运行了一次step来同步内部状态
        float dummy_p, dummy_v;
        traj_q1_.step(0, dummy_p, dummy_v);
        traj_q2_.step(0, dummy_p, dummy_v);
        traj_q3_.step(0, dummy_p, dummy_v);

        // 6. 启用电机 (准备起飞)
        j1_.setEnable(true);
        j2_m_.setEnable(true);
        j2_s_.setEnable(true);
        j3_m_.setEnable(true);
        j3_s_.setEnable(true);
    }

    void DualArmController::setJointTarget(float q1, float q2, float q3, float* t1, float* t2, float* t3)
    {
        // 高频去重: 如果目标没变就不重新规划
        if (fabsf(q1 - traj_q1_.curve.xe) > 1e-4f)
            traj_q1_.plan(traj_q1_.cur_pos, q1, traj_q1_.cur_vel, traj_q1_.cur_acc, config_.j1_max_vel, config_.j1_max_acc, config_.j1_max_jerk);

        if (fabsf(q2 - traj_q2_.curve.xe) > 1e-4f)
            traj_q2_.plan(traj_q2_.cur_pos, q2, traj_q2_.cur_vel, traj_q2_.cur_acc, config_.j2_max_vel, config_.j2_max_acc, config_.j2_max_jerk);

        if (fabsf(q3 - traj_q3_.curve.xe) > 1e-4f)
            traj_q3_.plan(traj_q3_.cur_pos, q3, traj_q3_.cur_vel, traj_q3_.cur_acc, config_.j3_max_vel, config_.j3_max_acc, config_.j3_max_jerk);

        if (t1)
            *t1 = traj_q1_.curve.total_time;
        if (t2)
            *t2 = traj_q2_.curve.total_time;
        if (t3)
            *t3 = traj_q3_.curve.total_time;
    }

    void DualArmController::update(float dt)
    {
        // --- 1. S曲线规划器更新 ---
        float q1_v_ref, q2_v_ref, q3_v_ref;
        traj_q1_.step(dt, cur_q1_ref_, q1_v_ref);
        traj_q2_.step(dt, cur_q2_ref_, q2_v_ref);
        traj_q3_.step(dt, cur_q3_ref_, q3_v_ref);

        // --- 2. 动力学重力补偿 ---
        float q1_read, q2_read, q3_read;
        getJointAngles(q1_read, q2_read, q3_read);

        float t1_g, t2_g, t3_g; // 关节所需的总重力矩 (Nm)
        calculateGravityComp(q1_read * DEG_TO_RAD, q2_read * DEG_TO_RAD, q3_read * DEG_TO_RAD, t1_g, t2_g, t3_g);

        // --- 3. 关节 2 (小臂) 同步纠偏 ---
        float q2_m_v_corr, q2_s_v_corr;
        applySyncCorrection(j2_m_, j2_s_, j2_m_init_pos_, j2_s_init_pos_, q2_v_ref, config_.reduction_2,
                            config_.sync_gain_2, config_.sync_deadzone_2,
                            q2_m_v_corr, q2_s_v_corr);

        // --- 4. 关节 3 (吸盘) 同步纠偏 ---
        float q3_m_v_corr, q3_s_v_corr;
        applySyncCorrection(j3_m_, j3_s_, j3_m_init_pos_, j3_s_init_pos_, q3_v_ref, config_.reduction_3,
                            config_.sync_gain_3, config_.sync_deadzone_3,
                            q3_m_v_corr, q3_s_v_corr);


        // --- 5. 双环 PID 目标分发 ---

        // 关节 1 (单电机)
        // 目标: 规划位置, 规划速度(前馈), 重力矩(前馈)
        float motor1_target_pos = (cur_q1_ref_ - config_.offset_1) * config_.reduction_1 + j1_init_pos_;
        float motor1_ff_vel     = q1_v_ref * config_.reduction_1;
        float motor1_ff_tau     = t1_g / config_.reduction_1;

        j1_.setTarget(motor1_target_pos, motor1_ff_vel, motor1_ff_tau);
        // j1_.setTarget(motor1_target_pos, motor1_ff_vel, 0.0f);
        // 关节 2 (双电机)
        // 重力矩平分 (t2_g / 2)
        // 速度前馈包含纠偏量: (规划速度 +/- 纠偏速度) * 减速比
        float motor2_m_target = (cur_q2_ref_ - config_.offset_2) * config_.reduction_2 + j2_m_init_pos_;
        float motor2_s_target = (cur_q2_ref_ - config_.offset_2) * config_.reduction_2 + j2_s_init_pos_;

        // j2_m_.setTarget(motor2_m_target, q2_m_v_corr * config_.reduction_2, (t2_g * 0.5f) / config_.reduction_2);
        // j2_s_.setTarget(motor2_s_target, q2_s_v_corr * config_.reduction_2, (t2_g * 0.5f) / config_.reduction_2);
        j2_m_.setTarget(motor2_m_target, q2_m_v_corr * config_.reduction_2, 0.0f);
        j2_s_.setTarget(motor2_s_target, q2_s_v_corr * config_.reduction_2, 0.0f);

        // 关节 3 (双电机)
        float motor3_m_target = (cur_q3_ref_ - config_.offset_3) * config_.reduction_3 + j3_m_init_pos_;
        float motor3_s_target = (cur_q3_ref_ - config_.offset_3) * config_.reduction_3 + j3_s_init_pos_;

        // j3_m_.setTarget(motor3_m_target, q3_m_v_corr * config_.reduction_3, (t3_g * 0.5f) / config_.reduction_3);
        // j3_s_.setTarget(motor3_s_target, q3_s_v_corr * config_.reduction_3, (t3_g * 0.5f) / config_.reduction_3);
        j3_m_.setTarget(motor3_m_target, q3_m_v_corr * config_.reduction_3, 0.0f);
        j3_s_.setTarget(motor3_s_target, q3_s_v_corr * config_.reduction_3, 0.0f);


        // --- 6. 执行底层 PID ---
        j1_.update();
        j2_m_.update();
        j2_s_.update();
        j3_m_.update();
        j3_s_.update();
    }

    void DualArmController::applySyncCorrection(const MotorCtrl& master, const MotorCtrl& slave,
                                                float master_init_pos, float slave_init_pos,
                                                float ref_speed, float reduction, float gain, float deadzone,
                                                float& output_v_m, float& output_v_s)
    {
        // 1. 获取当前电机轴角度
        float angle_m = master.getAngle();
        float angle_s = slave.getAngle();

        // 2. 转换为电机轴增量 (相对于各自零点)
        float delta_m = (angle_m - master_init_pos);
        float delta_s = (angle_s - slave_init_pos);

        // 3. 转换为关节角度增量
        // 如果电机正方向一致（通常 master reverse=false, slave reverse=true 被 driver 处理了），
        // 如果 driver 处理了 reverse，那么 getAngle 都是正向增加。
        // S_Curve 输出的是 joint angle (degree).
        // delta_m / reduction 就是 master 走的 joint degree.
        float q_m = delta_m / reduction;
        float q_s = delta_s / reduction;

        // 4. 计算偏差 (Master - Slave)
        // 如果 Master 跑得比 Slave 快 (q_m > q_s)，skew > 0
        float skew_err = q_m - q_s;

        // 5. 死区
        if (std::abs(skew_err) < deadzone)
        {
            skew_err = 0.0f;
        }

        // 6. 修正速度 (Degree/s)
        // 增益 gain (1/s)
        // v_corr 应该加到 Velocity FF 上
        float v_corr = skew_err * gain;

        // Master 跑太快 (skew>0) -> v_corr > 0 -> Master 减速 (ref - corr), Slave 加速 (ref + corr)
        // 这样两者会趋近
        output_v_m = ref_speed - v_corr;
        output_v_s = ref_speed + v_corr;
    }

    void DualArmController::getJointAngles(float& q1, float& q2, float& q3) const
    {
        // 关节角度 = ((电机角度 - 初始角度) / 减速比) + 偏移
        // 双电机取 Master 作为反馈基准
        q1 = ((j1_.getAngle() - j1_init_pos_) / config_.reduction_1) + config_.offset_1;
        q2 = ((j2_m_.getAngle() - j2_m_init_pos_) / config_.reduction_2) + config_.offset_2;
        q3 = ((j3_m_.getAngle() - j3_m_init_pos_) / config_.reduction_3) + config_.offset_3;
    }

    void DualArmController::calculateGravityComp(float q1, float q2, float q3, float& tau1, float& tau2, float& tau3)
    {
        float c1   = cosf(q1);
        float c12  = cosf(q1 + q2);
        float c123 = cosf(q1 + q2 + q3);

        // 吸盘关节 (Joint 3)
        // T3 = G3 * cos(q1 + q2 + q3)
        tau3 = G3_ * c123;

        // 小臂 (Joint 2)
        // T2 = G2 * cos(q1 + q2) + T3
        float term2 = G2_ * c12;
        tau2        = term2 + tau3;

        // 大臂 (Joint 1)
        // T1 = G1 * cos(q1) + T2
        float term1 = G1_ * c1;
        tau1        = term1 + tau2;
    }

    void DualArmController::getEndEffectorPose(float& x, float& y, float& phi) const
    {
        float q1, q2, q3;
        getJointAngles(q1, q2, q3);
        float q1_rad = q1 * DEG_TO_RAD;
        float q2_rad = q2 * DEG_TO_RAD;
        float q3_rad = q3 * DEG_TO_RAD;

        float c1   = cosf(q1_rad);
        float s1   = sinf(q1_rad);
        float c12  = cosf(q1_rad + q2_rad);
        float s12  = sinf(q1_rad + q2_rad);
        float c123 = cosf(q1_rad + q2_rad + q3_rad);
        float s123 = sinf(q1_rad + q2_rad + q3_rad);

        x   = config_.l1 * c1 + config_.l2 * c12 + config_.l3 * c123;
        y   = config_.l1 * s1 + config_.l2 * s12 + config_.l3 * s123;
        phi = q1 + q2 + q3;
    }

    bool DualArmController::isArrived() const
    {
        return !traj_q1_.running && !traj_q2_.running && !traj_q3_.running;
    }

} // namespace Arm
