#include "arm_ctrl.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float DEG_TO_RAD = M_PI / 180.0f;

namespace Arm
{

    // ============================================================================
    // SCurveTrajectory Implementation
    // ============================================================================

    void Controller::SCurveTrajectory::plan(float start, float end, float start_vel, float start_acc, float max_vel, float max_acc, float max_jerk)
    {
        SCurve_t temp_curve;
        SCurve_Result_t res = SCurve_Init(&temp_curve, start, end, start_vel, start_acc, max_vel, max_acc, max_jerk);
        if (res == S_CURVE_SUCCESS)
        {
            pending_curve = temp_curve;
            update_needed = true;
        }
        else
        {
            // Planning failed
            // Do nothing, keep running old trajectory or stop?
            // For now, ignore invalid plans
        }
    }

    void Controller::SCurveTrajectory::step(float dt, float& pos, float& vel)
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
    // Controller Implementation
    // ============================================================================

    Controller::Controller(MotorCtrl& joint1, MotorCtrl& joint2, MotorCtrl& joint3, const Config& config) :
        joint1_(joint1), joint2_(joint2), joint3_(joint3), config_(config),
        current_q1_ref_(0.0f), current_q2_ref_(0.0f), current_q3_ref_(0.0f),
        motor1_init_pos_(0.0f), motor2_init_pos_(0.0f), motor3_init_pos_(0.0f),
        current_payload_mass_(0.0f), target_payload_mass_(0.0f), payload_ramp_rate_(0.0f),
        soft_start_scale_(0.0f)
    {
        // 预计算重力补偿系数
        // G3 = m3 * g * lc3
        G3_ = config_.m3 * config_.g * config_.lc3;

        // G2 = (m2 * lc2 + m3 * l2) * g
        G2_ = (config_.m2 * config_.lc2 + config_.m3 * config_.l2) * config_.g;

        // G1 = (m1 * lc1 + m2 * l1 + m3 * l1) * g
        G1_ = (config_.m1 * config_.lc1 + config_.m2 * config_.l1 + config_.m3 * config_.l1) * config_.g;

        // 预计算负载重力系数 (假设负载在连杆末端)
        G_payload_factor_3_ = config_.g * config_.l3; // 实际上负载在吸盘末端，这里假设 l3 是吸盘长度
        G_payload_factor_2_ = config_.g * config_.l2;
        G_payload_factor_1_ = config_.g * config_.l1;
    }

    void Controller::init()
    {
        // 重置软启动
        soft_start_scale_ = 0.0f;

        // 1. 先禁用电机，防止在上电初始位置未知的情况下进行 PID 计算
        joint1_.setEnable(false);
        joint2_.setEnable(false);
        joint3_.setEnable(false);

        // 2. 更新反馈，获取当前物理位置
        // 调用 update() 在禁用状态下会执行：读取反馈 + 发送0力矩指令
        // 这比单纯 updateFeedback() 更安全，确保电机显式处于放松状态
        joint1_.update();
        joint2_.update();
        joint3_.update();

        // 记录上电时的电机初始位置 (度)
        motor1_init_pos_ = joint1_.getAngle();
        motor2_init_pos_ = joint2_.getAngle();
        motor3_init_pos_ = joint3_.getAngle();

        // 读取初始位置作为目标位置，防止上电跳变
        // 注意: getAngle 返回度，这里存储在内部状态中
        // 考虑减速比: 关节角度 = ((电机角度 - 初始角度) / 减速比) + 偏移
        // 初始时刻 (电机角度 - 初始角度) 为 0，所以初始关节角度 = 偏移
        float q1_deg = config_.offset_1;
        float q2_deg = config_.offset_2;
        float q3_deg = config_.offset_3;

        current_q1_ref_ = q1_deg;
        current_q2_ref_ = q2_deg;
        current_q3_ref_ = q3_deg;

        // 初始化轨迹为当前位置，速度为0
        traj_q1_.plan(q1_deg, q1_deg, 0, 0, config_.j1_max_vel, config_.j1_max_acc, config_.j1_max_jerk);
        traj_q2_.plan(q2_deg, q2_deg, 0, 0, config_.j2_max_vel, config_.j2_max_acc, config_.j2_max_jerk);
        traj_q3_.plan(q3_deg, q3_deg, 0, 0, config_.j3_max_vel, config_.j3_max_acc, config_.j3_max_jerk);

        // 3. 初始化完成后启用电机
        joint1_.setEnable(true);
        joint2_.setEnable(true);
        joint3_.setEnable(true);
    }

    void Controller::setPayload(float mass, float ramp_time)
    {
        target_payload_mass_ = mass;
        if (ramp_time > 0.001f)
        {
            // 计算变化率: (目标 - 当前) / 时间
            // 但这里我们只存储速率大小，方向在 update 中判断
            // 或者简单点，存储 abs 速率
            payload_ramp_rate_ = fabsf(mass - current_payload_mass_) / ramp_time;
        }
        else
        {
            current_payload_mass_ = mass;
            payload_ramp_rate_    = 0.0f;
        }
    }

    void Controller::setJointTarget(float q1, float q2, float q3, float* t1, float* t2, float* t3)
    {
        // 高频控制优化:
        // 使用当前"规划器"的内部状态(cur_pos, cur_vel, cur_acc)作为新轨迹的起点。
        // 1. 保证了位置、速度、加速度的连续性 (C2连续)，避免速度跳变。
        // 2. 避免了传感器噪声(特别是速度噪声)进入规划层，防止轨迹抖动。
        // 3. 增加判断：如果目标位置未发生改变，则跳过规划，节省计算资源。

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

    bool Controller::isArrived() const
    {
        return !traj_q1_.running && !traj_q2_.running && !traj_q3_.running;
    }

    void Controller::update(float dt)
    {
        // 0. 更新状态
        // 0.1 负载质量平滑过渡
        if (payload_ramp_rate_ > 0.0f)
        {
            float diff = target_payload_mass_ - current_payload_mass_;
            float step = payload_ramp_rate_ * dt;
            if (fabsf(diff) <= step)
            {
                current_payload_mass_ = target_payload_mass_;
            }
            else
            {
                current_payload_mass_ += (diff > 0 ? step : -step);
            }
        }

        // 0.2 软启动
        if (soft_start_scale_ < 1.0f)
        {
            soft_start_scale_ += dt / soft_start_duration_;
            if (soft_start_scale_ > 1.0f)
                soft_start_scale_ = 1.0f;
        }

        // 1. 计算轨迹生成的新目标状态
        float q1_vel_ref, q2_vel_ref, q3_vel_ref;

        // traj_q1_.step(dt, current_q1_ref_, q1_vel_ref);
        // traj_q2_.step(dt, current_q2_ref_, q2_vel_ref);
        // traj_q3_.step(dt, current_q3_ref_, q3_vel_ref);

        traj_q1_.step(dt, current_q1_ref_, q1_vel_ref);
        traj_q2_.step(dt, current_q2_ref_, q2_vel_ref);
        traj_q3_.step(dt, current_q3_ref_, q3_vel_ref);
        // 2. 计算重力补偿力矩 (需要弧度)
        float q1_real, q2_real, q3_real;
        getJointAngles(q1_real, q2_real, q3_real);

        float q1_rad = q1_real * DEG_TO_RAD;
        float q2_rad = q2_real * DEG_TO_RAD;
        float q3_rad = q3_real * DEG_TO_RAD;

        float tau1_g = 0.0f;
        float tau2_g = 0.0f;
        float tau3_g = 0.0f;
        calculateGravityComp(q1_rad, q2_rad, q3_rad, tau1_g, tau2_g, tau3_g);

        // 应用软启动比例
        tau1_g *= soft_start_scale_;
        tau2_g *= soft_start_scale_;
        tau3_g *= soft_start_scale_;

        // 3. 更新关节电机目标 (位置 + 前馈速度 + 前馈力矩)
        // setTarget 接受度数
        // 电机目标角度 = (关节目标角度 - 偏移) * 减速比 + 初始角度
        // 电机前馈速度 = 关节前馈速度 * 减速比
        // 电机前馈力矩 = 关节前馈力矩 / 减速比
        joint1_.setTarget((current_q1_ref_ - config_.offset_1) * config_.reduction_1 + motor1_init_pos_,
                          q1_vel_ref * config_.reduction_1,
                          tau1_g / config_.reduction_1);
        // joint1_.setTarget((current_q1_ref_ - config_.offset_1) * config_.reduction_1 + motor1_init_pos_,
        //                   q1_vel_ref * config_.reduction_1,
        //                   0);
        joint2_.setTarget((current_q2_ref_ - config_.offset_2) * config_.reduction_2 + motor2_init_pos_,
                          q2_vel_ref * config_.reduction_2,
                          tau2_g / config_.reduction_2);
        joint3_.setTarget((current_q3_ref_ - config_.offset_3) * config_.reduction_3 + motor3_init_pos_,
                          q3_vel_ref * config_.reduction_3,
                          tau3_g / config_.reduction_3);

        // 5. 执行底层控制更新
        joint1_.update();
        joint2_.update();
        joint3_.update();
    }

    void Controller::getEndEffectorPose(float& x, float& y, float& phi) const
    {
        // 关节角度 = ((电机角度 - 初始角度) / 减速比) + 偏移
        float q1 = ((joint1_.getAngle() - motor1_init_pos_) / config_.reduction_1) + config_.offset_1; // 度
        float q2 = ((joint2_.getAngle() - motor2_init_pos_) / config_.reduction_2) + config_.offset_2; // 度
        float q3 = ((joint3_.getAngle() - motor3_init_pos_) / config_.reduction_3) + config_.offset_3; // 度

        // 转换为弧度用于三角函数计算
        float q1_rad = q1 * DEG_TO_RAD;
        float q2_rad = q2 * DEG_TO_RAD;
        float q3_rad = q3 * DEG_TO_RAD;

        float l1 = config_.l1;
        float l2 = config_.l2;
        float l3 = config_.l3;

        // 平面 2-DOF 机械臂
        float q12  = q1_rad + q2_rad;
        float q123 = q1_rad + q2_rad + q3_rad;

        x = l1 * cosf(q1_rad) + l2 * cosf(q12) + l3 * cosf(q123);
        y = l1 * sinf(q1_rad) + l2 * sinf(q12) + l3 * sinf(q123);

        // x   = l1 * cosf(q1_rad) + l2 * cosf(q12);
        // y   = l1 * sinf(q1_rad) + l2 * sinf(q12);
        phi = q123;
    }

    void Controller::getJointAngles(float& q1, float& q2, float& q3) const
    {
        // 关节角度 = ((电机角度 - 初始角度) / 减速比) + 偏移
        q1 = ((joint1_.getAngle() - motor1_init_pos_) / config_.reduction_1) + config_.offset_1;
        q2 = ((joint2_.getAngle() - motor2_init_pos_) / config_.reduction_2) + config_.offset_2;
        q3 = ((joint3_.getAngle() - motor3_init_pos_) / config_.reduction_3) + config_.offset_3;
    }

    void Controller::calculateGravityComp(float q1, float q2, float q3, float& tau1, float& tau2, float& tau3)
    {
        float c1   = cosf(q1);
        float c12  = cosf(q1 + q2);
        float c123 = cosf(q1 + q2 + q3);

        // 考虑负载影响
        float G3_eff = G3_ + current_payload_mass_ * G_payload_factor_3_;
        float G2_eff = G2_ + current_payload_mass_ * G_payload_factor_2_;
        float G1_eff = G1_ + current_payload_mass_ * G_payload_factor_1_;

        // 吸盘关节 (Joint 3): 仅受自身重力矩影响
        // T3 = G3 * cos(q1 + q2 + q3)
        tau3 = G3_eff * c123;

        // 小臂 (Joint 2)
        // T2 = G2 * cos(q1 + q2) + T3
        float term2 = G2_eff * c12;
        tau2        = term2 + tau3;

        // 大臂 (Joint 1)
        // T1 = G1 * cos(q1) + T2
        float term1 = G1_eff * c1;
        tau1        = term1 + tau2;
    }

} // namespace Arm
