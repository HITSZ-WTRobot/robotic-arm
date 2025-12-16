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
        SCurve_Result_t res = SCurve_Init(&curve, start, end, start_vel, start_acc, max_vel, max_acc, max_jerk);
        if (res == S_CURVE_SUCCESS)
        {
            running      = true;
            current_time = 0;
            cur_pos      = start;
            cur_vel      = start_vel;
            cur_acc      = start_acc;
        }
        else
        {
            // Planning failed (e.g. constraints violation), stop at current or jump to end?
            // Safest is to stop or keep previous motion?
            // Here we just stop at start position for safety.
            running = false;
            cur_pos = start;
            cur_vel = 0;
            cur_acc = 0;
        }
    }

    void Controller::SCurveTrajectory::step(float dt, float& pos, float& vel)
    {
        if (!running)
        {
            pos = cur_pos;
            vel = 0.0f;
            return;
        }

        current_time += dt;

        if (current_time >= curve.total_time)
        {
            running = false;
            cur_pos = curve.xe;
            cur_vel = 0;
            cur_acc = 0;
            pos     = cur_pos;
            vel     = cur_vel;
            return;
        }

        cur_pos = SCurve_CalcX(&curve, current_time);
        cur_vel = SCurve_CalcV(&curve, current_time);
        cur_acc = SCurve_CalcA(&curve, current_time);

        pos = cur_pos;
        vel = cur_vel;
    }

    // ============================================================================
    // Controller Implementation
    // ============================================================================

    Controller::Controller(Motor& joint1, Motor& joint2, Motor& joint3, const Config& config) :
        joint1_(joint1), joint2_(joint2), joint3_(joint3), config_(config),
        current_q1_ref_(0.0f), current_q2_ref_(0.0f), current_q3_ref_(0.0f),
        motor1_init_pos_(0.0f), motor2_init_pos_(0.0f), motor3_init_pos_(0.0f)
    {
    }

    void Controller::init()
    {
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
    }

    void Controller::setJointTarget(float q1, float q2, float q3, float* t1, float* t2, float* t3)
    {
        // 获取当前状态作为起点 (解决速度跳变问题)
        // 使用当前规划器的输出作为起点可能比使用传感器反馈更平滑，
        // 但如果偏差较大，使用反馈更安全。这里为了连续性，使用当前电机的实际反馈速度。
        // 位置则使用当前规划的参考位置，避免因控制误差导致的轨迹回跳。

        traj_q1_.plan(traj_q1_.cur_pos, q1, traj_q1_.cur_vel, traj_q1_.cur_acc, config_.j1_max_vel, config_.j1_max_acc, config_.j1_max_jerk);
        traj_q2_.plan(traj_q2_.cur_pos, q2, traj_q2_.cur_vel, traj_q2_.cur_acc, config_.j2_max_vel, config_.j2_max_acc, config_.j2_max_jerk);
        traj_q3_.plan(traj_q3_.cur_pos, q3, traj_q3_.cur_vel, traj_q3_.cur_acc, config_.j3_max_vel, config_.j3_max_acc, config_.j3_max_jerk);

        if (t1) *t1 = traj_q1_.curve.total_time;
        if (t2) *t2 = traj_q2_.curve.total_time;
        if (t3) *t3 = traj_q3_.curve.total_time;
    }

    bool Controller::isArrived() const
    {
        return !traj_q1_.running && !traj_q2_.running && !traj_q3_.running;
    }

    void Controller::update(float dt)
    {
        // 1. 计算轨迹生成的新目标状态
        float q1_vel_ref, q2_vel_ref, q3_vel_ref;

        // traj_q1_.step(dt, current_q1_ref_, q1_vel_ref);
        // traj_q2_.step(dt, current_q2_ref_, q2_vel_ref);
        // traj_q3_.step(dt, current_q3_ref_, q3_vel_ref);

        traj_q1_.step(dt, current_q1_ref_, q1_vel_ref);
        traj_q2_.step(dt, current_q2_ref_, q2_vel_ref);
        traj_q3_.step(dt, current_q3_ref_, q3_vel_ref);
        // 2. 计算重力补偿力矩 (需要弧度)
        float q1_rad = current_q1_ref_ * DEG_TO_RAD;
        float q2_rad = current_q2_ref_ * DEG_TO_RAD;
        float q3_rad = current_q3_ref_ * DEG_TO_RAD;

        float tau1_g = 0.0f;
        float tau2_g = 0.0f;
        calculateGravityComp(q1_rad, q2_rad, q3_rad, tau1_g, tau2_g);

        // 3. 更新关节电机目标 (位置 + 前馈力矩)
        // setTarget 接受度数
        // 电机目标角度 = (关节目标角度 - 偏移) * 减速比 + 初始角度
        // 电机前馈力矩 = 关节前馈力矩 / 减速比
        joint1_.setTarget((current_q1_ref_ - config_.offset_1) * config_.reduction_1 + motor1_init_pos_, tau1_g / config_.reduction_1);
        // joint2_.setTarget((current_q2_ref_ - config_.offset_2) * config_.reduction_2 + motor2_init_pos_, tau2_g / config_.reduction_2);

        joint2_.setTarget((current_q2_ref_ - config_.offset_2) * config_.reduction_2 + motor2_init_pos_, 0.0f);
        joint3_.setTarget((current_q3_ref_ - config_.offset_3) * config_.reduction_3 + motor3_init_pos_, 0.0f);

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

        // 平面 2-DOF 机械臂
        float q12  = q1_rad + q2_rad;
        float q123 = q1_rad + q2_rad + q3_rad;

        // x = l1 * cosf(q1) + l2 * cosf(q12) + l3 * cosf(q123);
        // y = l1 * sinf(q1) + l2 * sinf(q12) + l3 * sinf(q123);

        x   = l1 * cosf(q1_rad) + l2 * cosf(q12);
        y   = l1 * sinf(q1_rad) + l2 * sinf(q12);
        phi = q123;
    }

    void Controller::getJointAngles(float& q1, float& q2, float& q3) const
    {
        // 关节角度 = ((电机角度 - 初始角度) / 减速比) + 偏移
        q1 = ((joint1_.getAngle() - motor1_init_pos_) / config_.reduction_1) + config_.offset_1;
        q2 = ((joint2_.getAngle() - motor2_init_pos_) / config_.reduction_2) + config_.offset_2;
        q3 = ((joint3_.getAngle() - motor3_init_pos_) / config_.reduction_3) + config_.offset_3;
    }

    void Controller::calculateGravityComp(float q1, float q2, float q3, float& tau1, float& tau2)
    {
        // 动力学参数
        float m1  = config_.m1;
        float m2  = config_.m2;
        float m3  = config_.m3; // 吸盘关节+吸盘质量
        float l1  = config_.l1;
        float l2  = config_.l2;
        float lc1 = config_.lc1;
        float lc2 = config_.lc2;
        float g   = config_.g;

        float c1  = cosf(q1);
        float c12 = cosf(q1 + q2);

        // 吸盘关节 (Joint 3): 仅受自身重力矩影响
        //    T3 = m3 * g * lc3 * cos(q1 + q2 + q3)

        // 小臂 (Joint 2)
        //    忽略 l3 长度，认为 m3 挂在 l2 末端
        //    T2 = m2 * g * lc2 * cos(q1 + q2) + m3 * g * l2 * cos(q1 + q2)
        float term2 = (m2 * lc2 + m3 * l2) * g * c12;
        tau2        = term2;

        // 大臂 (Joint 1)
        //    T1_gravity = (m1*lc1 + m2*l1 + m3*l1) * g * cos(q1) + T2
        float term1 = (m1 * lc1 + m2 * l1 + m3 * l1) * g * c1;
        tau1        = term1 + term2;
    }

} // namespace Arm
