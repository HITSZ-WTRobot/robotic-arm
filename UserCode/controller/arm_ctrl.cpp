#include "arm_ctrl.h"
#include <cmath>

namespace Arm
{

    Controller::Controller(Motor& joint1, Motor& joint2, Motor& gripper, const Config& config) :
        joint1_(joint1), joint2_(joint2), gripper_(gripper), config_(config),
        target_q1_(0.0f), target_q2_(0.0f), target_gripper_(0.0f)
    {
    }

    void Controller::init()
    {
        // 可以在这里进行一些初始化操作，例如读取初始位置作为目标位置，防止上电跳变
        target_q1_      = joint1_.getAngle();
        target_q2_      = joint2_.getAngle();
        target_gripper_ = gripper_.getAngle();
    }

    void Controller::setJointTarget(float q1, float q2)
    {
        target_q1_ = q1;
        target_q2_ = q2;
    }

    void Controller::setGripper(float open_width)
    {
        target_gripper_ = open_width;
    }

    void Controller::update()
    {
        // 1. 获取当前关节角度
        float q1 = joint1_.getAngle();
        float q2 = joint2_.getAngle();

        // 2. 计算重力补偿力矩
        float tau1_g = 0.0f;
        float tau2_g = 0.0f;
        calculateGravityComp(q1, q2, tau1_g, tau2_g);

        // 3. 更新关节电机目标 (位置 + 前馈力矩)
        joint1_.setTarget(target_q1_, tau1_g);
        joint2_.setTarget(target_q2_, tau2_g);

        // 4. 更新夹爪电机 (假设夹爪不需要重力补偿，或者补偿量很小忽略)
        // 如果夹爪是力控夹持，这里可能需要改用力矩模式，目前假设是位置控制
        gripper_.setTarget(target_gripper_, 0.0f);

        // 5. 执行底层控制更新
        joint1_.update();
        joint2_.update();
        gripper_.update();
    }

    void Controller::getEndEffectorPose(float& x, float& y) const
    {
        float q1 = joint1_.getAngle();
        float q2 = joint2_.getAngle();

        // 正运动学公式 (假设平面 2-DOF 机械臂)
        // x = l1 * cos(q1) + l2 * cos(q1 + q2)
        // y = l1 * sin(q1) + l2 * sin(q1 + q2)
        // 注意: q2 通常是相对于 q1 的角度，或者是绝对角度，取决于机械结构定义
        // 这里假设 q2 是相对于连杆1的相对角度

        float l1 = config_.l1;
        float l2 = config_.l2;

        x = l1 * cosf(q1) + l2 * cosf(q1 + q2);
        y = l1 * sinf(q1) + l2 * sinf(q1 + q2);
    }

    void Controller::getJointAngles(float& q1, float& q2) const
    {
        q1 = joint1_.getAngle();
        q2 = joint2_.getAngle();
    }

    void Controller::calculateGravityComp(float q1, float q2, float& tau1, float& tau2)
    {
        // 动力学参数
        float m1  = config_.m1;
        float m2  = config_.m2;
        float mg  = config_.m_gripper;
        float l1  = config_.l1;
        float lc1 = config_.lc1;
        float lc2 = config_.lc2;
        float g   = config_.g;

        // 关节 2 (小臂) 的重力矩
        // 负载包括: 小臂自重 + 夹爪(末端负载)
        // T2 = m2 * g * lc2 * cos(q1 + q2) + mg * g * l2 * cos(q1 + q2)
        // 注意: 角度定义假设水平向右为 0 度，逆时针为正
        float cos_q12 = cosf(q1 + q2);
        tau2          = (m2 * lc2 + mg * config_.l2) * g * cos_q12;

        // 关节 1 (大臂) 的重力矩
        // 负载包括: 大臂自重 + 小臂自重 + 夹爪
        // T1 = m1 * g * lc1 * cos(q1) + T2_projected_to_joint1
        // 更精确的拉格朗日动力学推导:
        // T1 = (m1 * lc1 + m2 * l1 + mg * l1) * g * cos(q1) + (m2 * lc2 + mg * l2) * g * cos(q1 + q2)

        float cos_q1 = cosf(q1);
        tau1         = (m1 * lc1 + m2 * l1 + mg * l1) * g * cos_q1 + tau2;
    }

} // namespace Arm
