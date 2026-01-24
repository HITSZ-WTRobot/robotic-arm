#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include "cmsis_os2.h"
#include "controller/arm_ctrl.h"
#include "main.h"

// 动作定义结构体
struct ActionDef {
    const float (*points)[3];
    int len;
};

// --- 轨迹数据 ---


// --- Action 0 ---

// Generated Waypoints: action_seq_0
static const float action_seq_0[][3] = {
    {0.483992f, -0.565233f, 0.081240f},
    {0.348656f, -0.317776f, -0.030880f},
    {0.176947f, -0.000000f, -0.176947f}
};
static const int action_seq_0_len = sizeof(action_seq_0) / sizeof(action_seq_0[0]);

// --- Action 1 ---

// Generated Waypoints: action_seq_1
static const float action_seq_1[][3] = {
    {0.457476f, 1.487347f, -0.548560f},
    {0.694260f, 1.597146f, -0.895142f},
    {0.588731f, 1.781573f, -0.974040f},
    {0.487221f, 1.953227f, -1.044185f}
};
static const int action_seq_1_len = sizeof(action_seq_1) / sizeof(action_seq_1[0]);

// --- Action 2 ---

// Generated Waypoints: action_seq_2
static const float action_seq_2[][3] = {
    {0.172873f, -0.294947f, -1.448722f},
    {0.145915f, -0.296504f, -1.420207f},
    {0.113462f, -0.287569f, -1.396689f}
};
static const int action_seq_2_len = sizeof(action_seq_2) / sizeof(action_seq_2[0]);

// --- Action 3 ---

// Generated Waypoints: action_seq_3
static const float action_seq_3[][3] = {
    {2.547942f, -1.955038f, 0.977892f},
    {2.598845f, -1.958305f, 0.930256f},
    {2.503794f, -1.786934f, 0.853937f},
    {2.383226f, -1.601787f, 0.789358f},
    {1.718801f, -0.574488f, -1.144313f},
    {1.227038f, -0.000000f, -1.227038f}
};
static const int action_seq_3_len = sizeof(action_seq_3) / sizeof(action_seq_3[0]);

// --- Action 4 ---

// Generated Waypoints: action_seq_4
static const float action_seq_4[][3] = {
    {2.200430f, -1.713449f, 0.909282f},
    {2.262795f, -1.881919f, 1.015387f},
    {3.106686f, -2.862340f, 1.570796f}
};
static const int action_seq_4_len = sizeof(action_seq_4) / sizeof(action_seq_4[0]);

// --- Action 5 ---

// Generated Waypoints: action_seq_5
static const float action_seq_5[][3] = {
    {1.570796f, -1.570796f, 0.000000f},
    {3.106686f, -2.862340f, 1.570796f}
};
static const int action_seq_5_len = sizeof(action_seq_5) / sizeof(action_seq_5[0]);

// Action Table Registration
static const ActionDef ACTION_TABLE[] = {
    {action_seq_0, action_seq_0_len}, // ID 0
    {action_seq_1, action_seq_1_len}, // ID 1
    {action_seq_2, action_seq_2_len}, // ID 2
    {action_seq_3, action_seq_3_len}, // ID 3
    {action_seq_4, action_seq_4_len}, // ID 4
    {action_seq_5, action_seq_5_len}, // ID 5
};
static const int ACTION_TOTAL_COUNT = sizeof(ACTION_TABLE) / sizeof(ACTION_TABLE[0]);

// 弧度转角度
static const float RAD2DEG = 180.0f / 3.1415926f;

/**
 * @brief 播放预设轨迹
 *
 * @param arm 控制器实例
 * @param action_id 动作ID (0 ~ ACTION_TOTAL_COUNT-1)
 */
static void Arm_PlayAction(Arm::Controller* arm, int action_id)
{
    if (!arm)
        return;

    // ID 合法性检查
    if (action_id < 0 || action_id >= ACTION_TOTAL_COUNT)
        return;

    const float(*traj)[3] = ACTION_TABLE[action_id].points;
    int len               = ACTION_TABLE[action_id].len;

    // 执行轨迹点
    for (int i = 0; i < len; ++i)
    {
        float t1, t2, t3;
        arm->setJointTarget(
            traj[i][0] * RAD2DEG,
            traj[i][1] * RAD2DEG,
            traj[i][2] * RAD2DEG,
            &t1, &t2, &t3);

        // 获取同步后的最大时间 (由于 Controller 已做 Sync，理论上 t1~t3 接近)
        float max_t = t1 > t2 ? t1 : t2;
        if (t3 > max_t)
            max_t = t3;

        // 轨迹平滑混合 (Trajectory Blending):
        // 这里的关键技巧是只等待 50% 的规划时间。
        // 当时间过半时，机械臂还在运动，此时下发下一个目标点。
        // Controller 检测到起点速度不为0，会自动规划一条 C2 连续的曲线衔接过去，
        // 从而实现类似"圆滑过弯"的效果，而不会在每个点停顿。
        uint32_t wait_ms = (uint32_t)(max_t * 1000.0f);

        // 最小间隔保护，防止指令过于密集堵塞
        if (wait_ms < 10)
            wait_ms = 10;

        osDelay(wait_ms);
    }
}

#endif // _TRAJECTORY_H_
