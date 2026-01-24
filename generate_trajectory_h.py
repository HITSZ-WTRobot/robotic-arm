import math
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Configuration ---
OUTPUT_FILE_PATH = "UserCode/app/trajectory.h"

CPP_HEADER = """#ifndef _TRAJECTORY_H_
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
"""

CPP_FOOTER = """
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
"""

class ThreeAxisManipulator:
    """平面三轴机械臂类"""
    def __init__(self, base_x=0, base_y=0):
        # 机械臂参数（单位：米）
        self.arm1_length = 0.346  # 大臂长
        self.arm2_length = 0.382  # 小臂长
        self.suction_length = 0.093  # 吸盘长
        self.base_x = base_x
        self.base_y = base_y
        
    def forward_kinematics(self, angles):
        """正运动学计算 - 标准坐标系 (0度向右, 逆时针为正)"""
        theta1, theta2, theta3 = angles
        
        # 计算各关节位置
        x1 = self.base_x + self.arm1_length * math.cos(theta1)
        y1 = self.base_y + self.arm1_length * math.sin(theta1)
        
        # theta2 是相对关节1的角度
        x2 = x1 + self.arm2_length * math.cos(theta1 + theta2)
        y2 = y1 + self.arm2_length * math.sin(theta1 + theta2)
        
        # theta3 是相对关节2的角度
        x3 = x2 + self.suction_length * math.cos(theta1 + theta2 + theta3)
        y3 = y2 + self.suction_length * math.sin(theta1 + theta2 + theta3)
        
        return (x1, y1), (x2, y2), (x3, y3)
    
    def inverse_kinematics(self, target_x, target_y, suction_angle_deg=0.0, elbow_down=False):
        """
        逆运动学计算
        :param target_x, target_y: 末端执行器 (吸盘顶端) 的坐标
        :param suction_angle_deg: 吸盘相对于地面的绝对角度 (度). 0=水平向右, 90=垂直向下
        :param elbow_down: 是否肘部朝下/反向 (对于某些解)
        """
        
        # 将角度转换为弧度
        # 用户输入处理: 
        # 0 -> 0 (水平)
        # -90 -> -pi/2 (垂直向下)
        # 90 -> -pi/2 (兼容旧逻辑，通常垂直向下对于抓取是 -90 物理方向)
        
        abs_suction_angle = math.radians(suction_angle_deg)

        # 1. 回推腕关节 (Wrist) 位置
        # Wrist = Tip - SuctionVector
        wrist_x = target_x - self.suction_length * math.cos(abs_suction_angle)
        wrist_y = target_y - self.suction_length * math.sin(abs_suction_angle)
        
        dx = wrist_x - self.base_x
        dy = wrist_y - self.base_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 检查是否在可达范围内
        max_reach = self.arm1_length + self.arm2_length
        if distance > max_reach:
            scale = max_reach / distance
            dx *= scale
            dy *= scale
            distance = max_reach
        
        # 计算前两个关节角度
        try:
            cos_theta2 = (distance**2 - self.arm1_length**2 - self.arm2_length**2) / \
                         (2 * self.arm1_length * self.arm2_length)
            cos_theta2 = max(-1, min(1, cos_theta2))
            theta2 = -math.acos(cos_theta2)
        except ValueError:
            theta1 = math.atan2(dy, dx)
            theta2 = 0
            theta3 = abs_suction_angle - theta1 - theta2
            return [theta1, theta2, theta3]
        
        # 处理肘部朝向
        if elbow_down:
            theta2 = -theta2

        # 计算关节1
        k1 = self.arm1_length + self.arm2_length * math.cos(theta2)
        k2 = self.arm2_length * math.sin(theta2)
        theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)
        
        # 计算关节3
        # abs_angle = theta1 + theta2 + theta3
        theta3 = abs_suction_angle - theta1 - theta2
            
        return [theta1, theta2, theta3]

def normalize_angle(angle):
    """归一化角度到 [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

def check_limits(conf_deg):
    """检查关节角度是否在限制范围内"""
    # q1: -20 to 180
    # q2: -175 to 175
    # q3: -100 to 100
    limits = [(-20, 180), (-175, 175), (-100, 100)]
    valid = True
    msg = ""
    for i, (val, (low, high)) in enumerate(zip(conf_deg, limits)):
        if not (low <= val <= high):
            valid = False
            msg += f" J{i+1}={val:.1f}(Limit {low}~{high})"
    return valid, msg

def generate_cpp_array_str(name, waypoints):
    """
    生成 C++ 格式的轨迹点数组字符串
    :param name: C++ 数组变量名
    :param waypoints: waypoints_info 列表
    """
    lines = []
    lines.append(f"\n// Generated Waypoints: {name}")
    lines.append(f"static const float {name}[][3] = {{")
    for i, wp in enumerate(waypoints):
        pt = wp['conf']
        q1 = pt[0]
        q2 = pt[1]
        q3 = pt[2]
        
        comma = "," if i < len(waypoints) - 1 else ""
        lines.append(f"    {{{q1:.6f}f, {q2:.6f}f, {q3:.6f}f}}{comma}")
    lines.append("};")
    lines.append(f"static const int {name}_len = sizeof({name}) / sizeof({name}[0]);")
    return "\n".join(lines)

# 2. Waypoints Data
# preset_waypoints_data = [
#     # stop=True 表示在该点速度减为0（停顿），stop=False 表示经过该点时不减速（连续运动）
#     ((0.780, 0.130), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
#     ((0.800, 0.130), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
#     ((0.820, 0.130), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
#     ((0.187, 0.520), {'elbow_down': True, 'suction_vertical': 80, 'stop': False}), # 演示：中间点不停顿
#     ((0.0735, 0.490), {'elbow_down': True, 'suction_vertical': 80, 'stop': True}), # 演示：中间点不停顿
#     ((0.0735, 0.437), {'elbow_down': True, 'suction_vertical': 80, 'stop': True}),
#     ((0.346, 0.422), {'elbow_down': True, 'suction_vertical': 90, 'stop': True}),
#     ((0.821, 0), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
# ]
# preset_waypoints_data = [
#     # stop=True 表示在该点速度减为0（停顿），stop=False 表示经过该点时不减速（连续运动）
#     ((0.720, -0.120), {'elbow_down': False, 'suction_vertical': -90, 'stop': True}),
#     ((0.720, -0.140), {'elbow_down': False, 'suction_vertical': -90, 'stop': True}),
#     ((0.187, 0.520), {'elbow_down': True, 'suction_vertical': 80, 'stop': False}), # 演示：中间点不停顿
#     ((0.0735, 0.490), {'elbow_down': True, 'suction_vertical': 80, 'stop': True}), # 演示：中间点不停顿
#     ((0.0735, 0.437), {'elbow_down': True, 'suction_vertical': 80, 'stop': True}),
#     ((0.187, 0.710), {'elbow_down': True, 'suction_vertical': 0, 'stop': False}),
#     ((0.220, 0.710), {'elbow_down': True, 'suction_vertical': 0, 'stop': False}),
#     ((0.346, 0.422), {'elbow_down': True, 'suction_vertical': 90, 'stop': True}),
#     ((0.821, 0), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
# ]


waypoints_data = [
    [#0 从初始位置出来去取物体水平（200mm 测面）
    # stop=True 表示在该点速度减为0（停顿），stop=False 表示经过该点时不减速（连续运动）
    ((0.780, 0.130), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
    ((0.800, 0.130), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
    ((0.820, 0.130), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
    ],
    [#1 放置到转盘上方
    ((0.187, 0.600), {'elbow_down': True, 'suction_vertical': 80, 'stop': False}), # 演示：中间点不停顿
    ((0.030, 0.600), {'elbow_down': True, 'suction_vertical': 80, 'stop': True}), # 演示：中间点不停顿
    ((0.030, 0.550), {'elbow_down': True, 'suction_vertical': 80, 'stop': True}),
    ((0.030, 0.500), {'elbow_down': True, 'suction_vertical': 80, 'stop': True}),
    ],
    [#2 去取物体垂直（-200mm 上面）
    ((0.720, -0.080), {'elbow_down': False, 'suction_vertical': -90, 'stop': True}),
    ((0.720, -0.100), {'elbow_down': False, 'suction_vertical': -90, 'stop': True}),
    ((0.720, -0.120), {'elbow_down': False, 'suction_vertical': -90, 'stop': True}),
    ],
    [#3 放到架子上
    ((0.030, 0.500), {'elbow_down': False, 'suction_vertical': 90, 'stop': True}),
    ((0.010, 0.500), {'elbow_down': False, 'suction_vertical': 90, 'stop': True}),
    ((0.010, 0.550), {'elbow_down': False, 'suction_vertical': 90, 'stop': True}),
    ((0.020, 0.600), {'elbow_down': False, 'suction_vertical': 90, 'stop': False}),
    ((0.200, 0.690), {'elbow_down': False, 'suction_vertical': 0, 'stop': False}),
    ((0.340, 0.690), {'elbow_down': False, 'suction_vertical': 0, 'stop': False}),
    ],
    [#4 回到初始位置1
    ((0.150, 0.550), {'elbow_down': False, 'suction_vertical': 80, 'stop': True}),
    ((0.150, 0.500), {'elbow_down': False, 'suction_vertical': 80, 'stop': True}),
    # ((0.220, 0.710), {'elbow_down': False, 'suction_vertical': 0, 'stop': False}),
    (178,-164,90)
    ],
    [#5 回到初始位置2
    ((0.475, 0.346), {'elbow_down': False, 'suction_vertical': 0, 'stop': True}),
    # ((0.220, 0.710), {'elbow_down': False, 'suction_vertical': 0, 'stop': False}),
    (178,-164,90)
    ]
]
arm = ThreeAxisManipulator()

print("\n// ==========================================")
print(f"// Generating Code for {OUTPUT_FILE_PATH}")
print("// ==========================================")

code_buffer = [CPP_HEADER]

action_names = []
last_waypoints_info = [] # For visualization of the last action
all_waypoints_info = [] # For visualization of all actions

for idx, action_points in enumerate(waypoints_data):
    waypoints_info = []
    
    # Process each point in the action
    for i, item in enumerate(action_points):
        # Format 1: Direct Joint Angles (q1, q2, q3) in degrees
        if isinstance(item, tuple) and len(item) == 3 and all(isinstance(v, (int, float)) for v in item):
            q1, q2, q3 = item
            conf_deg = [q1, q2, q3]
            conf_rad = [math.radians(a) for a in conf_deg]
            conf = [normalize_angle(a) for a in conf_rad]
            
            # Calculate FK for visualization
            (x1, y1), (x2, y2), (x3, y3) = arm.forward_kinematics(conf)
            
            waypoints_info.append({
                'conf': conf,
                'stop': True, # Default to stop for simple angle input
                'pos': (x3, y3),
                'action_idx': idx
            })
            continue

        # Format 2: Cartesian ((x,y), params)
        if isinstance(item, tuple) and len(item) == 2:
            pos, params = item
            x, y = pos
            elbow_down = params.get('elbow_down', False)
            suction_angle = params.get('suction_vertical', 0)
            should_stop = params.get('stop', True)
            
            # IK
            try:
                raw_conf = arm.inverse_kinematics(x, y, suction_angle_deg=suction_angle, elbow_down=elbow_down)
                # Normalize
                conf = [normalize_angle(a) for a in raw_conf]
                waypoints_info.append({'conf': conf, 'stop': should_stop, 'pos': pos, 'action_idx': idx})
            except Exception as e:
                print(f"[Error] Action {idx} Point {i} IK Failed: {e}")
            continue
            
        print(f"[Warning] Unknown point format in Action {idx}, Point {i}: {item}")

    if not waypoints_info:
        print(f"[Warning] Action {idx} produced no valid waypoints.")
        continue

    # Generate Name
    name = f"action_seq_{idx}"
    action_names.append(name)
    
    # Generate C++ Array string
    code_buffer.append(f"\n// --- Action {idx} ---")
    code_buffer.append(generate_cpp_array_str(name, waypoints_info))
    
    last_waypoints_info = waypoints_info
    all_waypoints_info.extend(waypoints_info)

# Generate Table Registration
code_buffer.append("\n// Action Table Registration")
code_buffer.append("static const ActionDef ACTION_TABLE[] = {")
for i, name in enumerate(action_names):
    code_buffer.append(f"    {{{name}, {name}_len}}, // ID {i}")
code_buffer.append("};")
code_buffer.append("static const int ACTION_TOTAL_COUNT = sizeof(ACTION_TABLE) / sizeof(ACTION_TABLE[0]);")

code_buffer.append(CPP_FOOTER)

# Write to file
try:
    with open(OUTPUT_FILE_PATH, 'w', encoding='utf-8') as f:
        f.write("\n".join(code_buffer))
    print(f"Successfully wrote code to {OUTPUT_FILE_PATH}")
except Exception as e:
    print(f"Error writing to file: {e}") 

# Visualization (Visualizes the last action in the list)
def visualize_trajectory(arm, waypoints_info):
    """可视化机械臂轨迹"""
    if not waypoints_info:
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    # 设置显示范围，根据机械臂尺寸调整
    limit = arm.arm1_length + arm.arm2_length + arm.suction_length + 0.1
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.grid(True)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Robotic Arm Sim - Last Action ({len(waypoints_info)} points)")

    # 绘制基座
    ax.plot(arm.base_x, arm.base_y, 'ks', markersize=12, label='Base')

    # arm_line: 机械臂连杆
    arm_line, = ax.plot([], [], 'o-', lw=3, markersize=8, color='b', label='Link')
    # path_line: 末端轨迹
    path_line, = ax.plot([], [], 'r--', lw=1, alpha=0.6, label='Tip Path')
    # target_points: 目标点
    target_x = [wp['pos'][0] for wp in waypoints_info]
    target_y = [wp['pos'][1] for wp in waypoints_info]
    ax.plot(target_x, target_y, 'gx', markersize=8, label='Target Waypoints')
    
    # Text for Waypoint index
    text_template = 'Action: %s | Waypoint: %d'
    text = ax.text(0.05, 0.95, '', transform=ax.transAxes)

    tip_x_list = []
    tip_y_list = []

    def init():
        arm_line.set_data([], [])
        path_line.set_data([], [])
        text.set_text('')
        return arm_line, path_line, text

    def update(frame):
        # 获取当前帧的关节角度
        wp = waypoints_info[frame]
        conf = wp['conf']
        act_idx = wp.get('action_idx', '?')
        # 计算正运动学
        (x1, y1), (x2, y2), (x3, y3) = arm.forward_kinematics(conf)
        
        # 连杆坐标: Base -> J1 -> J2 -> Tip
        xs = [arm.base_x, x1, x2, x3]
        ys = [arm.base_y, y1, y2, y3]
        
        arm_line.set_data(xs, ys)
        
        # 记录末端位置
        if frame < len(waypoints_info):
             if len(tip_x_list) <= frame:
                 tip_x_list.append(x3)
                 tip_y_list.append(y3)
        
        path_line.set_data(tip_x_list, tip_y_list)
        text.set_text(text_template % (str(act_idx), frame))
        return arm_line, path_line, text

    # interval=1000ms 切换一个点
    ani = animation.FuncAnimation(fig, update, frames=len(waypoints_info),
                                  init_func=init, blit=True, interval=800, repeat=True)
    
    plt.legend()
    plt.show()

print("\n--- Visualizing ALL Trajectories ---")
visualize_trajectory(arm, all_waypoints_info)

