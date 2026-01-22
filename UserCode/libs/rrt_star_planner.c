#include "rrt_star_planner.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 初始化规划器
RRTStarPlanner* init_planner(float link1, float link2, float link3, 
                           float base_x, float base_y, float safety_margin) 
{
    RRTStarPlanner* planner = (RRTStarPlanner*)malloc(sizeof(RRTStarPlanner));
    
    // 设置机械臂参数
    planner->robot_config.link_lengths[0] = link1;
    planner->robot_config.link_lengths[1] = link2;
    planner->robot_config.link_lengths[2] = link3;
    
    planner->robot_config.base_position.x = base_x;
    planner->robot_config.base_position.y = base_y;
    planner->robot_config.safety_margin = safety_margin;
    planner->robot_config.link_width = 0.05; // 默认连杆宽度
    
    // 默认关节限制
    set_joint_limits(planner, -PI, PI, -PI, PI, -PI, PI);
    
    // 默认规划参数
    set_planning_parameters(planner, 0.1, 0.1, 5000, 0.05);
    
    // 初始化障碍物
    planner->obstacle_count = 0;
    planner->safety_margin = safety_margin;
    
    // srand((unsigned int)time(NULL)); // 嵌入式系统默认无 RTC 支持 time()
    srand(0); //由于工程配置原因，暂时使用固定随机数种子
    
    return planner;
}

// 设置关节限制
void set_joint_limits(RRTStarPlanner* planner, 
                     float joint1_min, float joint1_max,
                     float joint2_min, float joint2_max, 
                     float joint3_min, float joint3_max) {
    planner->robot_config.joint_limits[0][0] = joint1_min;
    planner->robot_config.joint_limits[0][1] = joint1_max;
    planner->robot_config.joint_limits[1][0] = joint2_min;
    planner->robot_config.joint_limits[1][1] = joint2_max;
    planner->robot_config.joint_limits[2][0] = joint3_min;
    planner->robot_config.joint_limits[2][1] = joint3_max;
}

// 设置连杆宽度
void set_link_width(RRTStarPlanner* planner, float width) {
    planner->robot_config.link_width = width;
}

// 添加障碍物
void add_obstacle(RRTStarPlanner* planner, float center_x, float center_y, float radius) {
    if (planner->obstacle_count < MAX_OBSTACLES) {
        planner->obstacles[planner->obstacle_count].center.x = center_x;
        planner->obstacles[planner->obstacle_count].center.y = center_y;
        planner->obstacles[planner->obstacle_count].radius = radius;
        planner->obstacle_count++;
    }
}

// 设置规划参数
void set_planning_parameters(RRTStarPlanner* planner, 
                           float step_size, float goal_bias, 
                           int max_iter, float tolerance) {
    planner->step_size = step_size;
    planner->goal_bias = goal_bias;
    planner->max_iterations = max_iter;
    planner->goal_tolerance = tolerance;
    planner->search_radius = step_size * 2.0;
}

// 正运动学计算
void forward_kinematics(const RRTStarPlanner* planner, 
                       const float angles[3], Point2D positions[4]) {
    // 基座位置
    positions[0] = planner->robot_config.base_position;
    
    // 第一关节
    positions[1].x = positions[0].x + planner->robot_config.link_lengths[0] * cosf(angles[0]);
    positions[1].y = positions[0].y + planner->robot_config.link_lengths[0] * sinf(angles[0]);
    
    // 第二关节
    positions[2].x = positions[1].x + planner->robot_config.link_lengths[1] * cosf(angles[0] + angles[1]);
    positions[2].y = positions[1].y + planner->robot_config.link_lengths[1] * sinf(angles[0] + angles[1]);
    
    // 末端执行器（第三关节）
    positions[3].x = positions[2].x + planner->robot_config.link_lengths[2] * cosf(angles[0] + angles[1] + angles[2]);
    positions[3].y = positions[2].y + planner->robot_config.link_lengths[2] * sinf(angles[0] + angles[1] + angles[2]);
}

// 两点距离计算
float point_distance(Point2D p1, Point2D p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrtf(dx*dx + dy*dy);
}

// 计算点到线段的距离
float segment_point_distance(Point2D seg_start, Point2D seg_end, Point2D point) {
    float segment_length_sq = (seg_end.x - seg_start.x) * (seg_end.x - seg_start.x) + 
                              (seg_end.y - seg_start.y) * (seg_end.y - seg_start.y);
    
    if (segment_length_sq == 0.0) {
        return point_distance(seg_start, point);
    }
    
    // 计算投影参数t
    float t = ((point.x - seg_start.x) * (seg_end.x - seg_start.x) + 
                (point.y - seg_start.y) * (seg_end.y - seg_start.y)) / segment_length_sq;
    
    t = fmaxf(0.0, fminf(1.0, t)); // 限制t在[0,1]范围内
    
    // 计算投影点
    Point2D projection;
    projection.x = seg_start.x + t * (seg_end.x - seg_start.x);
    projection.y = seg_start.y + t * (seg_end.y - seg_start.y);
    
    return point_distance(point, projection);
}

// 检查线段与圆的相交（考虑安全距离）
int segment_circle_intersection(Point2D seg_start, Point2D seg_end, 
                               Point2D circle_center, float circle_radius, 
                               float safety_margin) {
    float effective_radius = circle_radius + safety_margin;
    float dist = segment_point_distance(seg_start, seg_end, circle_center);
    
    return dist <= effective_radius;
}

// 改进的碰撞检测：检查所有连杆与障碍物的碰撞
int check_collision(const RRTStarPlanner* planner, const float angles[3]) {
    Point2D positions[4];
    forward_kinematics(planner, angles, positions);
    
    // 检查每个连杆与所有障碍物的碰撞
    for (int link = 0; link < 3; link++) {
        Point2D start_pos = positions[link];
        Point2D end_pos = positions[link + 1];
        
        for (int i = 0; i < planner->obstacle_count; i++) {
            Obstacle obs = planner->obstacles[i];
            
            // 检查线段与障碍物的碰撞（考虑安全距离）
            if (segment_circle_intersection(start_pos, end_pos, obs.center, 
                                           obs.radius, planner->safety_margin)) {
                return 1; // 碰撞
            }
        }
    }
    
    return 0; // 无碰撞
}

// 关节空间距离计算
float calculate_distance(const float angles1[3], const float angles2[3]) {
    float distance = 0.0;
    for (int i = 0; i < 3; i++) {
        float diff = angles1[i] - angles2[i];
        // 处理角度环绕
        if (diff > PI) diff -= 2*PI;
        if (diff < -PI) diff += 2*PI;
        distance += diff * diff;
    }
    return sqrtf(distance);
}

// RRT*核心算法
PlanningResult* plan_path(RRTStarPlanner* planner, 
                         const float start_angles[3], 
                         const float goal_angles[3]) {
    PlanningResult* result = (PlanningResult*)malloc(sizeof(PlanningResult));
    if (!result) return NULL; // 内存分配失败

    result->success = 0;
    result->path = NULL;
    result->path_length = 0;

    Waypoint* tree = (Waypoint*)malloc(MAX_NODES * sizeof(Waypoint));
    if (!tree) {
        free(result);
        return NULL; // 内存不足，防止 Hard Fault
    }

    int node_count = 0;
    
    // 初始化起始点
    Waypoint start_node;
    memcpy(start_node.joint_angles, start_angles, 3 * sizeof(float));
    forward_kinematics(planner, start_angles, start_node.joint_positions);
    start_node.cost = 0.0;
    start_node.parent_index = -1;
    
    tree[node_count++] = start_node;
    
    // PlanningResult* result = (PlanningResult*)malloc(sizeof(PlanningResult));
    result->success = 0;
    result->path = NULL;
    result->path_length = 0;
    
    for (int iter = 0; iter < planner->max_iterations; iter++) {
        // 随机采样（带有目标偏向）
        float random_angles[3];
        if ((float)rand() / RAND_MAX < planner->goal_bias) {
            memcpy(random_angles, goal_angles, 3 * sizeof(float));
        } else {
            for (int i = 0; i < 3; i++) {
                random_angles[i] = random_float(
                    planner->robot_config.joint_limits[i][0],
                    planner->robot_config.joint_limits[i][1]);
            }
        }
        
        // 寻找最近节点
        int nearest_idx = 0;
        float min_dist = calculate_distance(tree[0].joint_angles, random_angles);
        for (int i = 1; i < node_count; i++) {
            float dist = calculate_distance(tree[i].joint_angles, random_angles);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        // 向随机点方向扩展
        Waypoint new_node;
        float direction[3];
        for (int i = 0; i < 3; i++) {
            direction[i] = random_angles[i] - tree[nearest_idx].joint_angles[i];
        }
        
        // 归一化方向向量
        float norm = min_dist;
        if (norm > planner->step_size) {
            for (int i = 0; i < 3; i++) {
                new_node.joint_angles[i] = tree[nearest_idx].joint_angles[i] + 
                    (direction[i] / norm) * planner->step_size;
                
                // 确保角度在限制范围内
                new_node.joint_angles[i] = fmaxf(planner->robot_config.joint_limits[i][0],
                    fminf(new_node.joint_angles[i], planner->robot_config.joint_limits[i][1]));
            }
        } else {
            memcpy(new_node.joint_angles, random_angles, 3 * sizeof(float));
        }
        
        // 改进的碰撞检测
        if (check_collision(planner, new_node.joint_angles)) {
            continue;
        }
        
        forward_kinematics(planner, new_node.joint_angles, new_node.joint_positions);
        
        // RRT*: 在搜索半径内寻找最优父节点
        float min_cost = tree[nearest_idx].cost + 
            calculate_distance(tree[nearest_idx].joint_angles, new_node.joint_angles);
        int best_parent = nearest_idx;
        
        for (int i = 0; i < node_count; i++) {
            float dist = calculate_distance(tree[i].joint_angles, new_node.joint_angles);
            if (dist <= planner->search_radius) {
                float new_cost = tree[i].cost + dist;
                if (new_cost < min_cost) {
                    // 检查路径是否无碰撞
                    int collision_detected = 0;
                    float test_angles[3];
                    float step = 0.1;
                    int steps = (int)(dist / step);
                    
                    // 插值检查路径上的碰撞
                    for (int s = 1; s <= steps; s++) {
                        float t = (float)s / steps;
                        for (int j = 0; j < 3; j++) {
                            test_angles[j] = tree[i].joint_angles[j] + 
                                            t * (new_node.joint_angles[j] - tree[i].joint_angles[j]);
                        }
                        if (check_collision(planner, test_angles)) {
                            collision_detected = 1;
                            break;
                        }
                    }
                    
                    if (!collision_detected) {
                        min_cost = new_cost;
                        best_parent = i;
                    }
                }
            }
        }
        
        new_node.cost = min_cost;
        new_node.parent_index = best_parent;
        
        // 添加到树中
        if (node_count < MAX_NODES) {
            tree[node_count++] = new_node;
        }
        
        // RRT*: 重布线
        for (int i = 0; i < node_count - 1; i++) {
            float dist = calculate_distance(tree[i].joint_angles, new_node.joint_angles);
            if (dist <= planner->search_radius) {
                float alternative_cost = new_node.cost + dist;
                if (alternative_cost < tree[i].cost) {
                    // 检查路径是否无碰撞
                    int collision_detected = 0;
                    float test_angles[3];
                    float step = 0.1;
                    int steps = (int)(dist / step);
                    
                    for (int s = 1; s <= steps; s++) {
                        float t = (float)s / steps;
                        for (int j = 0; j < 3; j++) {
                            test_angles[j] = new_node.joint_angles[j] + 
                                            t * (tree[i].joint_angles[j] - new_node.joint_angles[j]);
                        }
                        if (check_collision(planner, test_angles)) {
                            collision_detected = 1;
                            break;
                        }
                    }
                    
                    if (!collision_detected) {
                        tree[i].cost = alternative_cost;
                        tree[i].parent_index = node_count - 1;
                    }
                }
            }
        }
        
        // 检查是否到达目标
        float goal_dist = calculate_distance(new_node.joint_angles, goal_angles);
        if (goal_dist <= planner->goal_tolerance) {
            // 找到路径，回溯
            int path_length = 1;
            int current_idx = node_count - 1;
            
            while (current_idx != -1) {
                path_length++;
                current_idx = tree[current_idx].parent_index;
            }
            
            result->path = (Waypoint*)malloc(path_length * sizeof(Waypoint));
            result->path_length = path_length;
            
            current_idx = node_count - 1;
            for (int i = path_length - 1; i >= 0; i--) {
                if (current_idx == -1) break;
                result->path[i] = tree[current_idx];
                current_idx = tree[current_idx].parent_index;
            }
            
            // 添加目标点
            Waypoint goal_node;
            memcpy(goal_node.joint_angles, goal_angles, 3 * sizeof(float));
            forward_kinematics(planner, goal_angles, goal_node.joint_positions);
            goal_node.cost = new_node.cost + goal_dist;
            goal_node.parent_index = path_length - 2;
            result->path[path_length - 1] = goal_node;
            
            result->success = 1;
            result->nodes_generated = node_count;
            result->total_cost = goal_node.cost;
            
            free(tree);
            return result;
        }
    }
    
    free(tree);
    return result;
}

// 平滑路径
PlanningResult* smooth_path(const RRTStarPlanner* planner, 
                          const PlanningResult* original_path) {
    if (!original_path || original_path->path_length < 3) {
        return NULL;
    }
    
    PlanningResult* smoothed = (PlanningResult*)malloc(sizeof(PlanningResult));
    smoothed->path_length = original_path->path_length;
    smoothed->path = (Waypoint*)malloc(smoothed->path_length * sizeof(Waypoint));
    
    memcpy(smoothed->path, original_path->path, 
           smoothed->path_length * sizeof(Waypoint));
    
    // 简单的插值平滑
    for (int i = 1; i < smoothed->path_length - 1; i++) {
        for (int j = 0; j < 3; j++) {
            smoothed->path[i].joint_angles[j] = 
                0.3 * smoothed->path[i-1].joint_angles[j] + 
                0.4 * smoothed->path[i].joint_angles[j] + 
                0.3 * smoothed->path[i+1].joint_angles[j];
        }
        forward_kinematics(planner, smoothed->path[i].joint_angles, 
                          smoothed->path[i].joint_positions);
    }
    
    smoothed->success = 1;
    smoothed->nodes_generated = original_path->nodes_generated;
    smoothed->total_cost = original_path->total_cost;
    
    return smoothed;
}

// 工具函数
float random_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

float normalize_angle(float angle) {
    while (angle > PI) angle -= 2*PI;
    while (angle < -PI) angle += 2*PI;
    return angle;
}

// 释放资源
void free_planning_result(PlanningResult* result) {
    if (result && result->path) {
        free(result->path);
    }
    free(result);
}

void free_planner(RRTStarPlanner* planner) {
    free(planner);
}



Waypoint* Get_Path(float start_angles[3], float goal_angles[3], RRTStarPlanner* planner, int *path_length)
{
    Waypoint *path_buf = NULL;
    *path_length = 0;

    // 执行路径规划
    PlanningResult* result = plan_path(planner, start_angles, goal_angles);

    if (result && result->success)
    {
        // 只有当路径长度大于等于3时才进行平滑，否则直接使用原路径
        // 因为 smooth_path 在路径过短时会返回 NULL
        if (result->path_length >= 3) 
        {
            // 平滑路径处理
            PlanningResult* smoothed = smooth_path(planner, result);
            if (smoothed)
            {
                path_buf = smoothed->path;
                *path_length = smoothed->path_length;
                
                // 关键修正：将 path 指针置空，避免 free_planning_result 释放我们需要返回的内存
                smoothed->path = NULL;
                free_planning_result(smoothed);
            }
            else 
            {
                // 如果平滑失败（例如内存不足），回退使用原始路径
                goto use_original_path;
            }
        }
        else
        {
use_original_path:
            // 路径太短或平滑失败，直接复制原始路径
            // 注意：因为 result 最后会被 free，我们需要将 path 的所有权接管过来
            // 或者深拷贝一份。这里选择接管所有权。
            path_buf = result->path;
            *path_length = result->path_length;
            result->path = NULL; // 断开指针，防止被 free_planning_result 释放
        }
    }
    
    // 清理资源
    if (result) {
        free_planning_result(result);
    }
    
    return path_buf;
}

#ifdef __cplusplus
}
#endif