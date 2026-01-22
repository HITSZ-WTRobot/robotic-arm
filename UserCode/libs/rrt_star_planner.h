#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define PI 3.14159265358979323846f
// 警告：STM32F407 SRAM有限(128+64KB)。
// 10000个节点需要 ~520KB RAM，会导致 malloc 失败。
// 降至 2000 个节点 (~100KB)，需确保堆空间(Heap)足够大。
#define MAX_NODES 2000
#define MAX_OBSTACLES 100

// 二维点结构
typedef struct {
    float x, y;
} Point2D;

// 障碍物结构（圆形）
typedef struct {
    Point2D center;
    float radius;
} Obstacle;

// 机械臂配置结构体
typedef struct {
    float link_lengths[3];    // 三个连杆长度
    float joint_limits[3][2]; // 三个关节的角度限制 [min, max]
    Point2D base_position;     // 基座坐标
    float link_width;         // 连杆宽度（用于碰撞检测）
    float safety_margin;      // 安全距离
} RobotConfig;

// 路径点结构体
typedef struct {
    float joint_angles[3];    // 三个关节角度 [rad]
    Point2D joint_positions[4]; // 四个关节位置（包括基座和末端）
    float cost;               // 到达该点的代价
    int parent_index;          // 父节点索引
} Waypoint;

// 规划结果结构体
typedef struct {
    Waypoint* path;            // 路径点数组
    int path_length;           // 路径长度
    int nodes_generated;       // 生成的节点总数
    float total_cost;         // 总代价
    int success;               // 规划是否成功
} PlanningResult;

// RRT*规划器结构体
typedef struct {
    RobotConfig robot_config;
    float step_size;          // 扩展步长
    float goal_bias;          // 目标偏向概率
    int max_iterations;        // 最大迭代次数
    float goal_tolerance;     // 目标容差
    float search_radius;      // 重布线搜索半径
    
    // 障碍物信息
    Obstacle obstacles[MAX_OBSTACLES];
    int obstacle_count;
    float safety_margin;      // 全局安全距离
} RRTStarPlanner;

// 函数声明

// 初始化规划器
RRTStarPlanner* init_planner(float link1, float link2, float link3, 
                            float base_x, float base_y, float safety_margin);

// 设置关节限制
void set_joint_limits(RRTStarPlanner* planner, 
                     float joint1_min, float joint1_max,
                     float joint2_min, float joint2_max, 
                     float joint3_min, float joint3_max);

// 设置连杆宽度
void set_link_width(RRTStarPlanner* planner, float width);

// 添加障碍物
void add_obstacle(RRTStarPlanner* planner, float center_x, float center_y, float radius);

// 设置规划参数
void set_planning_parameters(RRTStarPlanner* planner, 
                           float step_size, float goal_bias, 
                           int max_iter, float tolerance);

// 正运动学计算：关节角度 -> 所有关节位置
void forward_kinematics(const RRTStarPlanner* planner, 
                       const float angles[3], Point2D positions[4]);

// 改进的碰撞检测：检查机械臂所有连杆与障碍物的碰撞
int check_collision(const RRTStarPlanner* planner, const float angles[3]);

// 距离计算
float calculate_distance(const float angles1[3], const float angles2[3]);

// 核心规划函数
PlanningResult* plan_path(RRTStarPlanner* planner, 
                         const float start_angles[3], 
                         const float goal_angles[3]);

// 平滑路径
PlanningResult* smooth_path(const RRTStarPlanner* planner, 
                          const PlanningResult* original_path);

// 释放资源
void free_planning_result(PlanningResult* result);
void free_planner(RRTStarPlanner* planner);

// 几何工具函数
float point_distance(Point2D p1, Point2D p2);
float segment_point_distance(Point2D seg_start, Point2D seg_end, Point2D point);
int segment_circle_intersection(Point2D seg_start, Point2D seg_end, 
                               Point2D circle_center, float circle_radius, 
                               float safety_margin);

// 工具函数
float random_float(float min, float max);
float normalize_angle(float angle);

// 整合接口函数
Waypoint* Get_Path(float start_angles[3], float goal_angles[3], 
                   RRTStarPlanner* planner, int* out_path_length);

#ifdef __cplusplus
}
#endif

#endif