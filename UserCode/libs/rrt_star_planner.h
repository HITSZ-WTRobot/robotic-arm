#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

#define PI 3.14159265358979323846
#define MAX_NODES 10000
#define MAX_OBSTACLES 100

// 二维点结构
typedef struct {
    double x, y;
} Point2D;

// 障碍物结构（圆形）
typedef struct {
    Point2D center;
    double radius;
} Obstacle;

// 机械臂配置结构体
typedef struct {
    double link_lengths[3];    // 三个连杆长度
    double joint_limits[3][2]; // 三个关节的角度限制 [min, max]
    Point2D base_position;     // 基座坐标
    double link_width;         // 连杆宽度（用于碰撞检测）
    double safety_margin;      // 安全距离
} RobotConfig;

// 路径点结构体
typedef struct {
    double joint_angles[3];    // 三个关节角度 [rad]
    Point2D joint_positions[4]; // 四个关节位置（包括基座和末端）
    double cost;               // 到达该点的代价
    int parent_index;          // 父节点索引
} Waypoint;

// 规划结果结构体
typedef struct {
    Waypoint* path;            // 路径点数组
    int path_length;           // 路径长度
    int nodes_generated;       // 生成的节点总数
    double total_cost;         // 总代价
    int success;               // 规划是否成功
} PlanningResult;

// RRT*规划器结构体
typedef struct {
    RobotConfig robot_config;
    double step_size;          // 扩展步长
    double goal_bias;          // 目标偏向概率
    int max_iterations;        // 最大迭代次数
    double goal_tolerance;     // 目标容差
    double search_radius;      // 重布线搜索半径
    
    // 障碍物信息
    Obstacle obstacles[MAX_OBSTACLES];
    int obstacle_count;
    double safety_margin;      // 全局安全距离
} RRTStarPlanner;

// 函数声明

// 初始化规划器
RRTStarPlanner* init_planner(double link1, double link2, double link3, 
                            double base_x, double base_y, double safety_margin);

// 设置关节限制
void set_joint_limits(RRTStarPlanner* planner, 
                     double joint1_min, double joint1_max,
                     double joint2_min, double joint2_max, 
                     double joint3_min, double joint3_max);

// 设置连杆宽度
void set_link_width(RRTStarPlanner* planner, double width);

// 添加障碍物
void add_obstacle(RRTStarPlanner* planner, double center_x, double center_y, double radius);

// 设置规划参数
void set_planning_parameters(RRTStarPlanner* planner, 
                           double step_size, double goal_bias, 
                           int max_iter, double tolerance);

// 正运动学计算：关节角度 -> 所有关节位置
void forward_kinematics(const RRTStarPlanner* planner, 
                       const double angles[3], Point2D positions[4]);

// 改进的碰撞检测：检查机械臂所有连杆与障碍物的碰撞
int check_collision(const RRTStarPlanner* planner, const double angles[3]);

// 距离计算
double calculate_distance(const double angles1[3], const double angles2[3]);

// 核心规划函数
PlanningResult* plan_path(RRTStarPlanner* planner, 
                         const double start_angles[3], 
                         const double goal_angles[3]);

// 平滑路径
PlanningResult* smooth_path(const RRTStarPlanner* planner, 
                          const PlanningResult* original_path);

// 释放资源
void free_planning_result(PlanningResult* result);
void free_planner(RRTStarPlanner* planner);

// 几何工具函数
double point_distance(Point2D p1, Point2D p2);
double segment_point_distance(Point2D seg_start, Point2D seg_end, Point2D point);
int segment_circle_intersection(Point2D seg_start, Point2D seg_end, 
                               Point2D circle_center, double circle_radius, 
                               double safety_margin);

// 工具函数
double random_double(double min, double max);
double normalize_angle(double angle);

#endif