---
title: TEB参数解析
date: 2023-8-11 17:49:03
tags:
  - ROS
  - TEB
categories:
  - 机器人
---
[teb参数含义](https://charon-cheung.github.io/2021/06/07/%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92/TEB%E7%AE%97%E6%B3%95/TEB%E6%89%80%E6%9C%89%E5%8F%82%E6%95%B0%E5%90%AB%E4%B9%89/#%E8%AF%9D%E9%A2%98)
```bash
 #*******************************************************************************
 # Trajectory
 #*******************************************************************************
 teb_autosize: True #优化期间允许改变轨迹的时域长度
 #TEB通过状态搜索树寻找最优路径，而dt_ref则是最优路径上的两个相邻姿态
 #（即位置、速度、航向信息，可通过TEB可视化在rivz中看到）的默认距离
 #此距离不固定，规划器自动根据速度大小调整这一距离，速度越大，相邻距离自然越大，
 #较小的值理论上可提供更高精度
 dt_ref: 0.3               #局部路径规划的解析度
 dt_hysteresis: 0.1        #允许改变的时域解析度的浮动范围， 一般为 dt_ref 的 10% 左右;
 max_samples: 200          #
 global_plan_overwrite_orientation: True      #覆盖全局路径中局部路径点的朝向
 allow_init_with_backwards_motion: False      #允许在开始时想后退来执行轨迹
 #考虑优化的全局计划子集的最大长度（累积欧几里得距离）
 #如果为0或负数：禁用；长度也受本地Costmap大小的限制
 max_global_plan_lookahead_dist: 2.5
 global_plan_viapoint_sep: -1  #从全局路径中选取的每两个连续通过点之间的最小间隔
 global_plan_prune_distance: 1  #该参数决定了从机器人当前位置的后面一定距离开始裁剪
 #如果为真，规划器在速度、加速度和转弯率计算中使用精确的弧长[增加的CPU时间]，否则使用欧几里德近似。
 exact_arc_length: False
 #在判断生成的轨迹是否冲突时使用，此时设置为3，即从轨迹起点开始逐个检查轨迹上的3个点，
 #若3个点均不发生碰撞，则认为本次轨迹有效。若小于0则检查所有路径点
 feasibility_check_no_poses: 3   #检测位姿可到达的时间间隔
 publish_feedback: False        #发布包含完整轨迹和活动障碍物列表的规划器反馈

#*******************************************************************************
  # Robot
#*******************************************************************************
 max_vel_x: 0.3             #最大x前向速度
 max_vel_x_backwards: 0.05  #最大x后退速度
 max_vel_y: 0.0             #最大y方向速度
 max_vel_theta: 0.25        #最大转向角速度
 acc_lim_x: 0.15            #最大x加速度
 acc_lim_theta: 0.15        #最大角速度
 min_turning_radius: 0.0    #车类机器人的最小转弯半径


 # types: "point", "circular", "two_circles", "line", "polygon"
 footprint_model: 
#type: "polygon"    #多边形
#vertices: [[-0.3, -0.2], [0.3, -0.2], [0.3, 0.2], [-0.3, 0.2]]
  type: "circular"   #圆
  radius: 0.25 # for type "circular"
 #line_start: [0.0, 0.0] # for type "line" #直线
 #line_end: [0.4, 0.0] # for type "line"
 #front_offset: 0.2 # for type "two_circles" 
 #front_radius: 0.2 # for type "two_circles"
 #rear_offset: 0.2 # for type "two_circles"
 #rear_radius: 0.2 # for type "two_circles"


 #*******************************************************************************
   # Obstacles
 #*******************************************************************************
 min_obstacle_dist: 0.22                     #和障碍物最小距离
 inflation_dist: 0.2                         #障碍物膨胀距离
 include_costmap_obstacles: True             #costmap 中的障碍物是否被直接考虑
 costmap_obstacles_behind_robot_dist: 0.5    #规划时考虑后面n米内的障碍物
 obstacle_poses_affected: 10                 #障碍物姿态受影响程度0-30

 dynamic_obstacle_inflation_dist: 0.4      #动态障碍物的膨胀范围
 include_dynamic_obstacles: False          #是否将动态障碍物预测为速度模型


#*******************************************************************************
   # Optimization
 #*******************************************************************************
 no_inner_iterations: 5         #被外循环调用后内循环执行优化次数
 no_outer_iterations: 4         #执行的外循环的优化次数执行的外循环的优化次数
 optimization_activate: True    #激活优化
 optimization_verbose: False    #打印优化过程详情
 penalty_epsilon: 0.1           #对于硬约束近似，在惩罚函数中添加安全范围
 obstacle_cost_exponent: 4
 weight_max_vel_x: 2            #最大x速度权重 0~2
 weight_max_vel_theta: 1        #最大角速度权重 0~1
 weight_acc_lim_x: 1            #最大x 加速度权重 0~1
 weight_acc_lim_theta: 1        #最大角速度权重 0~1
 weight_kinematics_nh: 1000     #非完整运动学的优化权重
 weight_kinematics_forward_drive: 5     #优化过程中，迫使机器人只选择前进方向，差速轮适用
 weight_kinematics_turning_radius: 1    #优化过程中，车型机器人的最小转弯半径的权重
 weight_optimaltime: 0.3 # must be > 0  #优化过程中，基于轨迹的时间上的权重
 weight_shortest_path: 0
 weight_obstacle: 100           #优化过程中，和障碍物最小距离的权重 0~50
 weight_inflation: 0.3          #优化过程中， 膨胀区的权重
 weight_dynamic_obstacle: 10    #优化过程中，和动态障碍物最小距离的权重
 weight_dynamic_obstacle_inflation: 0.2  #优化过程中，和动态障碍物膨胀区的权重 0~50
 weight_viapoint: 1             #优化过程中，和全局路径采样点距离的权重
 weight_adapt_factor: 2

 #*******************************************************************************
 # Homotopy Class Planner
 #*******************************************************************************
 enable_homotopy_class_planning: true   #激活并行规划
 enable_multithreading: True            #允许多线程并行处理
 max_number_classes: 3                  #考虑到的不同轨迹的最大数量
 selection_cost_hysteresis: 1.0
 selection_prefer_initial_plan: 0.9
 selection_obst_cost_scale: 100.0
 selection_alternative_time_cost: False

 roadmap_graph_no_samples: 15       #指定为创建路线图而生成的样本数
 roadmap_graph_area_width: 6        #指定该区域的宽度
 roadmap_graph_area_length_scale: 1.0
 h_signature_prescaler: 0.5         #（0.2 < value <= 1）缩放用于区分同伦类的内部参数
 h_signature_threshold: 0.1         #如果实部和复部的差都低于规定的阈值，则假定两个h签名相等。
 obstacle_heading_threshold: 0.45   #在障碍物航向和目标航向之间指定标量乘积的值，以便将障碍物考虑在内进行探索
 switching_blocking_period: 0.0     #指定允许切换到新的等效类之前需要终止的持续时间
 #为true，则将不同拓扑的所有轨迹附加到该组通孔点，否
 #则仅将与初始/全局计划共享相同拓扑的轨迹与它们连接 (对test_optim_node无效)
 viapoints_all_candidates: True 
 delete_detours_backwards: True 
 max_ratio_detours_duration_best_duration: 3.0
 visualize_hc_graph: False          #可视化创建的图形，用于探索不同的轨迹
 visualize_with_time_as_z_axis_scale: False  #在rviz里可看到优化使用的graph

 #*******************************************************************************
 # Recovery
 #*******************************************************************************
 #当规划器检测到系统异常，允许缩小时域规划范围,TEB将以更近的点作为规划目标，尝试重新规划出可行路径;
 shrink_horizon_backup: false         
 shrink_horizon_min_duration: 10  #如果检测到不可行的轨迹，激活缩小的水平线后备模式，本参数为其最短持续时间。
 oscillation_recovery: True       #尝试检测和解决振荡
oscillation_v_eps: 0.1          #(0,1)内的 normalized 线速度的平均值的阈值，判断机器人是否运动异常
oscillation_omega_eps: 0.1      #(0,1)内的 normalized 角速度的平均值，判断机器人是否运动异常
oscillation_recovery_min_duration: 10  #在这个时间内，是否再次发生FailureDetector检测的振荡
oscillation_filter_duration: 10  #failure_detector_中buffer容器的大小为oscillation_filter_duration * controller_frequency
```