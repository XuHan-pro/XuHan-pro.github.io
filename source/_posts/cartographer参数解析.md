---
title: cartographer参数解析
date: 2023-8-12 17:49:03
tags:
  - ROS
  - cartographer
categories:
  - cartographer
---
- [目录结构](#目录结构)
- [参数解析](#参数解析)
  - [基本参数](#基本参数)
  - [前端参数](#前端参数)
  - [后端参数](#后端参数)
- [参数调整经验](#参数调整经验)
  - [准备工作](#准备工作)
  - [前端参数](#前端参数-1)
  - [后端参数](#后端参数-1)
  - [系统实时性调整](#系统实时性调整)
- [地图保存脚本](#地图保存脚本)

# 目录结构
自定义配置文件：src/cartographer_ros/cartographer_ros/configuration_files/config.lua  
前端配置文件：src/cartographer/configuration_files/trajectory_builder_2d.lua  
后端配置文件： src/cartographer/configuration_files/pose_graph.lua  
# 参数解析
## 基本参数
```
  map_frame = "map",    --生成的地图坐标系
  tracking_frame = "base_link",    --跟踪的坐标系，可以是imu、小车、雷达
  published_frame = "base_link",    --cartographer正在发布pose的坐标，一般就是小车
  odom_frame = "odom",    --cartographer的里程计坐标系
  provide_odom_frame = false,--    cartographer是否发布里程计坐标
  publish_frame_projected_to_2d = true,    --是否无滚动、俯仰或z偏移
  use_odometry = false,    --订阅里程计
  use_nav_sat = false,    --订阅GPS
  use_landmarks = false,    --订阅路标
  num_laser_scans = 1,    --订阅雷达格式以及数量
  num_multi_echo_laser_scans = 0,    --订阅雷达格式以及数量
  num_subdivisions_per_laser_scan = 1,    --分割扫描点云
  num_point_clouds = 0,    --订阅雷达格式以及数量
  lookup_transform_timeout_sec = 0.2,   --tf2查找变换超时（s）
  submap_publish_period_sec = 0.3,    --发布子图实时间间隔（s）
  pose_publish_period_sec = 5e-3,    --发布pose时间间隔（s）
  trajectory_publish_period_sec = 30e-3,   --发布轨迹标记间隔（s，这里为30ms）
  rangefinder_sampling_ratio = 1.,    --以下5个参数为传感器（里程计、位姿、imu、反光板）采样比例
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

```
## 前端参数
```

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,            -- 是否使用imu数据
  min_range = 0.,                 -- 雷达数据的最远最近滤波, 保存中间值
  max_range = 30.,
  min_z = -0.8,                   -- 雷达数据的最高与最低的过滤, 保存中间值
  max_z = 2.,
  missing_data_ray_length = 5.,   -- 超过最大距离范围的数据点用这个距离代替
  num_accumulated_range_data = 1, -- 几帧有效的点云数据进行一次扫描匹配
  voxel_filter_size = 0.025,      -- 体素滤波的立方体的边长

  -- 使用固定的voxel滤波之后, 再使用自适应体素滤波器
  -- 体素滤波器 用于生成稀疏点云 以进行 扫描匹配
  adaptive_voxel_filter = {
    max_length = 0.5,             -- 尝试确定最佳的立方体边长, 边长最大为0.5
    min_num_points = 200,         -- 如果存在 较多点 并且大于'min_num_points', 则减小体素长度以尝试获得该最小点数
    max_range = 50.,              -- 距远离原点超过max_range 的点被移除
  },

  -- 闭环检测的自适应体素滤波器, 用于生成稀疏点云 以进行 闭环检测
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  -- 是否使用 real_time_correlative_scan_matcher 为ceres提供先验信息
  -- 计算复杂度高 , 但是很鲁棒 , 在odom或者imu不准时依然能达到很好的效果
  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,             -- 线性搜索窗口的大小
    angular_search_window = math.rad(20.),  -- 角度搜索窗口的大小
    translation_delta_cost_weight = 1e-1,   -- 用于计算各部分score的权重
    rotation_delta_cost_weight = 1e-1,
  },

  -- ceres匹配的一些配置参数
  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  -- 为了防止子图里插入太多数据, 在插入子图之前之前对数据进行过滤
  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,

  -- 位姿预测器
  pose_extrapolator = {
    use_imu_based = false,
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
    imu_based = {
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  -- 子图相关的一些配置
  submaps = {
    num_range_data = 90,          -- 一个子图里插入雷达数据的个数的一半
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID", -- 地图的种类, 还可以是tsdf格式
      resolution = 0.05,
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      -- 概率占用栅格地图的一些配置
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      -- tsdf地图的一些配置
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}
```
## 后端参数
```

POSE_GRAPH = {
  -- 每隔多少个节点执行一次后端优化
  optimize_every_n_nodes = 90,

  -- 约束构建的相关参数
  constraint_builder = {
    sampling_ratio = 0.3,                 -- 对局部子图进行回环检测时的计算频率, 数值越大, 计算次数越多
    max_constraint_distance = 15.,        -- 对局部子图进行回环检测时能成为约束的最大距离
    min_score = 0.55,                     -- 对局部子图进行回环检测时的最低分数阈值
    global_localization_min_score = 0.6,  -- 对整体子图进行回环检测时的最低分数阈值
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = true,                   -- 打印约束计算的log
    
    -- 基于分支定界算法的2d粗匹配器
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },

    -- 基于ceres的2d精匹配器
    ceres_scan_matcher = {
      occupied_space_weight = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },

    -- 基于分支定界算法的3d粗匹配器
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },

    -- 基于ceres的3d精匹配器
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },

  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,

  -- 优化残差方程的相关参数
  optimization_problem = {
    huber_scale = 1e1,                -- 值越大,（潜在）异常值的影响就越大
    acceleration_weight = 1.1e2,      -- 3d里imu的线加速度的权重
    rotation_weight = 1.6e4,          -- 3d里imu的旋转的权重
    
    -- 前端结果残差的权重
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    -- 里程计残差的权重
    odometry_translation_weight = 1e5,
    odometry_rotation_weight = 1e5,
    -- gps残差的权重
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,

    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },

  max_num_final_iterations = 200,   -- 在建图结束之后执行一次全局优化, 不要求实时性, 迭代次数多
  global_sampling_ratio = 0.003,    -- 纯定位时候查找回环的频率
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10., -- 纯定位时多少秒执行一次全子图的约束计算

  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}

```
# 参数调整经验
## 准备工作
启动机器地盘，通过rosbag采集机器人里程计、激光雷达以及tf信息供后续调试使用。
1、检查rosbag数据，确认数据无误
```
 rosrun cartographer_ros cartographer_rosbag_validate -bag_filename test.bag
```
2、里程计状态信息确认，位置和角度准确度（本地SLAM（前端）和全局SLAM（后端）均需要根据里程计准确度进行调整）
## 前端参数
1、关闭全局SLAM，对本地SLAM（trajectory_builder_2d.lua）进行参数调整。
```
optimize_every_n_nodes = 0
```
2、设置子图大小  
子图必须足够小，以使其内部的漂移低于分辨率，以便它们在本地正确、另一方面，它们应该足够大以使环路闭合能够正常工作。

3、CeresScanMatcher调整  
将CeresScanMatcher中的translation_weight和rotation_weight调到极大，可以看到优化后的轨迹和里程计完全相同，此时主要依靠里程计轨迹建图，CeresScanMatcher作用不大。由于里程计存在误差所以建图效果并不好，因此需要降低里程计在轨迹优化中的权重。里程计精度高则不需要。

## 后端参数
1、开启全局SLAM，对全局SLAM进行参数调整
```
optimize_every_n_nodes = 70 #一般为子图大小两倍
```
2、前端与传感器优化权重配置
```
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight 本地SLAM平移权重
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight 本地SLAM旋转权重
POSE_GRAPH.optimization_problem.odometry_translation_weight 里程计平移权重
POSE_GRAPH.optimization_problem.odometry_rotation_weight 里程计旋转权重
```
以上权重参数越大其对全局SLAM的影响越大，里程计精度越高其权重也可适当调高，反之则降低其权重，使得全局优化更加依赖回环检测的结果。  
3、适当增加min_score和huber_scale的大小，增加对机器人当前运动状态的置信度。  
```
POSE_GRAPH.constraint_builder.min_score = 0.75
POSE_GRAPH.optimization_problem.huber_scale = 1e2
```
4、对于环境相似的场景，减小闭环检测窗口大小减小错误匹配发生的概率
```
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 6.
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
```

## 系统实时性调整
1、降低前端延迟

- 增加 voxel_filter_size（体素滤波的立方体的边长）
- 增加 submaps.resolution（子图分辨率）
- 对于自适应体素滤波, 降低 .min_num_points（如果存在 较多点 并且大于'min_num_points', 则减小体素长度以尝试获得该最小点数）, .max_range（距远离原点超过max_range 的点被移除）, 增加 .max_length（尝试确定最佳的立方体边长）
- 降低 max_range (especially if data is noisy)
- 降低 submaps.num_range_data  

2、降低后端延迟
- 降低 optimize_every_n_nodes
- 增加 MAP_BUILDER.num_background_threads up to the number of cores
- 降低 global_sampling_ratio（纯定位时候查找回环的频率）
- 降低 constraint_builder.sampling_ratio（对局部子图进行回环检测时的计算频率）
- 增加 constraint_builder.min_score（对局部子图进行回环检测时的最低分数阈值）
- 对于自适应体素滤波, 降低 .min_num_points, .max_range, 增加 .max_length
- 增加 voxel_filter_size, submaps.resolution, 降低 submaps.num_range_data
- 降低搜索窗口大小, .linear_xy_search_window, .linear_z_search_window, .angular_search_window
- 增加 global_constraint_search_after_n_seconds（纯定位时多少秒执行一次全子图的约束计算）
- 降低 max_num_iterations（求解器迭代次数）
# 地图保存脚本
```bash
#!/bin/bash
# 刷新环境
source carto_ws/install_isolated/setup.sh 

# 地图保存路径
map_dir="${HOME}/map"      
# 地图名称：当前系统时间
map_name=$(date +%Y%m%d_%H-%M-%S)  

# 检测文件夹是否存在
if [ ! -d "$map_dir" ];then
  echo "文件夹不存在, 即将创建文件夹"
  mkdir -p $map_dir
fi

# 结束建图
rosservice call /finish_trajectory 0

# 生成 pbstream 文件
rosservice call /write_state "{filename: '$map_dir/$map_name.pbstream'}"

# pbstream 转换成 pgm 地图保存
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/$map_name.pbstream \
-map_filestem=$map_dir/$map_name
```