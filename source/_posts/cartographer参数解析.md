---
title: cartogeapher参数解析
date: 2023-8-14 17:49:03
tags:
  - ROS
  - cartogeapher
categories:
  - cartogeapher
---

# 目录结构

# 参数解析

```
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,--是否使用imu
  min_range = 0.,--雷达距离配置
  max_range = 30.,
  min_z = -0.8,--雷达高度配置，将高度数据转换成2D
  max_z = 2.,
  missing_data_ray_length = 5.,--超出max_range将以此长度进行free插入，充分利用max_range外的数据，也可以不使用。
  num_accumulated_range_data = 1,--将一帧雷达数据分成几个ros发出来，减少运动畸变影响。
  voxel_filter_size = 0.025,--体素滤波，使远处和近处的点云权重一致
  adaptive_voxel_filter = {
    max_length = 0.5,--最大边长0.5
    min_num_points = 200,--大于此数据，则减小体素滤波器的大小。
    max_range = 50.,--大于max_range的值被移除
  },
  loop_closure_adaptive_voxel_filter = {--闭环的体素滤波器，同上
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },
  use_online_correlative_scan_matching = false,--csm算法解决在线扫描匹配问题，为ceres优化提供先验，如果无IMU或odom的情况下，如无此项前端效果较差。但是一旦使用该项，IMU和Odom的效果将会变得很弱。
  real_time_correlative_scan_matcher = {--开启online后使用，分配搜索窗口的参数
    linear_search_window = 0.1, --线窗口
    angular_search_window = math.rad(20.),--角度窗口
    translation_delta_cost_weight = 1e-1,--这两个为平移和旋转的比例，如你知道你的机器人旋转不多，则可以较少它的权重，一般情况下1：1.
    rotation_delta_cost_weight = 1e-1,
  },
--通过online或者imu/odom的先验输入ceres，然后进行优化，以下为优化的参数配置
  ceres_scan_matcher = {
    occupied_space_weight = 1.,--数据源的权重
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {--谷歌开发的最小二乘库ceres Solver配置
      use_nonmonotonic_steps = false,--是否使用非单调的方法
      max_num_iterations = 20,--迭代次数
      num_threads = 1,--使用线程数
    },
  },
--运动过滤器，避免静止的时候插入scans
  motion_filter = {
    max_time_seconds = 5.,--过滤的时间、距离、角度
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },
  imu_gravity_time_constant = 10.,--一定时间内观察imu的重力，以确定是否使用imu数据
-- pose_extrapolator为位姿推测器的参数，好像已经不用了？？？
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
  submaps = {
    num_range_data = 90,--submaps插入的数量
    grid_options_2d = {--子图的形式
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,
    },
    range_data_inserter = {--概率模型插入
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,--插入free空间，没击中
        hit_probability = 0.55,--hit和miss的概率
        miss_probability = 0.49,
      },
      tsdf_range_data_inserter = {--除了2D概率模型，还可以进行TSDF模式插入，没有使用。
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

