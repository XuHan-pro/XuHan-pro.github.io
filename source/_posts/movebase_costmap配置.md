---
title: movebase_costmap配置
date: 2023-8-11 17:49:03
tags:
  - ROS
  - movebase
  - costmap
categories:
  - movebase
---
- [costmap\_common\_params](#costmap_common_params)
- [global\_costmap\_params](#global_costmap_params)
- [local\_costmap\_params](#local_costmap_params)

# costmap_common_params


```bash
max_obstacle_height: 2.0  #考虑最大障碍物高度范围

# Obstacle Cost Shaping (http://wiki.ros.org/costmap_2d/hydro/inflation)
footprint: [[0.35,-0.25],[0.35,0.25],[-0.35,0.25],[-0.35,-0.25]] #机器轮廓

static_layer:
  enabled:              true    #是否启用该层地图
  map_topic:            "/map"
  first_map_only:       false   #是否根据map_server提供的地图初始化
#障碍物层
obstacle_layer:
  enabled:              true
  max_obstacle_height:  2.0
  min_obstacle_height:  0.0
  combination_method:   1
  track_unknown_space:  true    #设置为true如果允许全局路径通过未知区域
  obstacle_range: 2.5     #机器人检测障碍物的最大范围  
  raytrace_range: 3       #在机器人移动过程中，实时清除代价地图上的障碍物的最大范围，更新可自由移动的空间数据。假如设置该值为3米，那么就意味着在3米内的障碍物，本来开始时是有的，但是本次检测却没有了，那么就需要在代价地图上来更新，将旧障碍物的空间标记为可以自由移动的空间。
  publish_voxel_map: false
  observation_sources:  scan
  scan:
    data_type: LaserScan
    topic: "/scan"
    marking: true
    clearing: true
    inf_is_valid: true
    expected_update_rate: 10 #20
#深度相机层
depth_layer:
  enabled:              true
  max_obstacle_height:  2.0
  min_obstacle_height:  0.0
  combination_method:   1
  track_unknown_space:  true    
  obstacle_range: 2.5 #2.0
  raytrace_range: 5 #5.0
  publish_voxel_map: false
  observation_sources:  scan 
  scan:
    data_type: LaserScan
    topic: "/wr_scan3"
    marking: true
    clearing: true
    inf_is_valid: true
    expected_update_rate: 5 #20

#超声波层
sonar_layer:
  enabled:            true
  clear_threshold:    0.8
  mark_threshold:     0.98
  topics: ["/ultrasonic_front_left", "/ultrasonic_front_right"]   
  clear_on_max_reading: true
#虚拟地图层(虚拟墙，禁止线，跌落风险区) 
virtual_layer:
  enabled:              true
  Forbidden_line:       [/virtual_costamp_layer/Forbidden_line]
  Drop_risk_areas:      [/virtual_costamp_layer/Drop_risk_areas]
  one_zone:             true


#膨胀层
inflation_layer:
  enabled:              true
  cost_scaling_factor:  10 #5 #10.0  # 代价变化率(default: 10)
  inflation_radius:     0.05 #0.50  # 膨胀半径
```

# global_costmap_params

```bash
global_costmap:
   global_frame: map
   robot_base_frame: base_link
   update_frequency: 1 #更新频率
   publish_frequency: 1 #可视化发布频率
   static_map: true #是否为静态地图
   rolling_window: false
   inflation_radius: 0.8 #膨胀半径
   cost_scaling_factor: 2.0
   trace_unknow_space: true
   transform_tolerance: 1 #tf超时阈值
   plugins:
    - {name: static_layer,      type: "costmap_2d::StaticLayer"}
    - {name: virtual_layer,      type: "virtual_costmap_layer::VirtualLayer" }
    - {name: inflation_layer,     type: "costmap_2d::InflationLayer"}

```
# local_costmap_params

```bash
local_costmap:
   global_frame: map #全局坐标系
   robot_base_frame: base_link #机器坐标系
   update_frequency:  10 #更新频率
   publish_frequency: 1 #可视化发布频率
   static_map: false
   rolling_window: true
   width:  5
   height: 5
   resolution: 0.05
   inflation_radius:     0.4
   transform_tolerance: 1 
   plugins:
    - {name: static_layer,      type: "costmap_2d::StaticLayer"}
    - {name: sonar_layer,         type: "range_sensor_layer::RangeSensorLayer"}
    - {name: obstacle_layer,      type: "costmap_2d::ObstacleLayer"}
    - {name: depth_layer,     type: "costmap_2d::ObstacleLayer"}
    - {name: virtual_layer,      type: "virtual_costmap_layer::VirtualLayer" }
    - {name: inflation_layer,     type: "costmap_2d::InflationLayer"}
```