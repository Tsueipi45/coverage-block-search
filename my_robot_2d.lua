-- Cartographer 2D SLAM configuration for robot
-- Sensors: RPLIDAR (laser) + IMU + Odometry (wheel odometry via EKF)
-- Three-sensor fusion: laser for scan matching, IMU for gravity/orientation,
-- odometry for translation prediction.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,          -- 轮式里程计
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,          -- 单线激光雷达
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 1e-3,   -- 1000Hz: 减少odom→map变换延迟
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,     -- 激光: 100% 采样
  odometry_sampling_ratio = 1.,        -- 里程计: 100% 采样
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,             -- IMU: 100% 采样
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ===== 激光雷达（RPLIDAR）=====
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0

-- ===== IMU =====
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- ===== 在线相关性扫描匹配 =====
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- ===== Submap =====
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- ===== Ceres 精匹配 =====
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- ===== 运动滤波器：避免静止时重复插入数据 =====
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1

-- ===== 位姿图（回环检测）=====
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
