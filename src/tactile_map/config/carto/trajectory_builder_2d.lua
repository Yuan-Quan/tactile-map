-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  min_range = 0.,--激光最近距离
  max_range = 0.1,--激光最远距离
  min_z = -0.8,--激光最小高度
  max_z = 2.,--激光最大高度
  missing_data_ray_length = 0.1,--不在min_range和max_range里默认给这个值
  num_accumulated_range_data = 1,--积累几帧激光数据作为一个Node节点
  voxel_filter_size = 0.025,--一帧激光的网络滤波大小，单位米

  adaptive_voxel_filter = {--自适应滤波，点云数量刚好大于最小点云数量，不超过太多
    max_length = 0.5,--采样最大间隔，单位米
    min_num_points = 200,--采样后最小点云数量
    max_range = 50.,--点云最远距离
  },

  loop_closure_adaptive_voxel_filter = {--闭环检测自适应滤波
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  use_online_correlative_scan_matching = false,--使用CSM激光匹配
  real_time_correlative_scan_matcher = {--快速CSN激光匹配
    linear_search_window = 0.1,--平移搜索范围
    angular_search_window = math.rad(20.),--角度搜索范围
    translation_delta_cost_weight = 1e-1,--平移权重，距离初始值越远，匹配得分越高才被信任
    rotation_delta_cost_weight = 1e-1,--旋转权重
  },

  ceres_scan_matcher = {--使用ceres优化方式进行激光匹配
    occupied_space_weight = 1.,--占据空间权重
    translation_weight = 10.,--平移权重
    rotation_weight = 40.,--旋转权重
    ceres_solver_options = {--ceres优化参数
      use_nonmonotonic_steps = false,--使用非单调步骤
      max_num_iterations = 20,--最大迭代次数
      num_threads = 1,--使用几个线程优化
    },
  },

  motion_filter = {--移动滤波，判断小车有没有动。就是检测2帧激光数据是否相似，要满足全部条件
    max_time_seconds = 2.,--2帧激光时间戳最小间隔
    max_distance_meters = 0.08,--2帧激光最小距离
    max_angle_radians = math.rad(1.),--2帧激光最小角度
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,--imu重力常数
  pose_extrapolator = {--位姿推断
    use_imu_based = false,--3d用来初始化位姿推断器的方式
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
    imu_based = {--这个3D中用到
      pose_queue_duration = 5.,
      gravity_constant = 9.806,--重力常数
      pose_translation_weight = 1.,--位姿偏移权重，偏移越远要求得分越高才被信任
      pose_rotation_weight = 1.,--位姿旋转权重
      imu_acceleration_weight = 1.,--IMU加速度权重
      imu_rotation_weight = 1.,--IMU旋转权重
      odometry_translation_weight = 1.,--里程计平移权重
      odometry_rotation_weight = 1.,--里程计旋转权重
      solver_options = {--优化参数
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  submaps = {--子图
    num_range_data = 90,--子图中Node的数量，过大产生位姿漂移，过小不利于后台回环检测
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",--概率栅格地图
      resolution = 0.05,--分辨率
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
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
