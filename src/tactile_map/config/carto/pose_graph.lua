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

POSE_GRAPH = {
  optimize_every_n_nodes = 90,--指定数量节点插入地图中，执行一次优化。0相当于没有全局
  constraint_builder = {--约束构造器
    sampling_ratio = 0.9,--约束因子，太大降低全局SLAM效率并无法实时回环检测，太小缺失约束并无效回环检测
    max_constraint_distance = 15.,--当前node节点与当前submap之间的距离小于  才建立全局约束
    min_score = 0.55,--使用fast_CSM检测当前node与当前submap的匹配程度，超过0.55才建立全局约束
    global_localization_min_score = 0.6,--使用fast_CSM检测当前node与当前submap的匹配程度，超过0.6才建立全局约束
    loop_closure_translation_weight = 1.1e4,--闭环检测平移权重
    loop_closure_rotation_weight = 1e5,--闭环检测旋转权重
    log_matches = true,--是否打印匹配结果

    fast_correlative_scan_matcher = {--激光匹配，用来实时回环检测
      linear_search_window = 0.4,--平移搜索范围
      angular_search_window = math.rad(30.),--角度搜索范围
      branch_and_bound_depth = 15,--分枝定界深度，金字塔层数
    },

    ceres_scan_matcher = {
      occupied_space_weight = 20.,--匹配的占据空间权重
      translation_weight = 10.,--匹配平移权重
      rotation_weight = 1.,--匹配旋转权重
      ceres_solver_options = {--ceres优化参数
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },

    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },

    ceres_scan_matcher_3d = {--参照前端ceres_scan_matcher
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
  matcher_translation_weight = 5e2,--当前submap与当前submap内的某个节点之间的平移约束
  matcher_rotation_weight = 1.6e3,--当前submap与当前submap内的某个节点之间的旋转约束
  optimization_problem = {
    huber_scale = 1e1,--Huber调节因子，他越大，错误的数据对整体的影响越大
    acceleration_weight = 1.1e2,--for 3d 加速权重
    rotation_weight = 1.6e4,--for 3d 旋转权重
    local_slam_pose_translation_weight = 1e5,--前后2个node之间局部观测与全局优化之间的平移约束权重
    local_slam_pose_rotation_weight = 1e5,--前后2个node之间的局部观测与全局优化之间旋转约束权重
    odometry_translation_weight = 1e5,--前后2个node之阿的局部观测与里程计观测之间的平移约束权重
    odometry_rotation_weight = 1e5,--前后2个node之阿的局部观测与里程计观测之间的旋转约束权重
    fixed_frame_pose_translation_weight = 1e1,--fixed 与gps相关
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,
    log_solver_summary = false,--IMU优化结果不理想，可以设置。
    use_online_imu_extrinsics_in_3d = true,--IMU优化结果不理想，可以设置。是否在线标定IMU外参
    fix_z_in_3d = false,
    ceres_solver_options = {--ceres优化参数
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,--最大迭代次数，越大越好
  global_sampling_ratio = 0.003,--全局地图匹配约束nodes采样比率，间隔1/0.003个node进行一次全局约束检测
  log_residual_histograms = true,--是否输出残差直方图
  global_constraint_search_after_n_seconds = 10.,--间隔多久全局匹配一次，这个要和global_sampling_ratio同时满足要求，才进行全局地图匹配寻找回环，这个参数在多机器人同时建图中使用。
  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}
