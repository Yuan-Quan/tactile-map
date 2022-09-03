# designer docs

## 启动序列

发布固定的坐标系变换关系

```xml
File: src/tactile_map/launch/static_tf.launch


<launch>
  <!-- This file contains all the static tf publisher -->

  <!-- Devices, Sensors -->
  <!-- those are measured physical distance between sensors and base center -->
  <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
    args="0.0 0.0 0.0 0.0 0.0 0.0 /base_link /laser_link 20" />
  <node pkg="tf" type="static_transform_publisher" name="baselink_to_imu"
    args="0.0 0.0 0.0 0.0 0.0 0.0 /base_link /imu_link 20" />

</launch>

```

启动IMU的上位机驱动程序

```xml
File: 

<launch>
  <node pkg="fdilink_ahrs" name="ahrs_driver" type="ahrs_driver" output="screen" >
    <!-- 是否输出debug信息 -->
    <param name="debug"  value="false"/>
    
    <!-- 串口设备，可通过rules.d配置固定。
     若使用DETA100,则value="/dev/wheeltec_ch340"
     若使用WHEELTEC N系列，则不需要改动 -->

    <param name="port"  value="/dev/ttyACM0"/>

    <!-- 波特率 -->
    <param name="baud"  value="921600"/>

    <!-- 发布的imu话题名 -->
    <param name="imu_topic"  value="imu"/>

    <!-- 发布的imu话题中的frame_id -->
    <param name="imu_frame"  value="imu_link"/>

    <!-- 地磁北的yaw角 -->
    <param name="mag_pose_2d_topic"  value="/mag_pose_2d"/>
    <!-- 欧拉角 -->
    <param name="Euler_angles_pub_"  value="/euler_angles"/>
    <!-- 磁力计磁场强度 -->
    <param name="Magnetic_pub_"  value="/magnetic"/>

    <!-- 发布的数据基于不同设备有不同的坐标系   -->
    <param name="device_type"  value="1"/> <!-- 0: origin_data, 1: for single imu or ucar in ROS, 2:for Xiao in ROS -->
  </node>

</launch> 

```

启动激光雷达上位机驱动

```xml
File: src/ydlidar_ros_driver-master/launch/X3.launch

<launch>
  <node name="ydlidar_lidar_publisher"  pkg="ydlidar_ros_driver"  type="ydlidar_ros_driver_node" output="screen" respawn="false" >
    <!-- string property -->
    <param name="port"         type="string" value="/dev/ttyUSB0"/>  
    <param name="frame_id"     type="string" value="laser_link"/>
    <param name="ignore_array"     type="string" value=""/>

    <!-- int property -->
    <param name="baudrate"         type="int" value="115200"/>  
    <!-- 0:TYPE_TOF, 1:TYPE_TRIANGLE, 2:TYPE_TOF_NET -->
    <param name="lidar_type"       type="int" value="1"/>  
    <!-- 0:YDLIDAR_TYPE_SERIAL, 1:YDLIDAR_TYPE_TCP -->
    <param name="device_type"         type="int" value="0"/>  
    <param name="sample_rate"         type="int" value="3"/>  
    <param name="abnormal_check_count"         type="int" value="4"/>  

    <!-- bool property -->
    <param name="resolution_fixed"    type="bool"   value="true"/>
    <param name="auto_reconnect"    type="bool"   value="true"/>
    <param name="reversion"    type="bool"   value="false"/>
    <param name="inverted"    type="bool"   value="true"/>
    <param name="isSingleChannel"    type="bool"   value="true"/>
    <param name="intensity"    type="bool"   value="false"/>
    <param name="support_motor_dtr"    type="bool"   value="true"/>
    <param name="invalid_range_is_inf"    type="bool"   value="false"/>
    <param name="point_cloud_preservative"    type="bool"   value="false"/>

    <!-- float property -->
    <param name="angle_min"    type="double" value="-180" />
    <param name="angle_max"    type="double" value="180" />
    <param name="range_min"    type="double" value="0.1" />
    <param name="range_max"    type="double" value="12.0" />
    <!-- frequency is invalid, External PWM control speed -->
    <param name="frequency"    type="double" value="10.0"/>
  </node>
</launch>
```

启动cartographer节点

```xml
File: src/tactile_map/launch/carto_mapping.launch

 <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find tactile_map)/config/carto
          -configuration_basename tk_map.lua"
      output="screen">
    <remap from="scan" to="scan" />
  </node>
```

将carto的submap数据格式转化为占位格栅地图发布

```xml
File: src/tactile_map/launch/carto_mapping.launch

  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.04" />
```

启动可视化

```xml
File: src/tactile_map/launch/carto_mapping.launch

  <node name="rviz" pkg="rviz" type="rviz" 
	  args="-d $(find tactile_map)/config/carto/cartographer.rviz" />
```

采样用户周围的地图, 降采样到针矩阵的分辨率, 以占位格栅形式发布

```xml
File: src/tactile_map_publisher/launch/tactile_map_publisher.launch

<launch>
  <node pkg="tactile_map_publisher" type="tactile_map_publisher" name="tactile_map_publisher" output="screen">
    <rosparam file="$(find tactile_map_publisher)/config/tactile_map_publisher.yaml" command="load" />
    <remap from="costmap" to="map" />
    <remap from="costmap_update" to="map_updates" />
  </node>

</launch>
```

接受上一个节点发出的格栅, 调用GPIO控制针矩阵动作

```xml
File: src/tactile_controller/launch/tactile_controller.launch

<launch>
  <node pkg="tactile_controller" type="tactile_controller.py" name="tactile_controller" output="screen"/>
</launch>

```

## 节点代码

### tactile_map_publisher

接收cartographer_occupancy_grid_node发出的占位格栅地图  
订阅cartographer的定位变换  
当有新的地图时, 更新地图  
根据预设的分辨率和尺寸, 把地图降采样成对应针矩阵的格栅地图  
根据预设的致命带价值, 计算新地图的代价
发送给针矩阵驱动节点

```cpp
File: src/tactile_map_publisher/src/tactile_map_publisher.cpp

#include <string>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <visualization_msgs/MarkerArray.h>

#include <kdl/frames.hpp>

// TODO: Figure out how to use tf2 DataConversions
// for more elegant and compact code
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <tactile_map_publisher/TactileMapPublisherConfig.h>
#include <Eigen/Dense>

using std::string;

class TactileMapPublisher
{
public:
  //! Constructor
  TactileMapPublisher();

  //! Helper founction for computing eucledian distances in the x-y plane.
  template <typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
  }

  double distance(int dx, int dy, double res)
  {
    return sqrt(pow(dx * res, 2) + pow(dy * res, 2));
  }

  inline int relu(int x)
  {
    return x > 0 ? x : 0;
  }

  //! Dynamic reconfigure callback.
  void reconfigure(tactile_map_publisher::TactileMapPublisherConfig &config, uint32_t level);

  //! Run the controller.
  void run();

private:
  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_costmap_, sub_costmap_update_;
  ros::Publisher pub_vis_, pub_tk_map_;
  ros::Time last_time_stamp_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  string global_frame_id_, base_frame_id_;
  int freq_, lethal_cost_, tactile_width_, tactile_height_;
  double map_res_, tactile_res_;
  bool is_ignore_non_lethal_grid_;
  Eigen::Vector2d map_origin_;

  dynamic_reconfigure::Server<tactile_map_publisher::TactileMapPublisherConfig> reconfigure_server_;
  dynamic_reconfigure::Server<tactile_map_publisher::TactileMapPublisherConfig>::CallbackType reconfigure_callback_;

  visualization_msgs::MarkerArray markers_;
  visualization_msgs::Marker tactile_overlay_;
  std::vector<std::vector<int>> map_;
  std::vector<std::vector<int>> tactile_map_;

  void recivecCostmap(nav_msgs::OccupancyGrid map);
  void recivecCostmapUpdate(map_msgs::OccupancyGridUpdate update);
  void publishMarker();
  int getMapValue(Eigen::Vector2d pos);
  int getMapValue(int i, int j);
  int getTactileMapValue(Eigen::Vector2i pos);
  void updateTactileMap();
  Eigen::Vector2i getMapCord(Eigen::Vector2d pos);
  Eigen::Vector2d getMapPos(Eigen::Vector2i pos);
};

TactileMapPublisher::TactileMapPublisher() : global_frame_id_("map"), nh_private_("~"), tf_listener_(tf_buffer_)
{
  // Get parameters from the parameter server
  nh_private_.param<string>("fixed_frame_id", global_frame_id_, "map");
  nh_private_.param<string>("base_frame_id", base_frame_id_, "base_link");
  nh_private_.param<int>("rate", freq_, 12);
  nh_private_.param<int>("lethal_cost", lethal_cost_, 99);
  nh_private_.param<double>("tactile_resolution", tactile_res_, 0.5);
  nh_private_.param<int>("tactile_width", tactile_width_, 4);
  nh_private_.param<int>("tactile_height", tactile_height_, 4);
  nh_private_.param<bool>("is_ignore_non_lethal_grid", is_ignore_non_lethal_grid_, false);

  // initialize the tactile_map_
  tactile_map_.resize(tactile_height_);
  for (size_t i = 0; i < tactile_height_; i++)
  {
    tactile_map_[i].resize(tactile_width_);
    for (size_t j = 0; j < tactile_width_; j++)
    {
      tactile_map_[i][j] = 0;
    }
  }

  pub_vis_ = nh_.advertise<visualization_msgs::MarkerArray>("tactile_map_visualization", 0);
  pub_tk_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("tk_map", 0);
  sub_costmap_ = nh_.subscribe("costmap", 1, &TactileMapPublisher::recivecCostmap, this);
  sub_costmap_update_ = nh_.subscribe("costmap_update", 1, &TactileMapPublisher::recivecCostmapUpdate, this);

  markers_.markers.resize(2);

  tactile_overlay_.header.frame_id = global_frame_id_;
  tactile_overlay_.header.stamp = ros::Time::now();
  tactile_overlay_.ns = "tactile_map_publisher";
  tactile_overlay_.action = visualization_msgs::Marker::ADD;
  tactile_overlay_.pose.orientation.w = 1.0;
  tactile_overlay_.id = 1;
  tactile_overlay_.type = visualization_msgs::Marker::POINTS;
  // tactile_overlay_.scale.x = tactile_res_;
  // tactile_overlay_.scale.y = tactile_res_;
  tactile_overlay_.scale.x = 0.04;
  tactile_overlay_.scale.y = 0.04;
  tactile_overlay_.color.r = 1.0f;
  tactile_overlay_.color.a = 0.3;
  tactile_overlay_.points.clear();

  reconfigure_callback_ = boost::bind(&TactileMapPublisher::reconfigure, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}

void TactileMapPublisher::recivecCostmap(nav_msgs::OccupancyGrid map)
{
  map_res_ = map.info.resolution;
  map_origin_ << map.info.origin.position.x, map.info.origin.position.y;

  size_t width = map.info.width;
  size_t height = map.info.height;
  size_t idx = 0;

  map_.clear();

  typedef std::vector<int> Row;

  for (size_t i = 0; i < height; i++)
  {
    Row row(width);

    for (size_t j = 0; j < width; j++)
    {
      row[j] = map.data[idx];
      idx++;
    }

    map_.push_back(row); // push each row after you fill it
  }
}

void TactileMapPublisher::recivecCostmapUpdate(map_msgs::OccupancyGridUpdate update)
{
  size_t idx = 0;
  for (size_t i = 0; i < update.height; i++)
  {
    for (size_t j = 0; j < update.width; j++)
    {
      map_[update.y + i][update.x + j] = update.data[idx];
      idx++;
    }
  }
}

int TactileMapPublisher::getMapValue(Eigen::Vector2d pos)
{
  auto cord = getMapCord(pos);
  if (cord.x() >= map_.size() || cord.y() >= map_[0].size())
  {
    return -1;
  }

  return map_[cord.x()][cord.y()];
}

int TactileMapPublisher::getMapValue(int i, int j)
{
  if (i >= map_.size() || j >= map_[0].size())
  {
    return -1;
  }

  return map_[i][j];
}
Eigen::Vector2i TactileMapPublisher::getMapCord(Eigen::Vector2d pos)
{
  int i = 0;
  int j = 0;
  j = (pos.x() - map_origin_.x()) / map_res_;
  i = (pos.y() - map_origin_.y()) / map_res_;

  return Eigen::Vector2i(i, j);
}

Eigen::Vector2d TactileMapPublisher::getMapPos(Eigen::Vector2i pos)
{
  double x = pos.y() * map_res_ + map_origin_.x();
  double y = pos.x() * map_res_ + map_origin_.y();

  return Eigen::Vector2d(x, y);
}

int TactileMapPublisher::getTactileMapValue(Eigen::Vector2i pos)
{
  // look up tf to determine the position of base in the map frame
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_buffer_.lookupTransform(global_frame_id_, base_frame_id_, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN(" tf error:\n");
    ROS_WARN_STREAM(ex.what());
  }
  Eigen::Vector2i centered_pos;
  centered_pos.x() = pos.x() - tactile_height_ / 2;
  centered_pos.y() = pos.y() - tactile_width_ / 2;

  tf::Quaternion q = tf::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                                    tf.transform.rotation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  Eigen::Rotation2D<double> rot2(yaw);

  Eigen::Vector2d center_pos;
  center_pos.x() = tf.transform.translation.x; // + rot2 * (centered_pos.x() * tactile_res_) ;
  center_pos.y() = tf.transform.translation.y; // + rot2 * (centered_pos.y() * tactile_res_);

  center_pos = center_pos + rot2 * Eigen::Vector2d(centered_pos.x() * tactile_res_, centered_pos.y() * tactile_res_);

  int unit_size;
  if (map_res_ < 10e-6)
  {
    unit_size = 0;
  }
  else
  {
    unit_size = tactile_res_ / map_res_;
  }

  tactile_overlay_.points.clear();
  int value = 0;
  for (size_t i = 0; i < unit_size; i++)
  {
    for (size_t j = 0; j < unit_size; j++)
    {
      Eigen::Vector2d point = center_pos + rot2 * (Eigen::Vector2d(j, i) * map_res_);
      // auto pos = getMapPos({center_map.x() + i, center_map.y() + j});
      auto cord = getMapCord(point);
      geometry_msgs::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = 0;
      int dv = relu(getMapValue(cord.x(), cord.y()));
      if (is_ignore_non_lethal_grid_)
      {
        if (dv < lethal_cost_)
        {
          dv = 0;
        }
      }

      value += dv;
      tactile_overlay_.points.push_back(p);
    }
  }

  value /= pow(unit_size, 2);

  //ROS_INFO("value: %d", value);
  return value;
}

void TactileMapPublisher::updateTactileMap()
{
  nav_msgs::OccupancyGrid tk_map;
  tk_map.header.frame_id = base_frame_id_;
  tk_map.info.resolution = tactile_res_;
  tk_map.info.width = tactile_width_;
  tk_map.info.height = tactile_height_;
  tk_map.info.origin.position.x = -1;
  tk_map.info.origin.position.y = -1;
  // tk_map.info.origin.position.x = tactile_height_ / 2 * tactile_res_;
  // tk_map.info.origin.position.y = -tactile_width_ / 2 * tactile_res_;

  // tk_map.data.resize(tk_map.info.width * tk_map.info.height);
  for (size_t i = 0; i < tactile_height_; i++)
  {
    for (size_t j = 0; j < tactile_width_; j++)
    {
      tk_map.data.push_back(getTactileMapValue({j, i}));
    }
  }

  pub_tk_map_.publish(tk_map);
}

void TactileMapPublisher::publishMarker()
{
  visualization_msgs::Marker lethal_obstacles;
  lethal_obstacles.header.frame_id = global_frame_id_;
  lethal_obstacles.header.stamp = ros::Time::now();
  lethal_obstacles.ns = "tactile_map_publisher";
  lethal_obstacles.action = visualization_msgs::Marker::ADD;
  lethal_obstacles.pose.orientation.w = 1.0;
  lethal_obstacles.id = 0;
  lethal_obstacles.type = visualization_msgs::Marker::POINTS;
  lethal_obstacles.scale.x = 0.04;
  lethal_obstacles.scale.y = 0.04;
  lethal_obstacles.color.g = 1.0f;
  lethal_obstacles.color.a = 1.0;
  lethal_obstacles.points.clear();

  // Create the vertices for the points
  for (size_t i = 0; i < map_.size(); i++)
  {
    for (size_t j = 0; j < map_[i].size(); j++)
    {
      if (map_[i][j] < lethal_cost_)
      {
        continue;
      }

      auto pos = getMapPos({i, j});
      geometry_msgs::Point p;
      p.x = pos.x();
      p.y = pos.y();
      p.z = 0;

      lethal_obstacles.points.push_back(p);
    }
  }

  markers_.markers[0] = lethal_obstacles;
  markers_.markers[1] = tactile_overlay_;

  pub_vis_.publish(markers_);
}

void TactileMapPublisher::run()
{
  while (ros::ok())
  {
    ros::spinOnce();
    publishMarker();
    ros::spinOnce();
    // getTactileMapValue({0, 0});
    ros::spinOnce();
    updateTactileMap();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / freq_));
  }
}

// this is the callback of dynamic reconfigure
void TactileMapPublisher::reconfigure(tactile_map_publisher::TactileMapPublisherConfig &config, uint32_t level)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tactile_map_publisher");

  TactileMapPublisher node;
  node.run();

  return 0;
}

```

### tactile_controller

触摸反馈针矩阵的驱动节点  
当格栅的代价超过阈值时, 驱动该针的电磁铁

```python
File: src/tactile_controller/tactile_controller.py

#!/usr/bin/env python

from __future__ import print_function
from select import select
import sys
import time
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import rospy
import RPi.GPIO as GPIO
# from msilib.schema import Control

import threading

import roslib
roslib.load_manifest('tactile_controller')


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


MUX_TABLE = {
#   CH EN S0 S1 S2 S3
    0: (0, 0, 0, 0),
    1: (1, 0, 0, 0),
    2: (0, 1, 0, 0),
    3: (1, 1, 0, 0),
    4: (0, 0, 1, 0),
    5: (1, 0, 1, 0),
    6: (0, 1, 1, 0),
    7: (1, 1, 1, 0),
    8: (0, 0, 0, 1),
    9: (1, 0, 0, 1),
    10: (0, 1, 0, 1),
    11: (1, 1, 0, 1),
    12: (0, 0, 1, 1),
    13: (1, 0, 1, 1),
    14: (0, 1, 1, 1),
    15: (1, 1, 1, 1),
}

RL0_GPIO = 21
RL1_GPIO = 20
RL2_GPIO = 16
RL3_GPIO = 26
RL4_GPIO = 19
RL5_GPIO = 13
RL6_GPIO = 6
RL7_GPIO = 12
RL8_GPIO = 5
RL9_GPIO = 0
RL10_GPIO = 1
RL11_GPIO = 7
RL12_GPIO = 8
RL13_GPIO = 11
RL14_GPIO = 9
RL15_GPIO = 25

RL_PINS = [RL0_GPIO, RL1_GPIO, RL2_GPIO, RL3_GPIO, RL4_GPIO, RL5_GPIO, RL6_GPIO, RL7_GPIO, RL8_GPIO, RL9_GPIO, RL10_GPIO, RL11_GPIO, RL12_GPIO, RL13_GPIO, RL14_GPIO, RL15_GPIO]

class Kontroller(object):
    def __init__(self):
        rospy.init_node('tactile_controller')
        self.rate = rospy.get_param("~rate", 12)
        self.thres = rospy.get_param("~threshould", 10)
        self.grid = OccupancyGrid()
        self.sub_grid = rospy.Subscriber("tk_map", OccupancyGrid, self.recive_map)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RL_PINS, GPIO.IN)
        print(RL_PINS)
        print("GPIO setup complete")
        #self.write()

    def recive_map(self, grid):
        self.grid = grid
        self.write()

    def write(self):
        i = 0
        for i in range(4):
            for j in range(4):
                idx = i*4 + j
                item = self.grid.data[idx]
                if item >= self.thres:
                    md = GPIO.OUT
                else:
                    md = GPIO.IN
                self.set_pin(j, i, md)

    def set_pin(self, i, j, state):
        idx = i*4 + j
        if idx >= 16 or idx <0:
            print("out of range pin idx")
            return
        GPIO.setup(RL_PINS[idx], state)

    def stop(self):
        print("\nShutdown gracefully")
        GPIO.setup(RL_PINS, GPIO.IN)
        GPIO.cleanup()
        print("GPIO uninitialize")
        
    def run(self):
        pass

if __name__ == "__main__":


    controller = Kontroller()

    rospy.on_shutdown(controller.stop)

    rospy.spin()


```

### map_builder

负责CSM梯度优化解出sub_map位置

```cpp
File: cartographer/mapping/map_builder.h

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class MapBuilder : public MapBuilderInterface {
 public:
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;

  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();
  }

  int num_trajectory_builders() const override {
    return trajectory_builders_.size();
  }

  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();
  }

  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

 private:
  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<PoseGraph> pose_graph_;

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
};

std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
```

```cpp
File: cartographer/mapping/map_builder.cc

#include "cartographer/mapping/map_builder.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace {

using mapping::proto::SerializedData;

std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

void MaybeAddPureLocalizationTrimmer(
    const int trajectory_id,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    PoseGraph* pose_graph) {
  if (trajectory_options.pure_localization()) {
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  if (trajectory_options.has_pure_localization_trimmer()) {
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id,
        trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
  }
}

}  // namespace

MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  CHECK(options.use_trajectory_builder_2d() ^
        options.use_trajectory_builder_3d());
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.use_trajectory_builder_3d()) {
    pose_graph_ = absl::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.collate_by_trajectory()) {
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}

int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  const int trajectory_id = trajectory_builders_.size();

  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;
  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }

  if (options_.use_trajectory_builder_3d()) {
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder3D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph3D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  } else {
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_2d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(
          trajectory_options.trajectory_builder_2d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder2D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  }
  MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
                                  pose_graph_.get());

  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    pose_graph_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

int MapBuilder::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_->FinishTrajectory(trajectory_id);
  pose_graph_->FinishTrajectory(trajectory_id);
}

std::string MapBuilder::SubmapToProto(
    const SubmapId& submap_id, proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

void MapBuilder::SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriterInterface* const writer) {
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, writer,
                    include_unfinished_submaps);
}

bool MapBuilder::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  io::ProtoStreamWriter writer(filename);
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, &writer,
                    include_unfinished_submaps);
  return (writer.Close());
}

std::map<int, int> MapBuilder::LoadState(
    io::ProtoStreamReaderInterface* const reader, bool load_frozen_state) {
  io::ProtoStreamDeserializer deserializer(reader);

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options();

  std::map<int, int> trajectory_remapping;
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(i);
    const int new_trajectory_id =
        AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      pose_graph_->FreezeTrajectory(new_trajectory_id);
    }
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {
    pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()),
                                 true);
  }

  if (options_.use_trajectory_builder_3d()) {
    CHECK_NE(deserializer.header().format_version(),
             io::kFormatVersionWithoutSubmapHistograms)
        << "The pbstream file contains submaps without rotational histograms. "
           "This can be converted with the 'pbstream migrate' tool, see the "
           "Cartographer documentation for details. ";
  }

  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id()));
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        pose_graph_->AddSubmapFromProto(submap_poses.at(submap_id),
                                        proto.submap());
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        pose_graph_->AddNodeFromProto(node_pose, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData: {
        if (load_frozen_state) break;
        pose_graph_->AddImuData(
            trajectory_remapping.at(proto.imu_data().trajectory_id()),
            sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData: {
        if (load_frozen_state) break;
        pose_graph_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData: {
        if (load_frozen_state) break;
        pose_graph_->AddFixedFramePoseData(
            trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData: {
        if (load_frozen_state) break;
        pose_graph_->AddLandmarkData(
            trajectory_remapping.at(proto.landmark_data().trajectory_id()),
            sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  if (load_frozen_state) {
    // Add information about which nodes belong to which submap.
    // This is required, even without constraints.
    for (const proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      if (constraint_proto.tag() !=
          proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      pose_graph_->AddNodeToSubmap(
          NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          SubmapId{constraint_proto.submap_id().trajectory_id(),
                   constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(
        FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
  return trajectory_remapping;
}

std::map<int, int> MapBuilder::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  io::ProtoStreamReader stream(state_filename);
  return LoadState(&stream, load_frozen_state);
}

std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options) {
  return absl::make_unique<MapBuilder>(options);
}

}  // namespace mapping
}  // namespace cartographer
```

### pose_extraplator

积分IMU等高频率传感器采集的速度信息  
外推当前位置与姿态

```cpp
 File: cartographer/cartographer/mapping/pose_extrapolator.h

 #ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
class PoseExtrapolator : public PoseExtrapolatorInterface {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;

  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  void AddImuData(const sensor::ImuData& imu_data) override;
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) override;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  const common::Duration pose_queue_duration_;
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
```

```cpp
File: cartographer/cartographer/mapping/pose_extrapolator.cc

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  AdvanceImuTracker(time, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }

  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                               ? linear_velocity_from_poses_
                                               : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
```
