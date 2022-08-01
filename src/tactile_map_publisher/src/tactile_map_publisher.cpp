#include <string>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

#include <ros/ros.h>
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
#include <ackermann_msgs/AckermannDriveStamped.h>
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
  ros::Publisher pub_vis_;
  ros::Time last_time_stamp_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  string global_frame_id_, base_frame_id_;
  int freq_, lethal_cost_, tactile_width_, tactile_height_;
  double map_res_, tactile_res_;
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
  Eigen::Vector2d center_pos;
  center_pos.x() = tf.transform.translation.x + centered_pos.x() * tactile_res_;
  center_pos.y() = tf.transform.translation.y + centered_pos.y() * tactile_res_;
  Eigen::Vector2i center_map = getMapCord(center_pos);

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
      if (center_map.x() + i >= map_.size() || center_map.y() + i >= map_[0].size())
      {
        continue;
        ROS_WARN("out of map");
      }
      auto pos = getMapPos({center_map.x() + i, center_map.y() + j});
      geometry_msgs::Point p;
      p.x = pos.x();
      p.y = pos.y();
      p.z = 0;
      int dv = relu(getMapValue(center_map.x() + i, center_map.y() + j));
      // if (dv < lethal_cost_)
      //{
      //   dv = 0;
      // }
      value += dv;
      tactile_overlay_.points.push_back(p);
    }
  }

  value /= pow(unit_size, 2);

  ROS_INFO("value: %d", value);
  return value;
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
    getTactileMapValue({2, 2});
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
