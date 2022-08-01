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
  string global_frame_id_;
  int freq_, lethal_cost_;
  double map_res_;
  Eigen::Vector2d map_origin_;

  dynamic_reconfigure::Server<tactile_map_publisher::TactileMapPublisherConfig> reconfigure_server_;
  dynamic_reconfigure::Server<tactile_map_publisher::TactileMapPublisherConfig>::CallbackType reconfigure_callback_;

  visualization_msgs::MarkerArray markers_;
  std::vector<std::vector<int>> map_;

  void recivecCostmap(nav_msgs::OccupancyGrid map);
  void recivecCostmapUpdate(map_msgs::OccupancyGridUpdate update);
  void publishMarker();
  int getMapValue(Eigen::Vector2d pos);
  int getMapValue(int i, int j);
  Eigen::Vector2i getMapCord(Eigen::Vector2d pos);
  Eigen::Vector2d getMapPos(Eigen::Vector2i pos);
};

TactileMapPublisher::TactileMapPublisher() : global_frame_id_("map"), nh_private_("~"), tf_listener_(tf_buffer_)
{
  // Get parameters from the parameter server
  nh_private_.param<string>("global_frame_id", global_frame_id_, "map");
  nh_private_.param<int>("rate", freq_, 12);
  nh_private_.param<int>("lethal_cost", lethal_cost_, 99);

  pub_vis_ = nh_.advertise<visualization_msgs::MarkerArray>("tactile_map_visualization", 0);
  sub_costmap_ = nh_.subscribe("costmap", 1, &TactileMapPublisher::recivecCostmap, this);
  sub_costmap_update_ = nh_.subscribe("costmap_update", 1, &TactileMapPublisher::recivecCostmapUpdate, this);

  markers_.markers.resize(1);

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

  pub_vis_.publish(markers_);
}

void TactileMapPublisher::run()
{
  while (ros::ok())
  {
    ros::spinOnce();
    publishMarker();
    ros::spinOnce();
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
