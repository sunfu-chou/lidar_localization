/**
 *
 * @file area_obstacles_extractor.cpp
 * @brief
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author sunfu.chou (sunfu.chou@gmail.com)
 * @version 0.1
 * @date 2021-07-02
 *
 */

#include "lidar_localization/area_obstacles_extractor.h"

using namespace std;
using namespace lidar_localization;

AreaObstaclesExtractor::AreaObstaclesExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
  : nh_(nh), nh_local_(nh_local)
{
  params_srv_ = nh_local_.advertiseService("params", &AreaObstaclesExtractor::updateParams, this);
  initialize();
}

AreaObstaclesExtractor::~AreaObstaclesExtractor()
{
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("p_x_min_range_");
  nh_local_.deleteParam("p_x_max_range_");
  nh_local_.deleteParam("p_y_min_range_");
  nh_local_.deleteParam("p_y_max_range_");
  nh_local_.deleteParam("obstacle_radius");
}

bool AreaObstaclesExtractor::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  bool get_param_ok = true;
  bool prev_active = p_active_;

  get_param_ok = nh_local_.param<bool>("active", p_active_, true);

  get_param_ok = nh_local_.param<double>("p_x_min_range_", p_x_min_range_, 0.0);
  get_param_ok = nh_local_.param<double>("p_x_max_range_", p_x_max_range_, 2.0);
  get_param_ok = nh_local_.param<double>("p_y_min_range_", p_y_min_range_, 0.0);
  get_param_ok = nh_local_.param<double>("p_y_max_range_", p_y_max_range_, 3.0);
  get_param_ok = nh_local_.param<double>("obstacle_radius", p_radius_, 0.1);

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      obstacles_sub_ = nh_.subscribe("obstacles_to_map", 10, &AreaObstaclesExtractor::obstacleCallback, this);
      obstacles_array_pub_ = nh_.advertise<costmap_converter::ObstacleArrayMsg>("obstacle_array", 10);
      marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    }
    else
    {
      obstacles_sub_.shutdown();
      obstacles_array_pub_.shutdown();
      marker_pub_.shutdown();
    }
  }

  if (get_param_ok)
  {
    ROS_INFO_STREAM("[Area Obstacles Extractor]: "
                    << "param set ok");
  }
  else
  {
    ROS_WARN_STREAM("[Area Obstacles Extractor]: "
                    << "param set fail");
  }
  return true;
}

void AreaObstaclesExtractor::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr)
{
  ros::Time now = ros::Time::now();
  output_obstacles_array_.obstacles.clear();
  output_obstacles_array_.header.stamp = now;
  output_obstacles_array_.header.frame_id = ptr->header.frame_id;

  output_marker_array_.markers.clear();

  for (const obstacle_detector::CircleObstacle& circle : ptr->circles)
  {
    if (circle.center.x > 0 && circle.center.x < 2)
      if (circle.center.y > 0 && circle.center.y < 3)
      {
        costmap_converter::ObstacleMsg obstacle_msg;
        obstacle_msg.header.frame_id = ptr->header.frame_id;
        obstacle_msg.header.stamp = now;

        geometry_msgs::Point32 point;
        point.x = circle.center.x;
        point.y = circle.center.y;
        obstacle_msg.polygon.points.push_back(point);

        obstacle_msg.radius = p_radius_;

        output_obstacles_array_.obstacles.push_back(obstacle_msg);

        visualization_msgs::Marker marker;
        marker.header.frame_id = ptr->header.frame_id;
        marker.header.stamp = now;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = circle.center.x;
        marker.pose.position.y = circle.center.y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.5;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        marker.color.a = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        output_marker_array_.markers.push_back(marker);
      }
  }
  publishObstacles();
  publishMarkers();
}

void AreaObstaclesExtractor::publishObstacles()
{
  obstacles_array_pub_.publish(output_obstacles_array_);
}

void AreaObstaclesExtractor::publishMarkers()
{
  marker_pub_.publish(output_marker_array_);
}
