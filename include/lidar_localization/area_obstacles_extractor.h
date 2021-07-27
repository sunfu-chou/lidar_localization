/**
 *
 * @file area_obstacles_extractor.h
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

#pragma once

#include <ros/ros.h>
#include <vector>

#include <std_srvs/Empty.h>
#include <obstacle_detector/Obstacles.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

namespace lidar_localization
{
/**
 * @class LidarLocalization
 * @brief A class that use obstacles to localize robot position
 */
class AreaObstaclesExtractor
{
public:
  /**
   * @brief Construct for the class `Lidar Localization`
   *
   * @param nh the global node handler
   * @param nh_local the local node handler, use for get params
   */
  AreaObstaclesExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~AreaObstaclesExtractor();

private:
  /**
   * @brief To get params and set to this node in the constructor
   *
   */
  void initialize()
  {
    std_srvs::Empty empt;
    updateParams(empt.request, empt.response);
  }

  /**
   * @brief A service call that get params and set to this node
   *
   * @param req The service request
   * @param res The service response
   * @return true if the service call is succeeds,
   * @return false otherwise
   */
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * @brief Topic `obstacles` callback function
   *
   * @param ptr The obstaacles data
   */
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr);

  /**
   * @brief Topic `odom` callback function
   * 
   * @param ptr The odom data
   *    */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& ptr);

  /**
   * @brief Topic `obstacles_to_map` publisher function
   *
   */
  void publishObstacles();

  /**
   * @brief Topic `obstaclefield_marker` publisher function
   *
   */
  void publishMarkers();

  /* ros node */
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;

  /* ros inter-node */
  ros::Subscriber sub_obstacles_;
  ros::Subscriber sub_obstacles_odom_;
  ros::Publisher pub_obstacles_array_;
  ros::Publisher pub_marker_;

  costmap_converter::ObstacleArrayMsg output_obstacles_array_;
  visualization_msgs::MarkerArray output_marker_array_;
  nav_msgs::Odometry input_odom_;
  /* private variables */

  /* ros param */
  bool p_active_;

  double p_x_min_range_;
  double p_x_max_range_;
  double p_y_min_range_;
  double p_y_max_range_;
  double p_radius_;
};
}  // namespace lidar_localization
