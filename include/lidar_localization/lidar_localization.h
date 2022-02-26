/**
 *
 * @file lidar_localization.h
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
 * @date 2021-05-02
 *
 */

#pragma once

#define _USE_MATH_DEFINES
#include <armadillo>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <queue>
#include <array>
#include <numeric>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <XmlRpcException.h>

#include <lidar_localization/polygon.h>
#include <lidar_localization/util/math_util.h>
#include <obstacle_detector/Obstacles.h>

namespace lidar_localization
{
/**
 * @class LidarLocalization
 * @brief A class that use obstacles to localize robot position
 */
class LidarLocalization
{
public:
  /**
   * @brief Construct for the class `Lidar Localization`
   *
   * @param nh the global node handler
   * @param nh_local the local node handler, use for get params
   */
  LidarLocalization(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~LidarLocalization();

private:
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
   * @brief Topic `lidar_pose` publisher function
   *
   */
  void publishRobotPose();

  /**
   * @brief Tf broadcasters that send three fixed beacons pose to **map**
   *
   */
  void publishLandmarks();

  void findLandmarks();

  /* ros node */
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;

  /* ros inter-node */
  ros::Subscriber sub_obstacles_;
  ros::Publisher pub_robot_pose_;
  ros::Publisher pub_landmarks_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::vector<geometry_msgs::Point> input_obstacles_;
  geometry_msgs::PoseWithCovarianceStamped output_robot_pose_;
  visualization_msgs::MarkerArray output_beacons_;

  /* private variables */
  std::vector<std::vector<double>> landmarks_length;
  std::vector<Polygon> polygons;

  /* ros param */
  bool p_active_;

  std::vector<double> p_covariance_;
  std::vector<geometry_msgs::Point> p_landmarks_;
  int p_landmarks_count_;

  std::string p_map_frame_id_;
  std::string p_base_frame_id_;
};
}  // namespace lidar_localization
