/**
 *
 * @file lidar_localization.cpp
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
 * @date 2021-05-11
 *
 */

#include "lidar_localization/lidar_localization.h"

using namespace lidar_localization;
using namespace arma;

LidarLocalization::LidarLocalization(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local), tf2_listener_(tf2_buffer_)
{
  params_srv_ = nh_local_.advertiseService("params", &LidarLocalization::updateParams, this);
  initialize();
}

LidarLocalization::~LidarLocalization()
{
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("covariance");
  nh_local_.deleteParam("landmarks");
  nh_local_.deleteParam("map_frame");
  nh_local_.deleteParam("base_link_frame");
}

bool LidarLocalization::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  bool get_param_ok = true;
  bool prev_active = p_active_;

  get_param_ok = nh_local_.param<bool>("active", p_active_, true);

  p_covariance_.clear();
  p_covariance_.resize(36, 0.0);
  get_param_ok = nh_local_.getParam("covariance", p_covariance_);

  p_landmarks_.clear();
  p_landmarks_.resize(36, 0.0);
  get_param_ok = nh_local_.getParam("landmarks", p_landmarks_);

  get_param_ok = nh_local_.param<std::string>("map_frame", p_map_frame_id_, std::string("map"));
  get_param_ok = nh_local_.param<std::string>("base_link_frame", p_base_frame_id_, std::string("base_link"));

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      sub_obstacles_ = nh_.subscribe("obstacles", 10, &LidarLocalization::obstacleCallback, this);
      pub_robot_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar_pose", 10);
      pub_landmarks_ = nh_.advertise<visualization_msgs::MarkerArray>("landmarks_marker", 10);
    }
    else
    {
      sub_obstacles_.shutdown();
      pub_robot_pose_.shutdown();
      pub_landmarks_.shutdown();
    }
  }

  if (get_param_ok)
  {
    ROS_INFO_STREAM("[Lidar Localization]: "
                    << "set param ok");
  }
  else
  {
    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "set param failed");
  }

  std::cout << "cov: " << p_covariance_.size() << std::endl;
  for (const auto& e : p_covariance_)
  {
    std::cout << e << ", ";
  }

  std::cout << std::endl;

  std::cout << "lm: " << p_landmarks_.size() <<std::endl;
  for (const auto& e : p_landmarks_)
  {
    std::cout << e << ", ";
  }
  std::cout << std::endl;
  return true;
}

void LidarLocalization::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr)
{
}

void LidarLocalization::publishRobotPose()
{
  ros::Time now = ros::Time::now();

  output_robot_pose_.header.frame_id = p_map_frame_id_;
  output_robot_pose_.header.stamp = now;

  // clang-format off
                                       // x         y         z  pitch roll yaw
    output_robot_pose_.pose.covariance = {0, 0,        0, 0,    0,   0,
                                          0,        0, 0, 0,    0,   0,
                                          0,        0,        0, 0,    0,   0,
                                          0,        0,        0, 0,    0,   0,
                                          0,        0,        0, 0,    0,   0,
                                          0,        0,        0, 0,    0,   0};
  // clang-format on
  pub_robot_pose_.publish(output_robot_pose_);
}

void LidarLocalization::publishLandmarks()
{
  ros::Time now = ros::Time::now();
}
