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
  std_srvs::Empty empt;
  updateParams(empt.request, empt.response);
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
  bool prev_active = p_active_;

  // get bool param "active"
  if (!nh_local_.param<bool>("active", p_active_, true))
  {
    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "set param failed: "
                    << "active");
  }

  // get list param "covariance"
  p_covariance_.clear();
  if (!nh_local_.getParam("covariance", p_covariance_))
  {
    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "set param failed: "
                    << "covariance");
    p_covariance_.clear();
    p_covariance_.resize(36, 0);
    p_covariance_[0] = 0.1;
    p_covariance_[7] = 0.1;
    p_covariance_[35] = 0.1;
  }

  // get list of list param "landmarks"
  p_landmarks_.clear();
  XmlRpc::XmlRpcValue landmarks;

  if (nh_local_.hasParam("landmarks"))
  {
    try
    {
      nh_local_.getParam("landmarks", landmarks);
      ROS_ASSERT(landmarks.getType() == XmlRpc::XmlRpcValue::TypeArray);

      p_landmarks_count_ = landmarks.size();
      for (int i = 0; i < p_landmarks_count_; i++)
      {
        geometry_msgs::Point landmark;
        landmark.x = landmarks[i][0];
        landmark.y = landmarks[i][1];
        p_landmarks_.push_back(landmark);
      }
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_WARN_STREAM("[Lidar Localization]: "
                      << "set param failed: "
                      << "landmarks");
      ROS_WARN_STREAM("[Lidar Localization]: " << e.getMessage());
      p_landmarks_.clear();

      geometry_msgs::Point landmark;
      landmark.x = 1.00;
      landmark.y = -0.05;
      p_landmarks_.push_back(landmark);
      landmark.x = 1.953;
      landmark.y = 3.1;
      p_landmarks_.push_back(landmark);
      landmark.x = 0.055;
      landmark.y = 3.1;
      p_landmarks_.push_back(landmark);
    }
  }

  // update landmarks_length
  landmarks_length.clear();
  for (int i = 0; i < p_landmarks_count_; i++)
  {
    std::vector<double> row;
    for (int j = 0; j < i + 1; j++)
    {
      double edge_len = Util::length(p_landmarks_[i], p_landmarks_[j]);
      row.push_back(edge_len);
    }
    landmarks_length.push_back(row);
  }

  // get string params
  nh_local_.param<std::string>("map_frame", p_map_frame_id_, std::string("map"));
  nh_local_.param<std::string>("base_link_frame", p_base_frame_id_, std::string("base_link"));

  if (p_active_ != prev_active)
  {
    // start sub and pubs
    if (p_active_)
    {
      sub_obstacles_ = nh_.subscribe("obstacles", 10, &LidarLocalization::obstacleCallback, this);
      pub_robot_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar_pose", 10);
      pub_landmarks_ = nh_.advertise<visualization_msgs::MarkerArray>("landmarks_marker", 10);
    }
    // stop sub and pubs
    else
    {
      sub_obstacles_.shutdown();
      pub_robot_pose_.shutdown();
      pub_landmarks_.shutdown();
    }
  }

  return true;
}

void LidarLocalization::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr)
{
  input_obstacles_.clear();
  for (const auto& obstacle : ptr->circles)
  {
    input_obstacles_.push_back(obstacle.center);
  }

  findLandmarks();
}

void LidarLocalization::publishRobotPose()
{
  ros::Time now = ros::Time::now();

  output_robot_pose_.header.frame_id = p_map_frame_id_;
  output_robot_pose_.header.stamp = now;

  // clang-format off
                                      //x  y  z  pitch roll yaw
  output_robot_pose_.pose.covariance = {0, 0, 0,    0,    0,  0,
                                        0, 0, 0,    0,    0,  0,
                                        0, 0, 0,    0,    0,  0,
                                        0, 0, 0,    0,    0,  0,
                                        0, 0, 0,    0,    0,  0,
                                        0, 0, 0,    0,    0,  0};
  // clang-format on
  pub_robot_pose_.publish(output_robot_pose_);
}

void LidarLocalization::publishLandmarks()
{
  ros::Time now = ros::Time::now();
}

void LidarLocalization::findLandmarks()
{
  /* filter obstacle not possible out */
  // std::vector<int> possible_obstacles;
  // int l = input_obstacles_.size();
  // for (int i = 0; i < l; i++)
  // {
  //   for (int j = 0; j < i + 1; j++)
  //   {
  //     double edge_len = Util::length(input_obstacles_[i], input_obstacles_[j]);
  //   }
  // }(
  std::vector<int> possible_obstacles(input_obstacles_.size());
  std::iota(possible_obstacles.begin(), possible_obstacles.end(), 0);

  polygons.clear();
  do
  {
    Polygon polygon;
    for (int i = 0; i < p_landmarks_count_; i++)
    {
      polygon.vertices.push_back(possible_obstacles[i]);
    }
    polygons.push_back(polygon);
    std::reverse(possible_obstacles.begin() + p_landmarks_count_, possible_obstacles.end());
  } while (std::next_permutation(possible_obstacles.begin(), possible_obstacles.end()));

  // polygons.clear();
  // int l = input_obstacles_.size();

  // for (int i = 0; i < l; i++)
  // {
  //   std::vector<int> vertices;
  //   vertices.push_back(i);
  //   std::queue<int> q;
  //   q.push(i);
  //   while (!q.empty())
  //   {
  //     int f = q.front();
  //     if (vertices.size() == 3)
  //     {
  //       if (isSameShape())
  //       {
  //         Polygon polygon;
  //         polygons.push_back(polygon);
  //       }
  //     }
  //   }
  // }
}
