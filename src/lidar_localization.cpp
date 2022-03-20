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
  else
  {
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
  p_landmarks_count_ = p_landmarks_.size();

  // update polygons
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
  ROS_INFO_STREAM("cb start");
  ros::Time start = ros::Time::now();
  getRobotPosefromTF();
  input_obstacles_.clear();
  for (const auto& obstacle : ptr->circles)
  {
    input_obstacles_.push_back(obstacle.center);
  }
  ROS_INFO_STREAM("pushback finished");

  findLandmarks();
  ROS_INFO_STREAM("findLandmarks finished");

  publishRobotPose();
  ROS_INFO_STREAM("publishRobotPose finished");
  ros::Time stop = ros::Time::now();

  // ROS_INFO_STREAM("size: " << std::setw(6) << polygons.size() << ", Took " << std::setprecision(5) << stop - start);
}

void LidarLocalization::publishRobotPose()
{
  ros::Time now = ros::Time::now();

  output_robot_pose_.header.frame_id = p_map_frame_id_;
  output_robot_pose_.header.stamp = now;
  if (!polygons.size())
  {
    return;
  }
  double min_cost = polygons[0].cost;
  int min_index = 0;
  for (int i = 0; i < polygons.size(); i++)
  {
    if (polygons[i].cost < min_cost)
    {
      min_cost = polygons[i].cost;
      min_index = i;
    }
  }
  if (min_cost < 0.1)
  {
    output_robot_pose_.pose.pose = polygons[min_index].robot_pose;

    // clang-format off
                                      //x  y  z  pitch roll yaw
    output_robot_pose_.pose.covariance = {0.01, 0, 0,    0,    0,  0,
                                          0, 0.01, 0,    0,    0,  0,
                                          0, 0, 0,    0,    0,  0,
                                          0, 0, 0,    0,    0,  0,
                                          0, 0, 0,    0,    0,  0,
                                          0, 0, 0,    0,    0,  0.01};
    // clang-format on
    pub_robot_pose_.publish(output_robot_pose_);
  }
}

void LidarLocalization::publishLandmarks()
{
  ros::Time now = ros::Time::now();
}

void LidarLocalization::getRobotPosefromTF()
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf2_buffer_.lookupTransform(p_map_frame_id_, p_base_frame_id_, ros::Time());
    robot_pose_now.position.x = transform.transform.translation.x;
    robot_pose_now.position.y = transform.transform.translation.y;
    robot_pose_now.orientation = transform.transform.rotation;
  }
  catch (const tf2::TransformException& ex)
  {
    try
    {
      transform = tf2_buffer_.lookupTransform(p_map_frame_id_, p_base_frame_id_, ros::Time());
      robot_pose_now.position.x = transform.transform.translation.x;
      robot_pose_now.position.y = transform.transform.translation.y;
      robot_pose_now.orientation = transform.transform.rotation;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM(ex.what());
    }
  }
}

// TODO: add filter and test time
void LidarLocalization::findLandmarks()
{
  // TODO: filter obstacle not possible out

  // std::vector<int> possible_obstacles;
  // int l = input_obstacles_.size();
  // for (int i = 0; i < l; i++)
  // {
  //   for (int j = 0; j < i + 1; j++)
  //   {
  //     double edge_len = Util::length(input_obstacles_[i], input_obstacles_[j]);
  //   }
  // }
  if (!input_obstacles_.size())
    return;
  std::vector<int> possible_obstacles(input_obstacles_.size());
  std::iota(possible_obstacles.begin(), possible_obstacles.end(), 0);

  // calc all distance for obstacles
  obstacles_length.clear();
  obstacles_length.resize(input_obstacles_.size());
  for (const auto& idx : possible_obstacles)
  {
    obstacles_length[idx] = Util::length(input_obstacles_[idx]);
  }

  // iterate all possible polygon
  polygons.clear();
  do
  {
    Polygon polygon;
    for (int i = 0; i < p_landmarks_count_; i++)
    {
      polygon.vertices.push_back(possible_obstacles[i]);
    }

    bool is_edge_length_match = true;
    for (int i = 0; i < p_landmarks_count_; i++)
    {
      std::vector<double> row;
      for (int j = 0; j < i + 1; j++)
      {
        double edge_len = Util::length(input_obstacles_[polygon.vertices[i]], input_obstacles_[polygon.vertices[j]]);
        // TODO: parameterize 0.1 as tolerance
        if (abs(edge_len - landmarks_length[i][j]) > 0.1)
        {
          is_edge_length_match = false;
          break;
        }
      }

      if (is_edge_length_match)
      {
        polygon.edge_length.push_back(row);
      }
      else
      {
        break;
      }
    }

    if (is_edge_length_match)
    {
      calcRobotPose(polygon);
      arma::vec3 a, b;
      a << input_obstacles_[polygon.vertices[1]].x - input_obstacles_[polygon.vertices[0]].x << input_obstacles_[polygon.vertices[1]].y - input_obstacles_[polygon.vertices[0]].y << 0;
      b << input_obstacles_[polygon.vertices[2]].x - input_obstacles_[polygon.vertices[1]].x << input_obstacles_[polygon.vertices[2]].y - input_obstacles_[polygon.vertices[1]].y << 0;
      vec c = arma::cross(a, b);
      // TODO: parameterize range
      if (polygon.robot_pose.position.x > 0 && polygon.robot_pose.position.x < 2.0)
      {
        if (polygon.robot_pose.position.y > 0 && polygon.robot_pose.position.y < 3.0)
        {
          if (c(2) > 0)
          {
            polygons.push_back(polygon);
          }
        }
      }
    }
    std::reverse(possible_obstacles.begin() + p_landmarks_count_, possible_obstacles.end());
  } while (std::next_permutation(possible_obstacles.begin(), possible_obstacles.end()));
}

void LidarLocalization::calcRobotPose(Polygon& polygon)
{
  mat A(2, 2);
  vec b(2);
  vec X(2);
  // clang-format off
  A(0, 0) = 2 * (p_landmarks_[0].x - p_landmarks_[2].x);
  A(0, 1) = 2 * (p_landmarks_[0].y - p_landmarks_[2].y);

  A(1, 0) = 2 * (p_landmarks_[1].x - p_landmarks_[2].x);
  A(1, 1) = 2 * (p_landmarks_[1].y - p_landmarks_[2].y);

  b(0) = (pow(p_landmarks_[0].x, 2) - pow(p_landmarks_[2].x, 2)) +
         (pow(p_landmarks_[0].y, 2) - pow(p_landmarks_[2].y, 2)) +
         (pow(obstacles_length[polygon.vertices[2]], 2) - pow(obstacles_length[polygon.vertices[0]], 2));
  b(1) = (pow(p_landmarks_[1].x, 2) - pow(p_landmarks_[2].x, 2)) +
         (pow(p_landmarks_[1].y, 2) - pow(p_landmarks_[2].y, 2)) +
         (pow(obstacles_length[polygon.vertices[2]], 2) - pow(obstacles_length[polygon.vertices[1]], 2));
  // clang-format on
  try
  {
    X = solve(A.t() * A, A.t() * b, solve_opts::no_approx);
    polygon.robot_pose.position.x = X(0);
    polygon.robot_pose.position.y = X(1);

    double robot_yaw = 0;
    double robot_sin = 0;
    double robot_cos = 0;

    for (int i = 0; i < 3; i++)
    {
      // clang-format off
      double theta = atan2(p_landmarks_[i].y - polygon.robot_pose.position.y,
                           p_landmarks_[i].x - polygon.robot_pose.position.x) -
                     atan2(input_obstacles_[polygon.vertices[i]].y, input_obstacles_[polygon.vertices[i]].x);
      // clang-format on
      robot_sin += sin(theta);
      robot_cos += cos(theta);
      // ROS_INFO_STREAM("i:" << i << " ," << theta);
    }
    robot_yaw = atan2(robot_sin, robot_cos);
    // ROS_INFO_STREAM("yaw: " << robot_yaw);
    tf2::Quaternion q;
    q.setRPY(0., 0., robot_yaw);
    polygon.robot_pose.orientation = tf2::toMsg(q);
  }
  catch (const std::runtime_error& ex)
  {
    ROS_WARN_STREAM(A);
    ROS_WARN_STREAM(b);
    ROS_WARN_STREAM(ex.what());
  }
}

void LidarLocalization::calcCosts(Polygon& polygon)
{
  tf2::Quaternion q;
  tf2::fromMsg(robot_pose_now.orientation, q);
  tf2::Matrix3x3 mat_robot(q);
  double _, robot_yaw, robot_calc_yaw;
  mat_robot.getRPY(_, _, robot_yaw);

  tf2::fromMsg(polygon.robot_pose.orientation, q);
  tf2::Matrix3x3 mat_calc_robot(q);
  mat_calc_robot.getRPY(_, _, robot_calc_yaw);

  polygon.cost =
      0.7 * (pow(robot_pose_now.position.x - polygon.robot_pose.position.x, 2) + pow(robot_pose_now.position.y - polygon.robot_pose.position.y, 2)) + 0.3 * (pow(robot_calc_yaw - robot_calc_yaw, 2));
}
