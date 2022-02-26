#pragma once

#include <cmath>

namespace lidar_localization
{
namespace Util
{

template <class T, class S>
double length(T& pose1, S& pose2)
{
  return sqrt(pow(pose1.x - pose2.x, 2.) + pow(pose1.y - pose2.y, 2.));
}

template <class T>
double length(T& pose1)
{
  return sqrt(pow(pose1.x, 2.) + pow(pose1.y, 2.));
}

double length(double x, double y)
{
  return sqrt(pow(x, 2.) + pow(y, 2.));
}

double length(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x1 - x2, 2.) + pow(y1 - y2, 2.));
}

bool approxEqual(double measure, double expect, double tolerance)
{
  return std::abs(measure - expect) < tolerance;
}
}  // namespace Util
}  // namespace lidar_localization
