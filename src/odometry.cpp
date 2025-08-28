#include "odometry.h"
#include <ctime>
#include <iterator>
#include <numeric>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  // Linear velocity (m/s) =(wheel circumference * revolutions per second)
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  // atan2 returns radians, convert to degrees
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {

  MotionCommand res = {0.0, 0.0}; // store total time and angle traversed

 /* Implement you odometry logic here */ 

     if (path.size() < 2) return res; // no motion if only one point

  double prev_angle = angle(path[0].first, path[0].second,
                            path[1].first, path[1].second);

  for (size_t i = 0; i < path.size() - 1; i++) {
    // --- Distance and forward time ---
    double d = distance(path[i].first, path[i].second,
                        path[i+1].first, path[i+1].second);
    res.time_sec += d / linear_vel;
//angle_deg
    // --- Turning angle ---
    if (i + 2 < path.size()) {
      double next_angle = angle(path[i+1].first, path[i+1].second,
                                path[i+2].first, path[i+2].second);

      double dtheta = next_angle - prev_angle;
      while (dtheta > 180) dtheta -= 360;
      while (dtheta < -180) dtheta += 360;

      res.angle_deg += dtheta;
      prev_angle = next_angle;
    }
  }

  return res;
}
