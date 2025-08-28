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
MotionCommand Odometry::computeCommands(vector<pair<int,int>> &path) {
    MotionCommand res;
    res.time_sec = 0.0;
    res.angle_deg = 0.0;

    if (path.size() < 2) return res;

    double linear_vel = 1.0; // fixed speed = 1 m/s

    // --- sum distances for all path segments ---
    double total_dist = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        total_dist += distance(path[i-1].first, path[i-1].second,
                               path[i].first, path[i].second);
    }
    res.time_sec = total_dist / linear_vel;

    // --- accumulate turning angles ---
    double prev_angle = angle(path[0].first, path[0].second,
                              path[1].first, path[1].second);
    for (size_t i = 2; i < path.size(); i++) {
        double curr_angle = angle(path[i-1].first, path[i-1].second,
                                  path[i].first, path[i].second);
        double dtheta = curr_angle - prev_angle;

        // normalize to [-180, 180]
        while (dtheta > 180) dtheta -= 360;
        while (dtheta < -180) dtheta += 360;

        res.angle_deg += fabs(dtheta);
        prev_angle = curr_angle;
    }

 return res;
}