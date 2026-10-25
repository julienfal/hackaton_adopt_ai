#pragma once
#include <cmath>


struct Point2D
{
    double x;
    double y;
};

struct Pose2D
{
    double x;
    double y;
    double yaw;
};

Point2D transformToRobotFrame(const Pose2D& robot_pose, const Point2D& path);

