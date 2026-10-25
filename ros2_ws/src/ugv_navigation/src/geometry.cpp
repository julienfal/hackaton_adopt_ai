#include "ugv_navigation/geometry.hpp"

Point2D transformToRobotFrame(const Pose2D& robot_pose, const Point2D& path)
{
    Point2D params;
    double delta_x = path.x - robot_pose.x;
    double delta_y = path.y - robot_pose.y;

    params.x = delta_x * cos(robot_pose.yaw) + delta_y * sin(robot_pose.yaw);
    params.y = -delta_x * sin(robot_pose.yaw) + delta_y * cos(robot_pose.yaw);

    return params;
}
