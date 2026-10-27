#pragma once
#include <cmath>
#include <vector>
#include <optional>
#include <cstddef>

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
std::optional<std::size_t> findClosestPointIndex(const Point2D& robot_position, const std::vector<Point2D>& path_points);
std::optional<std::size_t> findLookaheadPointIndex(const Point2D& robot_position, const std::vector<Point2D>& path_points, double lookahead_distance);
std::optional<double> computeCurvature(const Point2D& point_express_in_robot_frame);

