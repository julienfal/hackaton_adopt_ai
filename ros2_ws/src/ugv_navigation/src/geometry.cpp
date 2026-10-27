#include "ugv_navigation/geometry.hpp"

#include <limits>

//
//  4. Transform the goal point into vehicle coordinates

Point2D transformToRobotFrame(const Pose2D& robot_pose, const Point2D& path)
{
    Point2D params;
    double delta_x = path.x - robot_pose.x;
    double delta_y = path.y - robot_pose.y;

    params.x = delta_x * cos(robot_pose.yaw) + delta_y * sin(robot_pose.yaw);
    params.y = -delta_x * sin(robot_pose.yaw) + delta_y * cos(robot_pose.yaw);

    return params;
}

//2. Find the closest path point
std::optional<std::size_t> findClosestPointIndex(const Point2D& robot_position, const std::vector<Point2D>& path_points)
{
    if (path_points.empty())
    {
        return std::nullopt; // Return an empty optional if the path is empty
    }
    double min_distance_squared = std::numeric_limits<double>::max();
    std::size_t closest_index = 0;
    for (std::size_t i = 0; i < path_points.size(); ++i)
    {
        const double dx = path_points[i].x - robot_position.x;
        const double dy = path_points[i].y - robot_position.y;
        const double distance_squared = dx * dx + dy * dy;

        // Update only for a strictly closer point, keeping the first index on ties.
        if (distance_squared < min_distance_squared)
        {
            min_distance_squared = distance_squared;
            closest_index = i;
        }
    }
    return closest_index;
}

/*Find the goal point*/
std::optional<std::size_t> findLookaheadPointIndex(const Point2D& robot_position, const std::vector<Point2D>& path_points, double lookahead_distance)
{
    if (path_points.empty() || lookahead_distance <= 0.0)
    {
        return std::nullopt; // Return an empty optional if the path is empty
    }
    auto closest_index = findClosestPointIndex(robot_position, path_points);
    if (!closest_index.has_value())
    {
        return std::nullopt;
    }

    for(std::size_t i=closest_index.value(); i<path_points.size(); i++){
        
        const double dx = path_points[i].x - robot_position.x;
        const double dy = path_points[i].y - robot_position.y;
        const double distance_squared = dx * dx + dy * dy;

        if (distance_squared >= lookahead_distance * lookahead_distance)
        {
            return i; // Return the index of the first point that is at least the lookahead distance away
        }
    }

    return std::nullopt; // Return an empty optional if no point is found
}

/* 5 Calculate the curvature and request the vehicle to set the steering to that curvature*/

std::optional<double> computeCurvature(const Point2D& point_express_in_robot_frame)
{
    const double min_target_distance = 1e-6; // Minimum distance to avoid division by zero
    const double distance_squared = point_express_in_robot_frame.x * point_express_in_robot_frame.x + point_express_in_robot_frame.y * point_express_in_robot_frame.y;
    if (distance_squared <= min_target_distance * min_target_distance)
    {
        return std::nullopt; // Return an empty optional if the point is at or too close to the origin.
    }
    /*\[
    \kappa=\frac{2y_L}{x_L^2+y_L^2}
    \]*/
    double curvature = (2.0 * point_express_in_robot_frame.y) / distance_squared;
    return curvature;
}