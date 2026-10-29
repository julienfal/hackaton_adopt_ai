#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <chrono>
#include <vector>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ugv_navigation/geometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.h"

class PurePursuitNode : public rclcpp::Node
{
    public:
        PurePursuitNode() : Node("pure_pursuit_node")
        {
            RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node has been started.");
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ackermann_steering_controller/odometry", 10, std::bind(&PurePursuitNode::odom_callback, this, std::placeholders::_1));

            path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/planned_path", 10, std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
            command_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_steering_controller/reference", 10);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Publish at 20 Hz, comfortably within the controller's command timeout.
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {return this->on_timer();});

        }
    
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        nav_msgs::msg::Path current_path_;
        bool goal_reached_ = false;
        bool odom_received_ = false;
        rclcpp::Time last_odom_stamp_{0, 0, this->get_clock()->get_clock_type()};
        Pose2D robot_pose_;


        void publishCommand(double speed, double angular_speed){
            auto command = geometry_msgs::msg::TwistStamped();
            command.header.frame_id = "rear_axle";
            command.header.stamp = this->now();
            command.twist.linear.x = speed;
            command.twist.angular.z = angular_speed;
            command_publisher_->publish(command);

        }


        void on_timer()
        {
            if (goal_reached_ || current_path_.poses.empty() || current_path_.header.frame_id.empty())
            {
                publishCommand(0.0, 0.0);
                return;
            }

            if (!odom_received_)
            {
                publishCommand(0.0, 0.0);
                return;
            }

            const double odom_age = (this->now() - last_odom_stamp_).seconds();
            if (odom_age < 0.0 || odom_age > 0.5)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Odometry age %.3f s is outside [0, 0.5] s; stopping.", odom_age);
                publishCommand(0.0, 0.0);
                return;
            }

            if (current_path_.header.frame_id != "odom")
            {
                RCLCPP_WARN(
                    this->get_logger(), "Expected path frame 'odom', received '%s'.",
                    current_path_.header.frame_id.c_str());
                publishCommand(0.0, 0.0);
                return;
            }

            std::vector<Point2D> path_points;
            path_points.reserve(current_path_.poses.size());
            for (const auto& pose : current_path_.poses)
            {
                if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
                    !std::isfinite(pose.pose.position.z))
                {
                    publishCommand(0.0, 0.0);
                    return;
                }
                path_points.push_back({pose.pose.position.x, pose.pose.position.y});
            }

            const Point2D robot_position{robot_pose_.x, robot_pose_.y};
            if (!std::isfinite(robot_position.x) || !std::isfinite(robot_position.y))
            {
                publishCommand(0.0, 0.0);
                return;
            }

            const double lookahead_distance = 1.5; // Metres; temporary simulation setting.
            // Near the end, all remaining points can be inside the lookahead circle.
            const std::size_t target_index = findLookaheadPointIndex(
                robot_position, path_points, lookahead_distance).value_or(path_points.size() - 1);

            geometry_msgs::msg::PointStamped target_point;
            target_point.header.frame_id = current_path_.header.frame_id;
            target_point.point = current_path_.poses[target_index].pose.position;
            target_point.header.stamp = rclcpp::Time(0); // Latest available transform.

            try {
                // Inspect the dynamic odom transform and reuse this exact snapshot below.
                const auto transform = tf_buffer_->lookupTransform(
                    "rear_axle", "odom", tf2::TimePointZero);
                const rclcpp::Time tf_stamp(transform.header.stamp, this->get_clock()->get_clock_type());
                const double tf_age = (this->now() - tf_stamp).seconds();
                if (tf_age < 0.0 || tf_age > 0.5)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "TF age %.3f s is outside [0, 0.5] s; stopping.", tf_age);
                    publishCommand(0.0, 0.0);
                    return;
                }

                geometry_msgs::msg::PointStamped goal_point = target_point;
                goal_point.point = current_path_.poses.back().pose.position;
                geometry_msgs::msg::PointStamped goal_in_robot;
                tf2::doTransform(goal_point, goal_in_robot, transform);
                const double goal_distance = std::hypot(goal_in_robot.point.x, goal_in_robot.point.y);
                if (!std::isfinite(goal_distance))
                {
                    publishCommand(0.0, 0.0);
                    return;
                }
                constexpr double goal_tolerance = 0.15; // Metres; position only.
                if (goal_distance <= goal_tolerance)
                {
                    goal_reached_ = true;
                    RCLCPP_INFO(this->get_logger(), "Path complete: endpoint is %.3f m away.", goal_distance);
                    publishCommand(0.0, 0.0);
                    return;
                }

                geometry_msgs::msg::PointStamped point_in_robot;
                tf2::doTransform(target_point, point_in_robot, transform);
                if (!std::isfinite(point_in_robot.point.x) || !std::isfinite(point_in_robot.point.y) ||
                    point_in_robot.point.x <= 0.0)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "Target is invalid or not ahead of the rear axle; stopping.");
                    publishCommand(0.0, 0.0);
                    return;
                }
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Path point in rear_axle: (%.3f, %.3f, %.3f), lookahead index: %zu",
                    point_in_robot.point.x,
                    point_in_robot.point.y,
                    point_in_robot.point.z,
                    target_index);
                Point2D point_express_in_robot_frame;
                point_express_in_robot_frame.x = point_in_robot.point.x;
                point_express_in_robot_frame.y = point_in_robot.point.y;

                std::optional<double> curvature = computeCurvature(point_express_in_robot_frame);

                if (!curvature.has_value() || !std::isfinite(*curvature)){
                    RCLCPP_WARN(
                    this->get_logger(), "Curvature is unavailable or non-finite; stopping.");
                    publishCommand(0.0, 0.0);
                    return;
                }

                // Temporary simulation settings: m/s and inverse metres.
                constexpr double max_speed = 0.2;
                constexpr double slowdown_distance = 1.0; // Metres from endpoint.
                const double speed = max_speed * std::clamp(goal_distance / slowdown_distance, 0.0, 1.0);
                constexpr double max_curvature = 0.5;
                const double limited_curvature = std::clamp(*curvature, -max_curvature, max_curvature);
                const double rotation_speed = speed * limited_curvature;
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Curvature: requested %.3f, limited %.3f", *curvature, limited_curvature);
                publishCommand(speed, rotation_speed);

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(
                    this->get_logger(), "Could not transform from %s to %s: %s",
                    target_point.header.frame_id.c_str(), "rear_axle", ex.what());
                publishCommand(0.0, 0.0);
                return;
            }
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            // Use measurement time, so delayed messages are not treated as fresh.
            last_odom_stamp_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
            if (!odom_received_)
            {
                RCLCPP_INFO(this->get_logger(), "Received first odometry message.");
                odom_received_ = true;
            }
            robot_pose_.x = msg->pose.pose.position.x;
            robot_pose_.y = msg->pose.pose.position.y;
            robot_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
            // RCLCPP_INFO(this->get_logger(), "Received odometry data: Position (%.2f, %.2f), Orientation (%.2f, %.2f, %.2f, %.2f)",
            //             msg->pose.pose.position.x, msg->pose.pose.position.y,
            //             msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            //             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            // RCLCPP_INFO(this->get_logger(), "Yaw: %.2f radians", robot_pose_.yaw);
        }

        void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
        {
            current_path_ = *msg;
            // Each received path is a new tracking request, including a repeated path.
            goal_reached_ = false;
            if (current_path_.poses.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received an empty path.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "frame id: %s ,  number of points: %zu", current_path_.header.frame_id.c_str(), current_path_.poses.size());
            RCLCPP_INFO(this->get_logger(), "First point: Position (%.2f, %.2f), Orientation (%.2f, %.2f, %.2f, %.2f)",
                        current_path_.poses.front().pose.position.x, current_path_.poses.front().pose.position.y,
                        current_path_.poses.front().pose.orientation.x, current_path_.poses.front().pose.orientation.y,
                        current_path_.poses.front().pose.orientation.z, current_path_.poses.front().pose.orientation.w);

            RCLCPP_INFO(this->get_logger(), "last point: Position (%.2f, %.2f), Orientation (%.2f, %.2f, %.2f, %.2f)",
                        current_path_.poses.back().pose.position.x, current_path_.poses.back().pose.position.y,
                        current_path_.poses.back().pose.orientation.x, current_path_.poses.back().pose.orientation.y,
                        current_path_.poses.back().pose.orientation.z, current_path_.poses.back().pose.orientation.w);

        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    {
    Point2D test_point{3.0, 2.0};
    Pose2D test_pose{0.0, 0.0, 0.0}; // 45 degrees in radians
    Point2D transformed_point = transformToRobotFrame(test_pose, test_point);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transformed Point: (%.2f, %.2f)", transformed_point.x, transformed_point.y);
    }
    {
    Point2D test_point{5.0, 3.0};
    Pose2D test_pose{2.0, 1.0, 0.0}; // 45 degrees in radians
    Point2D transformed_point = transformToRobotFrame(test_pose, test_point);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transformed Point: (%.2f, %.2f)", transformed_point.x, transformed_point.y);
    }
    {
    Point2D test_point{5.0, 3.0};
    Pose2D test_pose{2.0, 1.0, M_PI / 2}; // 45 degrees in radians
    Point2D transformed_point = transformToRobotFrame(test_pose, test_point);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transformed Point: (%.2f, %.2f)", transformed_point.x, transformed_point.y);
    }

    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
