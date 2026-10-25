#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ugv_navigation/geometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>


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
        }
    
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

        nav_msgs::msg::Path current_path_;
        bool odom_received_ = false;
        Pose2D robot_pose_;


        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            if (!odom_received_)
            {
                RCLCPP_INFO(this->get_logger(), "Received first odometry message.");
                odom_received_ = true;
            }
            robot_pose_.x = msg->pose.pose.position.x;
            robot_pose_.y = msg->pose.pose.position.y;
            robot_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
            RCLCPP_INFO(this->get_logger(), "Received odometry data: Position (%.2f, %.2f), Orientation (%.2f, %.2f, %.2f, %.2f)",
                        msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            RCLCPP_INFO(this->get_logger(), "Yaw: %.2f radians", robot_pose_.yaw);  
        }

        void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
        {
            current_path_ = *msg;
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