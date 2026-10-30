#include <memory>
#include <chrono>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
using namespace std::chrono_literals;

using std::placeholders::_1;

class TrackingEvaluatorNode : public rclcpp::Node
{
  public:
    TrackingEvaluatorNode()
    : Node("tracking_evaluator_node")
    {
        planned_path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10, std::bind(&TrackingEvaluatorNode::planned_path_callback, this, _1));

        // This subscription accepts odometry from both reliable and best-effort publishers.
        rclcpp::SensorDataQoS odometry_qos;
        robot_pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/ackermann_steering_controller/odometry", odometry_qos,
          std::bind(&TrackingEvaluatorNode::robot_pose_callback, this, _1));

        // Retain the latest trace for RViz subscribers that connect later.
        rclcpp::QoS path_qos(1);
        path_qos.transient_local();
        actual_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
          "/actual_path", path_qos);
        timer_ = this->create_wall_timer(
        100ms, std::bind(&TrackingEvaluatorNode::actual_path_callback, this));
    }

  private:
    void planned_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
      ref_path = *msg;
      recording_active = false;
      odom_received = false;

      recorded_actual_path.poses.clear();
      recorded_actual_path.header.frame_id = "odom";
      recorded_actual_path.header.stamp = this->now();

      // Clear RViz even when the new path cancels recording or is invalid.
      actual_path_publisher_->publish(recorded_actual_path);

      if (ref_path.header.frame_id != "odom")
      {
        RCLCPP_WARN(this->get_logger(), "Reference path must be in odom. Recording is inactive.");
        return;
      }

      if (ref_path.poses.empty())
      {
        RCLCPP_INFO(this->get_logger(), "Empty reference path: trajectory cleared and recording stopped.");
        return;
      }

      for (const geometry_msgs::msg::PoseStamped& pose : ref_path.poses)
      {
        // An empty pose frame means we use the frame from the Path header.
        if (!pose.header.frame_id.empty() && pose.header.frame_id != "odom")
        {
          RCLCPP_WARN(this->get_logger(), "A reference pose has the wrong frame. Recording is inactive.");
          return;
        }

        const geometry_msgs::msg::Point& position = pose.pose.position;
        // isfinite rejects NaN and infinity, which are not usable coordinates.
        bool position_valid = std::isfinite(position.x) &&
                              std::isfinite(position.y) &&
                              std::isfinite(position.z);
        if (!position_valid)
        {
          RCLCPP_WARN(this->get_logger(), "Reference path has invalid coordinates. Recording is inactive.");
          return;
        }
      }

      recording_active = true;
      RCLCPP_INFO(this->get_logger(), "Trajectory cleared. Recording started.");
    }

    void robot_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      if (!recording_active)
      {
        return;
      }

      // Do not let the timer reuse a previous pose if this new message is invalid.
      odom_received = false;

      if (msg->header.frame_id != "odom" || msg->child_frame_id != "rear_axle")
      {
        // Limit repeated warnings to one per second.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Ignoring odometry: expected odom and rear_axle frames.");
        return;
      }

      const geometry_msgs::msg::Point& position = msg->pose.pose.position;
      const geometry_msgs::msg::Quaternion& orientation = msg->pose.pose.orientation;
      bool position_valid = std::isfinite(position.x) &&
                            std::isfinite(position.y) &&
                            std::isfinite(position.z);
      bool orientation_valid = std::isfinite(orientation.x) &&
                               std::isfinite(orientation.y) &&
                               std::isfinite(orientation.z) &&
                               std::isfinite(orientation.w);

      if (!position_valid || !orientation_valid)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Ignoring odometry: position or orientation contains NaN or infinity.");
        return;
      }

      latest_odom = *msg;
      odom_received = true;
    }

    void actual_path_callback()
    {
      if (!recording_active)
      {
        return;
      }

      if (!odom_received)
      {
        return;
      }

      // Use the node's clock type so timestamp comparisons also work in simulation.
      rclcpp::Time odom_time(latest_odom.header.stamp, this->get_clock()->get_clock_type());
      double odom_age = (this->now() - odom_time).seconds();

      if (odom_age < 0.0 || odom_age > 0.5)
      {
        return;
      }

      // Only append measurements newer than the last pose already in the trace.
      if (!recorded_actual_path.poses.empty())
      {
        rclcpp::Time last_recorded_time(
          recorded_actual_path.poses.back().header.stamp, this->get_clock()->get_clock_type());

        if (odom_time <= last_recorded_time)
        {
          return;
        }
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header = latest_odom.header;
      pose.pose = latest_odom.pose.pose;
      recorded_actual_path.poses.push_back(pose);
      recorded_actual_path.header.stamp = pose.header.stamp;
      actual_path_publisher_->publish(recorded_actual_path);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planned_path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_publisher_;
    nav_msgs::msg::Path ref_path;
    nav_msgs::msg::Path recorded_actual_path;
    nav_msgs::msg::Odometry latest_odom;
    bool odom_received = false;
    bool recording_active = false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingEvaluatorNode>());
  rclcpp::shutdown();
  return 0;
}
