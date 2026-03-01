#ifndef KINEENVIRONMENT_HPP
#define KINEENVIRONMENT_HPP

#include <chrono>
#include <memory>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>

#include <moveit_msgs/msg/display_trajectory.hpp>

#include <threepp/objects/Robot.hpp>

#include "TrajectoryAnimator.hpp"

using namespace threepp;

class KineEnvironmentNode : public rclcpp::Node
{
public:
    KineEnvironmentNode();

    void run();

    ~KineEnvironmentNode() override;

private:
    std::string urdf_;
    std::shared_ptr<Robot> robot_, ghost_;
    std::vector<std::string> jointNames_;

    bool goal_planning_;
    std::unique_ptr<TrajectoryAnimator> animator_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr execute_pub_;

    std::jthread thread_;
};

#endif // KINEENVIRONMENT_HPP
