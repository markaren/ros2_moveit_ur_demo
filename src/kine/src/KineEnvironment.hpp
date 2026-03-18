#ifndef KINEENVIRONMENT_HPP
#define KINEENVIRONMENT_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

#include <target_planner/action/execute.hpp>
#include <target_planner/action/plan.hpp>
#include <target_planner/action/plan_and_execute.hpp>

#include <threepp/objects/Robot.hpp>

#include "TrajectoryAnimator.hpp"

using namespace threepp;

class KineEnvironmentNode : public rclcpp::Node
{
public:
    KineEnvironmentNode();

    ~KineEnvironmentNode() override;

private:
    using Plan = target_planner::action::Plan;
    using Execute = target_planner::action::Execute;
    using PlanAndExecute = target_planner::action::PlanAndExecute;

    std::string urdf_;
    std::vector<std::string> jointNames_;
    std::shared_ptr<Robot> robot_, planner_ghost_, ik_ghost_;

    bool goal_planning_;
    std::unique_ptr<TrajectoryAnimator> animator_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    rclcpp_action::Client<Plan>::SharedPtr plan_client_;
    rclcpp_action::Client<Execute>::SharedPtr execute_client_;
    rclcpp_action::Client<PlanAndExecute>::SharedPtr plan_and_execute_client_;

    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> planner_params_client_;

    std::atomic<bool> servers_ready_{false};
    std::atomic<bool> action_busy_{false};
    std::atomic<bool> has_plan_{false};
    std::atomic<bool> ik_pending_{false};
    std::mutex status_mutex_;
    std::string action_status_;

    std::function<void()> cancel_fn_;
    std::mutex cancel_mutex_;
    std::atomic<bool> cancel_requested_{false};

    std::mutex robot_mutex_;
    std::mutex ik_ghost_mutex_;
    std::jthread thread_;

    void run();

    void requestIK(const geometry_msgs::msg::Pose& target_pose);

    void sendPlanGoal(const geometry_msgs::msg::PoseStamped& target);
    void sendExecuteGoal();
    void sendPlanAndExecuteGoal(const geometry_msgs::msg::PoseStamped& target);

    void setStatus(const std::string& status);
};

#endif// KINEENVIRONMENT_HPP
