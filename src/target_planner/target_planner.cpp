#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <atomic>
#include <mutex>
#include <thread>

class TargetPlanner: public rclcpp::Node {
public:
    TargetPlanner(): Node("target_planner") {
        declare_parameter("planning_time", 5.0);
        declare_parameter("goal_position_tolerance", 0.01);
        declare_parameter("goal_orientation_tolerance", 0.1);
        declare_parameter("max_velocity_scaling_factor", 1.0);
        declare_parameter("max_acceleration_scaling_factor", 1.0);

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                "target_pose", 10,
                [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    if (planning_.exchange(true)) {
                        RCLCPP_WARN(get_logger(), "Planning already in progress, ignoring new target");
                        return;
                    }
                    if (plan_thread_.joinable()) plan_thread_.join();
                    plan_thread_ = std::thread(&TargetPlanner::plan, this, *msg);
                });

        execute_sub_ = create_subscription<std_msgs::msg::Empty>(
                "execute_plan", 10,
                [this](std_msgs::msg::Empty::SharedPtr) {
                    if (executing_.exchange(true)) {
                        RCLCPP_WARN(get_logger(), "Execution already in progress, ignoring");
                        return;
                    }
                    if (execute_thread_.joinable()) execute_thread_.join();
                    execute_thread_ = std::thread(&TargetPlanner::execute, this);
                });
    }

    void init() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "ur_manipulator");
        move_group_->setPlanningTime(get_parameter("planning_time").as_double());
        move_group_->setGoalPositionTolerance(get_parameter("goal_position_tolerance").as_double());
        move_group_->setGoalOrientationTolerance(get_parameter("goal_orientation_tolerance").as_double());
        move_group_->setMaxVelocityScalingFactor(get_parameter("max_velocity_scaling_factor").as_double());
        move_group_->setMaxAccelerationScalingFactor(get_parameter("max_acceleration_scaling_factor").as_double());
        RCLCPP_INFO(get_logger(), "TargetPlanner ready");
    }

    ~TargetPlanner() override {
        if (plan_thread_.joinable()) plan_thread_.join();
        if (execute_thread_.joinable()) execute_thread_.join();
    }

private:
    std::atomic<bool> has_plan_{false};
    std::atomic<bool> planning_{false};
    std::atomic<bool> executing_{false};
    std::mutex plan_mutex_;
    std::thread plan_thread_;
    std::thread execute_thread_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr execute_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan last_plan_;

    void plan(const geometry_msgs::msg::PoseStamped& target) {
        RCLCPP_INFO(get_logger(),
                    "Planning to: pos=(%.3f, %.3f, %.3f) ori=(%.3f, %.3f, %.3f, %.3f)",
                    target.pose.position.x, target.pose.position.y, target.pose.position.z,
                    target.pose.orientation.x, target.pose.orientation.y,
                    target.pose.orientation.z, target.pose.orientation.w);

        move_group_->setPoseTarget(target.pose, "tool0");

        moveit::planning_interface::MoveGroupInterface::Plan candidate;
        moveit::core::MoveItErrorCode result;
        constexpr int max_attempts = 3;
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            result = move_group_->plan(candidate);
            if (result == moveit::core::MoveItErrorCode::SUCCESS) break;
            RCLCPP_WARN(get_logger(), "Planning attempt %d/%d failed (error: %d)", attempt, max_attempts, result.val);
        }

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            {
                std::lock_guard lock(plan_mutex_);
                last_plan_ = std::move(candidate);
            }
            has_plan_ = true;
            RCLCPP_INFO(get_logger(), "Plan succeeded (%zu points). Publish to 'execute_plan' to execute.",
                        last_plan_.trajectory.joint_trajectory.points.size());
        } else {
            has_plan_ = false;
            RCLCPP_WARN(get_logger(), "Planning failed (error code: %d)", result.val);
        }

        planning_ = false;
    }

    void execute() {
        if (!has_plan_) {
            RCLCPP_WARN(get_logger(), "No valid plan to execute");
            executing_ = false;
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan_copy;
        {
            std::lock_guard lock(plan_mutex_);
            plan_copy = last_plan_;
            has_plan_ = false;
        }

        RCLCPP_INFO(get_logger(), "Executing plan...");
        const auto result = move_group_->execute(plan_copy);

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(get_logger(), "Execution succeeded");
        } else {
            RCLCPP_ERROR(get_logger(), "Execution failed (error code: %d)", result.val);
        }
        executing_ = false;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetPlanner>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
