#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class TargetPlanner : public rclcpp::Node
{
public:
    TargetPlanner() : Node("target_planner")
    {
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                plan(*msg);
            });

        execute_sub_ = create_subscription<std_msgs::msg::Empty>(
            "execute_plan", 10,
            [this](std_msgs::msg::Empty::SharedPtr /*msg*/)
            {
                execute();
            });
    }

    void init()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_manipulator");
        move_group_->setPlanningTime(10.0);
        move_group_->setGoalPositionTolerance(0.01);
        move_group_->setGoalOrientationTolerance(0.1);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        RCLCPP_INFO(get_logger(), "TargetPlanner ready");
    }

private:
    void plan(const geometry_msgs::msg::PoseStamped& target)
    {
        RCLCPP_INFO(get_logger(),
                    "Planning to: pos=(%.3f, %.3f, %.3f) ori=(%.3f, %.3f, %.3f, %.3f)",
                    target.pose.position.x, target.pose.position.y, target.pose.position.z,
                    target.pose.orientation.x, target.pose.orientation.y,
                    target.pose.orientation.z, target.pose.orientation.w);

        move_group_->setPoseTarget(target.pose, "tool0");

        auto result = move_group_->plan(last_plan_);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            has_plan_ = true;
            RCLCPP_INFO(get_logger(), "Plan succeeded (%zu points). Publish to 'execute_plan' to execute.",
                        last_plan_.trajectory.joint_trajectory.points.size());
        }
        else
        {
            has_plan_ = false;
            RCLCPP_WARN(get_logger(), "Planning failed (error code: %d)", result.val);
        }
    }

    void execute()
    {
        if (!has_plan_)
        {
            RCLCPP_WARN(get_logger(), "No valid plan to execute");
            return;
        }

        RCLCPP_INFO(get_logger(), "Executing plan...");
        auto result = move_group_->execute(last_plan_);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Execution succeeded");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Execution failed (error code: %d)", result.val);
        }

        has_plan_ = false;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr execute_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan last_plan_;
    bool has_plan_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetPlanner>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
