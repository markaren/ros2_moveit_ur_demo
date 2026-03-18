#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <target_planner/action/execute.hpp>
#include <target_planner/action/plan.hpp>
#include <target_planner/action/plan_and_execute.hpp>

#include <mutex>
#include <thread>

/**
 * @brief ROS 2 node that exposes MoveIt planning and execution as three actions,
 *        mirroring the Plan / Execute / Plan&Execute workflow of the RViz MotionPlanning panel.
 *
 * All goals are rejected while another action is in progress.
 * Call init() after construction to create the MoveGroupInterface.
 *
 * **Actions:**
 * - `plan` (target_planner/action/Plan): plan to a target pose and store the trajectory.
 * - `execute` (target_planner/action/Execute): execute the last stored plan.
 * - `plan_and_execute` (target_planner/action/PlanAndExecute): plan then immediately execute.
 *
 * **Parameters:**
 * - `planning_time` (double, default 5.0): max planning time in seconds.
 * - `goal_position_tolerance` (double, default 0.01): position tolerance in metres.
 * - `goal_orientation_tolerance` (double, default 0.1): orientation tolerance in radians.
 * - `max_velocity_scaling_factor` (double, default 1.0): velocity scaling [0, 1].
 * - `max_acceleration_scaling_factor` (double, default 1.0): acceleration scaling [0, 1].
 */
class TargetPlanner: public rclcpp::Node {
public:
    using Plan = target_planner::action::Plan;
    using Execute = target_planner::action::Execute;
    using PlanAndExecute = target_planner::action::PlanAndExecute;

    using GoalHandlePlan = rclcpp_action::ServerGoalHandle<Plan>;
    using GoalHandleExecute = rclcpp_action::ServerGoalHandle<Execute>;
    using GoalHandlePlanAndExecute = rclcpp_action::ServerGoalHandle<PlanAndExecute>;

    TargetPlanner(): Node("target_planner") {
        declare_parameter("planning_time", 5.0);
        declare_parameter("goal_position_tolerance", 0.01);
        declare_parameter("goal_orientation_tolerance", 0.1);
        declare_parameter("max_velocity_scaling_factor", 1.0);
        declare_parameter("max_acceleration_scaling_factor", 1.0);

        plan_server_ = rclcpp_action::create_server<Plan>(
                this, "plan",
                [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Plan::Goal>) {
                    State expected = State::IDLE;
                    if (!state_.compare_exchange_strong(expected, State::PLANNING)) {
                        RCLCPP_WARN(get_logger(), "Busy (%s), rejecting plan goal", state_name(expected));
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                },
                [](const std::shared_ptr<GoalHandlePlan>) {
                    return rclcpp_action::CancelResponse::ACCEPT;
                },
                [this](std::shared_ptr<GoalHandlePlan> goal_handle) {
                    if (plan_thread_.joinable()) plan_thread_.join();
                    plan_thread_ = std::thread(&TargetPlanner::do_plan, this, goal_handle);
                });

        execute_server_ = rclcpp_action::create_server<Execute>(
                this, "execute",
                [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Execute::Goal>) {
                    State expected = State::IDLE;
                    if (!state_.compare_exchange_strong(expected, State::EXECUTING)) {
                        RCLCPP_WARN(get_logger(), "Busy (%s), rejecting execute goal", state_name(expected));
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                },
                [](const std::shared_ptr<GoalHandleExecute>) {
                    return rclcpp_action::CancelResponse::ACCEPT;
                },
                [this](std::shared_ptr<GoalHandleExecute> goal_handle) {
                    if (execute_thread_.joinable()) execute_thread_.join();
                    execute_thread_ = std::thread(&TargetPlanner::do_execute, this, goal_handle);
                });

        plan_and_execute_server_ = rclcpp_action::create_server<PlanAndExecute>(
                this, "plan_and_execute",
                [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const PlanAndExecute::Goal>) {
                    State expected = State::IDLE;
                    if (!state_.compare_exchange_strong(expected, State::PLANNING)) {
                        RCLCPP_WARN(get_logger(), "Busy (%s), rejecting plan_and_execute goal", state_name(expected));
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                },
                [](const std::shared_ptr<GoalHandlePlanAndExecute>) {
                    return rclcpp_action::CancelResponse::ACCEPT;
                },
                [this](std::shared_ptr<GoalHandlePlanAndExecute> goal_handle) {
                    if (plan_and_execute_thread_.joinable()) plan_and_execute_thread_.join();
                    plan_and_execute_thread_ = std::thread(&TargetPlanner::do_plan_and_execute, this, goal_handle);
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
        if (plan_and_execute_thread_.joinable()) plan_and_execute_thread_.join();
    }

private:
    enum class State { IDLE, PLANNING, EXECUTING };

    static const char* state_name(State s) {
        switch (s) {
            case State::IDLE: return "IDLE";
            case State::PLANNING: return "PLANNING";
            case State::EXECUTING: return "EXECUTING";
        }
        return "UNKNOWN";
    }

    /// RAII guard that resets state to IDLE on scope exit, even if an exception is thrown.
    struct StateGuard {
        std::atomic<State>& state;
        ~StateGuard() { state = State::IDLE; }
    };

    std::atomic<State> state_{State::IDLE};
    std::atomic<bool> has_plan_{false};
    std::mutex plan_mutex_;
    std::mutex move_group_mutex_;
    std::thread plan_thread_;
    std::thread execute_thread_;
    std::thread plan_and_execute_thread_;

    rclcpp_action::Server<Plan>::SharedPtr plan_server_;
    rclcpp_action::Server<Execute>::SharedPtr execute_server_;
    rclcpp_action::Server<PlanAndExecute>::SharedPtr plan_and_execute_server_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan last_plan_;

    /// Runs the planning loop with retry. Calls on_attempt(attempt, max) before each try.
    /// Returns the candidate plan; out_result carries the final error code.
    moveit::planning_interface::MoveGroupInterface::Plan run_planner(
            const geometry_msgs::msg::PoseStamped& target,
            moveit::core::MoveItErrorCode& out_result,
            const std::function<bool()>& is_canceling,
            const std::function<void(int, int)>& on_attempt = nullptr) {
        moveit::planning_interface::MoveGroupInterface::Plan candidate;
        constexpr int max_attempts = 3;

        std::lock_guard lock(move_group_mutex_);
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseReferenceFrame(target.header.frame_id);
        move_group_->setPoseTarget(target.pose, "tool0");

        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            if (is_canceling()) break;
            if (on_attempt) on_attempt(attempt, max_attempts);
            out_result = move_group_->plan(candidate);
            if (out_result == moveit::core::MoveItErrorCode::SUCCESS) break;
            RCLCPP_WARN(get_logger(), "Planning attempt %d/%d failed (error: %d)", attempt, max_attempts, out_result.val);
        }

        move_group_->clearPoseTargets();
        return candidate;
    }

    void do_plan(std::shared_ptr<GoalHandlePlan> goal_handle) {
        StateGuard guard{state_};
        const auto& target = goal_handle->get_goal()->target_pose;
        auto feedback = std::make_shared<Plan::Feedback>();
        auto result = std::make_shared<Plan::Result>();

        RCLCPP_INFO(get_logger(), "Planning to: pos=(%.3f, %.3f, %.3f) frame='%s'",
                    target.pose.position.x, target.pose.position.y, target.pose.position.z,
                    target.header.frame_id.c_str());

        moveit::core::MoveItErrorCode error;
        auto candidate = run_planner(target, error,
                [&] { return goal_handle->is_canceling(); },
                [&](int attempt, int max) {
                    feedback->attempt = static_cast<uint8_t>(attempt);
                    feedback->max_attempts = static_cast<uint8_t>(max);
                    goal_handle->publish_feedback(feedback);
                });

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }

        if (error == moveit::core::MoveItErrorCode::SUCCESS) {
            {
                std::lock_guard lock(plan_mutex_);
                last_plan_ = std::move(candidate);
                has_plan_ = true;
            }
            result->success = true;
            result->trajectory_points = static_cast<uint32_t>(last_plan_.trajectory.joint_trajectory.points.size());
            result->message = "Planning succeeded";
            RCLCPP_INFO(get_logger(), "Plan succeeded (%u points)", result->trajectory_points);
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "Planning failed (error: " + std::to_string(error.val) + ")";
            RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
            goal_handle->abort(result);
        }
    }

    void do_execute(std::shared_ptr<GoalHandleExecute> goal_handle) {
        StateGuard guard{state_};
        auto result = std::make_shared<Execute::Result>();

        moveit::planning_interface::MoveGroupInterface::Plan plan_copy;
        {
            std::lock_guard lock(plan_mutex_);
            if (!has_plan_) {
                result->success = false;
                result->message = "No valid plan to execute";
                RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
                goal_handle->abort(result);
                return;
            }
            plan_copy = last_plan_;
            has_plan_ = false;
        }

        RCLCPP_INFO(get_logger(), "Executing plan...");
        moveit::core::MoveItErrorCode error;
        {
            std::lock_guard lock(move_group_mutex_);
            move_group_->setStartStateToCurrentState();
            error = move_group_->execute(plan_copy);
        }

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }

        if (error == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Execution succeeded";
            RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "Execution failed (error: " + std::to_string(error.val) + ")";
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            goal_handle->abort(result);
        }
    }

    void do_plan_and_execute(std::shared_ptr<GoalHandlePlanAndExecute> goal_handle) {
        StateGuard guard{state_};
        // state_ is already PLANNING (set in handle_goal)
        const auto& target = goal_handle->get_goal()->target_pose;
        auto feedback = std::make_shared<PlanAndExecute::Feedback>();
        auto result = std::make_shared<PlanAndExecute::Result>();

        RCLCPP_INFO(get_logger(), "Plan&Execute to: pos=(%.3f, %.3f, %.3f) frame='%s'",
                    target.pose.position.x, target.pose.position.y, target.pose.position.z,
                    target.header.frame_id.c_str());

        // Phase 1: Plan
        feedback->phase = "PLANNING";
        goal_handle->publish_feedback(feedback);

        moveit::core::MoveItErrorCode error;
        auto candidate = run_planner(target, error,
                [&] { return goal_handle->is_canceling(); });

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }

        if (error != moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = false;
            result->message = "Planning failed (error: " + std::to_string(error.val) + ")";
            RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
            goal_handle->abort(result);
            return;
        }

        // Phase 2: Execute
        state_ = State::EXECUTING;
        feedback->phase = "EXECUTING";
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(get_logger(), "Executing plan...");
        {
            std::lock_guard lock(move_group_mutex_);
            move_group_->setStartStateToCurrentState();
            error = move_group_->execute(candidate);
        }

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }

        result->trajectory_points = static_cast<uint32_t>(candidate.trajectory.joint_trajectory.points.size());
        if (error == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Plan and execute succeeded";
            RCLCPP_INFO(get_logger(), "%s (%u points)", result->message.c_str(), result->trajectory_points);
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "Execution failed (error: " + std::to_string(error.val) + ")";
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetPlanner>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
