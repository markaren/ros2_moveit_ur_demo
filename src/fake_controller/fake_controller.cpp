#include <memory>
#include <string>
#include <chrono>
#include <algorithm>
#include <mutex>
#include <thread>
#include <atomic>
#include <unordered_map>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class FakeController : public rclcpp::Node
{
public:
    FakeController()
        : Node("fake_controller_node")
    {
        // --- Parameters ---
        jointNames_ = this->declare_parameter<std::vector<std::string>>(
            "joint_names", std::vector<std::string>{});

        controller_name_ = this->declare_parameter<std::string>(
            "controller_name", "fake_ur_manipulator_controller");

        publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
        execution_rate_hz_ = this->declare_parameter<double>("execution_rate_hz", 125.0);
        interpolation_method_ = this->declare_parameter<std::string>("interpolation", "cubic");

        if (jointNames_.empty())
        {
            RCLCPP_WARN(get_logger(), "No joint_names provided — the controller won't publish anything useful.");
        }

        jointPositions_.resize(jointNames_.size(), 0.0);
        jointVelocities_.resize(jointNames_.size(), 0.0);
        for (size_t i = 0; i < jointNames_.size(); ++i)
        {
            jointIndexByName_[jointNames_[i]] = i;
        }

        // --- Publisher ---
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
            [this]
            {
                if (jointNames_.empty()) return;
                sensor_msgs::msg::JointState js;
                js.header.stamp = this->now();
                js.name = jointNames_;
                {
                    std::lock_guard lock(joint_mutex_);
                    js.position = jointPositions_;
                    js.velocity = jointVelocities_;
                    js.effort.assign(jointNames_.size(), 0.0);
                }
                joint_pub_->publish(js);
            });

        // --- Action server ---
        const std::string action_name = "/" + controller_name_ + "/follow_joint_trajectory";
        action_server_ = rclcpp_action::create_server<FollowJT>(
            this,
            action_name,
            std::bind(&FakeController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FakeController::handle_cancel, this, std::placeholders::_1),
            std::bind(&FakeController::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "FakeController ready — action: %s, joints: %zu, publish: %.0f Hz, exec: %.0f Hz, interp: %s",
                    action_name.c_str(), jointNames_.size(), publish_rate_hz_, execution_rate_hz_,
                    interpolation_method_.c_str());
    }

    ~FakeController() override
    {
        shutdown_requested_.store(true);
        cancel_execution();
    }

private:
    // ---------- Joint state ----------
    std::vector<double> jointPositions_;
    std::vector<double> jointVelocities_;
    std::vector<std::string> jointNames_;
    std::unordered_map<std::string, size_t> jointIndexByName_;
    std::mutex joint_mutex_;

    // ---------- Parameters ----------
    std::string controller_name_;
    double publish_rate_hz_{50.0};
    double execution_rate_hz_{125.0};
    std::string interpolation_method_{"cubic"};

    // ---------- Execution ----------
    std::jthread execution_thread_;
    std::atomic_bool cancel_requested_{false};
    std::atomic_bool shutdown_requested_{false};

    // ---------- ROS handles ----------
    using FollowJT = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJT>;
    rclcpp_action::Server<FollowJT>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    // ================================================================
    //  Interpolation helpers
    // ================================================================

    /// Linear interpolation between two waypoints.
    static double lerp(double q0, double q1, double alpha)
    {
        return (1.0 - alpha) * q0 + alpha * q1;
    }

    /// Cubic Hermite interpolation using positions and velocities at endpoints.
    static double cubic_hermite(double q0, double v0, double q1, double v1,
                                double dt, double alpha)
    {
        if (dt <= 0.0) return q1;
        // Normalized time s ∈ [0,1], scale velocities by dt
        const double s = alpha;
        const double s2 = s * s;
        const double s3 = s2 * s;

        const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        const double h10 = s3 - 2.0 * s2 + s;
        const double h01 = -2.0 * s3 + 3.0 * s2;
        const double h11 = s3 - s2;

        return h00 * q0 + h10 * (v0 * dt) + h01 * q1 + h11 * (v1 * dt);
    }

    /// Compute velocity via finite‑difference (if velocities aren't supplied).
    static double fd_velocity(double q_prev, double q_next, double dt)
    {
        return (dt > 0.0) ? (q_next - q_prev) / dt : 0.0;
    }

    // ================================================================
    //  Action callbacks
    // ================================================================

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const FollowJT::Goal> goal)
    {
        const auto& traj = goal->trajectory;

        if (traj.joint_names.empty() || traj.points.empty())
        {
            RCLCPP_WARN(get_logger(), "Rejected goal: empty joint_names or points");
            return rclcpp_action::GoalResponse::REJECT;
        }

        for (const auto& name : traj.joint_names)
        {
            if (!jointIndexByName_.contains(name))
            {
                RCLCPP_ERROR(get_logger(), "Rejected goal: unknown joint '%s'", name.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        for (size_t i = 0; i < traj.points.size(); ++i)
        {
            const auto& pt = traj.points[i];
            if (pt.positions.size() != traj.joint_names.size())
            {
                RCLCPP_ERROR(get_logger(),
                             "Rejected goal: point %zu has %zu positions (expected %zu)",
                             i, pt.positions.size(), traj.joint_names.size());
                return rclcpp_action::GoalResponse::REJECT;
            }

            // Verify monotonically increasing time_from_start
            if (i > 0)
            {
                const double t_prev = rclcpp::Duration(traj.points[i - 1].time_from_start).seconds();
                const double t_cur = rclcpp::Duration(pt.time_from_start).seconds();
                if (t_cur < t_prev)
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Rejected goal: time_from_start is not monotonically increasing at point %zu", i);
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
        }

        RCLCPP_INFO(get_logger(), "Accepted goal with %zu joints, %zu waypoints",
                    traj.joint_names.size(), traj.points.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> /*goal_handle*/)
    {
        RCLCPP_INFO(get_logger(), "Cancel requested");
        cancel_execution();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // Preempt any running goal
        cancel_execution();
        if (execution_thread_.joinable())
        {
            execution_thread_.join();
        }
        cancel_requested_.store(false);
        execution_thread_ = std::jthread{&FakeController::execute_goal, this, goal_handle};
    }

    void cancel_execution()
    {
        cancel_requested_.store(true);
    }

    // ================================================================
    //  Goal execution (runs in dedicated thread)
    // ================================================================

    void execute_goal(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto& goal = goal_handle->get_goal();
        const auto& traj = goal->trajectory;
        const size_t n_joints = traj.joint_names.size();
        const size_t n_points = traj.points.size();

        // Build index mapping: incoming order → our internal order
        std::vector<size_t> idx_map(n_joints);
        for (size_t i = 0; i < n_joints; ++i)
        {
            idx_map[i] = jointIndexByName_.at(traj.joint_names[i]);
        }

        // Helper: apply positions + velocities to internal state
        auto apply_state = [&](const std::vector<double>& pos,
                               const std::vector<double>& vel)
        {
            std::lock_guard lock(joint_mutex_);
            for (size_t j = 0; j < n_joints; ++j)
            {
                jointPositions_[idx_map[j]] = pos[j];
                jointVelocities_[idx_map[j]] = vel[j];
            }
        };

        // Check whether velocities are available in all waypoints
        const bool have_velocities = std::ranges::all_of(
            traj.points, [&](const auto& pt)
            {
                return pt.velocities.size() == n_joints;
            });

        const bool use_cubic = (interpolation_method_ == "cubic") && have_velocities;

        // Pre-apply the first waypoint immediately
        {
            const auto& first = traj.points.front();
            std::vector<double> zero_vel(n_joints, 0.0);
            apply_state(first.positions, have_velocities ? first.velocities : zero_vel);
        }

        const auto start_time = now();
        size_t seg = 0;

        rclcpp::Rate rate(execution_rate_hz_);
        auto result = std::make_shared<FollowJT::Result>();
        auto feedback = std::make_shared<FollowJT::Feedback>();
        feedback->joint_names = traj.joint_names;

        std::vector<double> interp_pos(n_joints);
        std::vector<double> interp_vel(n_joints, 0.0);

        while (rclcpp::ok() && !shutdown_requested_.load())
        {
            if (cancel_requested_.load() || goal_handle->is_canceling())
            {
                // Zero out velocities on cancel
                {
                    std::lock_guard lock(joint_mutex_);
                    std::fill(jointVelocities_.begin(), jointVelocities_.end(), 0.0);
                }
                result->error_code = FollowJT::Result::SUCCESSFUL; // cancel is not an error
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Goal canceled");
                return;
            }

            const rclcpp::Duration elapsed = now() - start_time;
            const double t_sec = elapsed.seconds();

            // Advance segment index
            while (seg + 1 < n_points &&
                rclcpp::Duration(traj.points[seg + 1].time_from_start).seconds() <= t_sec)
            {
                ++seg;
            }

            // Check for completion
            const double t_final = rclcpp::Duration(traj.points.back().time_from_start).seconds();
            if (seg >= n_points - 1 && t_sec >= t_final)
            {
                std::vector<double> zero_vel(n_joints, 0.0);
                apply_state(traj.points.back().positions,
                            have_velocities ? traj.points.back().velocities : zero_vel);

                // Check goal tolerances if provided
                bool within_tolerance = true;
                if (!goal->goal_tolerance.empty())
                {
                    std::lock_guard lock(joint_mutex_);
                    for (const auto& tol : goal->goal_tolerance)
                    {
                        auto it = jointIndexByName_.find(tol.name);
                        if (it == jointIndexByName_.end()) continue;
                        // We're a fake controller, so we always hit the target exactly.
                        // But log the tolerances for debugging.
                        RCLCPP_DEBUG(get_logger(), "Goal tolerance for '%s': position=%.6f",
                                     tol.name.c_str(), tol.position);
                    }
                }

                result->error_code = within_tolerance
                                         ? FollowJT::Result::SUCCESSFUL
                                         : FollowJT::Result::GOAL_TOLERANCE_VIOLATED;
                goal_handle->succeed(result);

                const double total_s = (now() - start_time).seconds();
                RCLCPP_INFO(get_logger(), "Goal succeeded (%.3f s, %zu waypoints)", total_s, n_points);
                return;
            }

            // --- Interpolate between seg and seg+1 ---
            const auto& p0 = traj.points[seg];
            const auto& p1 = traj.points[std::min(seg + 1, n_points - 1)];

            const double t0 = rclcpp::Duration(p0.time_from_start).seconds();
            const double t1 = rclcpp::Duration(p1.time_from_start).seconds();
            const double dt = t1 - t0;
            const double alpha = (dt > 0.0) ? std::clamp((t_sec - t0) / dt, 0.0, 1.0) : 1.0;

            for (size_t j = 0; j < n_joints; ++j)
            {
                if (use_cubic)
                {
                    interp_pos[j] = cubic_hermite(
                        p0.positions[j], p0.velocities[j],
                        p1.positions[j], p1.velocities[j],
                        dt, alpha);

                    // Derivative of cubic Hermite for velocity feedback
                    if (dt > 0.0)
                    {
                        const double s = alpha;
                        const double ds_dt = 1.0 / dt;
                        const double dh00 = (6.0 * s * s - 6.0 * s) * ds_dt;
                        const double dh10 = (3.0 * s * s - 4.0 * s + 1.0) * ds_dt;
                        const double dh01 = (-6.0 * s * s + 6.0 * s) * ds_dt;
                        const double dh11 = (3.0 * s * s - 2.0 * s) * ds_dt;
                        interp_vel[j] = dh00 * p0.positions[j] + dh10 * (p0.velocities[j] * dt) +
                            dh01 * p1.positions[j] + dh11 * (p1.velocities[j] * dt);
                    }
                    else
                    {
                        interp_vel[j] = 0.0;
                    }
                }
                else
                {
                    interp_pos[j] = lerp(p0.positions[j], p1.positions[j], alpha);
                    interp_vel[j] = (dt > 0.0) ? (p1.positions[j] - p0.positions[j]) / dt : 0.0;
                }
            }

            apply_state(interp_pos, interp_vel);

            // --- Publish feedback ---
            feedback->desired.positions = p1.positions;
            if (have_velocities)
                feedback->desired.velocities = p1.velocities;
            feedback->desired.time_from_start = p1.time_from_start;

            feedback->actual.positions = interp_pos;
            feedback->actual.velocities = interp_vel;
            feedback->actual.time_from_start = elapsed;

            // Error = desired (interpolated target) - actual (same here, so ~0)
            feedback->error.positions.resize(n_joints, 0.0);
            feedback->error.velocities.resize(n_joints, 0.0);
            feedback->error.time_from_start = rclcpp::Duration(0, 0);

            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }

        // Node shutting down before trajectory finished
        result->error_code = FollowJT::Result::INVALID_GOAL;
        if (!goal_handle->is_canceling())
        {
            goal_handle->abort(result);
            RCLCPP_WARN(get_logger(), "Goal aborted — node shutting down");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
