#include <memory>
#include <string>
#include <chrono>

#include <control_msgs/control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action/server.hpp>
#include <rclcpp_action/rclcpp_action/server_goal_handle.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;


class FakeController : public rclcpp::Node
{
public:
    FakeController()
        : Node("fake_controller_node")
    {
        jointNames_ = this->declare_parameter<std::vector<std::string>>(
            "joint_names",
            std::vector<std::string>{}
        );
        controller_name_ = this->declare_parameter<std::string>(
            "controller_name", "fake_ur_manipulator_controller");


        jointValues_.resize(jointNames_.size());
        for (size_t i = 0; i < jointNames_.size(); ++i)
        {
            jointIndexByName_[jointNames_[i]] = i;
        }


        joint_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        publish_timer_ = this->create_wall_timer(20ms, [this]
        {
            if (jointNames_.empty()) return;
            sensor_msgs::msg::JointState js;
            js.header.stamp = this->now();
            js.name = jointNames_;
            {
                std::lock_guard lock(joint_mutex_);
                js.position = jointValues_;
            }
            joint_pub_->publish(js);
        });

        const std::string action_name = "/" + controller_name_ + "/follow_joint_trajectory";
        // Action server: /<controller_name>/follow_joint_trajectory
        action_server_ = rclcpp_action::create_server<FollowJT>(
            this,
            action_name,
            std::bind(&FakeController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FakeController::handle_cancel, this, std::placeholders::_1),
            std::bind(&FakeController::handle_accepted, this, std::placeholders::_1)
        );
    }

    ~FakeController()
    {
        cancel_requested_.store(true);
        if (execution_thread_.joinable())
        {
            execution_thread_.join();
        }
    }

private:
    std::vector<double> jointValues_;
    std::vector<std::string> jointNames_;
    std::unordered_map<std::string, size_t> jointIndexByName_;

    std::string controller_name_{"fake_ur_manipulator_controller"};

    std::mutex joint_mutex_;
    std::thread execution_thread_;
    std::atomic_bool cancel_requested_{false};


    using FollowJT = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJT>;
    rclcpp_action::Server<FollowJT>::SharedPtr action_server_;

    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const FollowJT::Goal> goal)
    {
        const auto& traj = goal->trajectory;

        if (traj.joint_names.empty() || traj.points.empty())
        {
            RCLCPP_WARN(get_logger(), "Rejected FollowJointTrajectory goal: empty joint_names/points");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (traj.joint_names.size() != jointNames_.size())
        {
            RCLCPP_ERROR(get_logger(), "Rejected goal: joint count mismatch (expected %zu, got %zu)",
                         jointNames_.size(), traj.joint_names.size());
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

        for (const auto& pt : traj.points)
        {
            if (pt.positions.size() != traj.joint_names.size())
            {
                RCLCPP_ERROR(get_logger(), "Rejected goal: a trajectory point has wrong positions size.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> /*goal_handle*/)
    {
        cancel_requested_.store(true);
        RCLCPP_INFO(get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        cancel_requested_.store(true);
        if (execution_thread_.joinable())
        {
            execution_thread_.join();
        }
        cancel_requested_.store(false);
        execution_thread_ = std::thread{&FakeController::execute_goal, this, goal_handle};
    }

    void execute_goal(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto& goal = goal_handle->get_goal();
        const auto& traj = goal->trajectory;

        // Map incoming joint order -> our joint order
        std::vector<size_t> incoming_to_mine(traj.joint_names.size());
        for (size_t i = 0; i < traj.joint_names.size(); ++i)
        {
            incoming_to_mine[i] = jointIndexByName_.at(traj.joint_names[i]);
        }

        auto apply_positions = [&](const std::vector<double>& positions_incoming)
        {
            std::lock_guard lock(joint_mutex_);
            for (size_t i = 0; i < positions_incoming.size(); ++i) {
                jointValues_[incoming_to_mine[i]] = positions_incoming[i];
            }
        };

        const auto start = now();
        size_t idx = 0;

        apply_positions(traj.points.front().positions);

        rclcpp::Rate rate(120.0);
        const auto result = std::make_shared<FollowJT::Result>();
        const auto feedback = std::make_shared<FollowJT::Feedback>();
        feedback->joint_names = traj.joint_names;

        std::vector<double> interp(traj.joint_names.size());
        while (rclcpp::ok())
        {
            if (cancel_requested_.load() || goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                return;
            }

            const auto t = now() - start;

            while (idx + 1 < traj.points.size() &&
                rclcpp::Duration(traj.points[idx + 1].time_from_start) <= t)
            {
                idx++;
            }

            // finished?
            if (idx >= traj.points.size() - 1 &&
                rclcpp::Duration(traj.points.back().time_from_start) <= t)
            {
                apply_positions(traj.points.back().positions);

                result->error_code = FollowJT::Result::SUCCESSFUL;
                goal_handle->succeed(result);
                return;
            }

            const auto& p0 = traj.points[idx];
            const auto& p1 = traj.points[idx + 1];

            const double t0 = rclcpp::Duration(p0.time_from_start).seconds();
            const double t1 = rclcpp::Duration(p1.time_from_start).seconds();
            const double tn = rclcpp::Duration(t).seconds();

            const double a = (t1 > t0)
                                 ? std::clamp((tn - t0) / (t1 - t0), 0.0, 1.0)
                                 : 1.0;


            for (size_t j = 0; j < interp.size(); ++j)
            {
                interp[j] = ((1.0 - a) * p0.positions[j] + a * p1.positions[j]);
            }
            apply_positions(interp);

            feedback->desired = p1;
            feedback->actual.positions.resize(interp.size());
            for (size_t j = 0; j < interp.size(); ++j) feedback->actual.positions[j] = interp[j];
            feedback->actual.time_from_start = t;
            goal_handle->publish_feedback(feedback);


            rate.sleep();
        }

        result->error_code = FollowJT::Result::INVALID_GOAL;
        goal_handle->abort(result);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeController>());
    rclcpp::shutdown();
    return 0;
}
