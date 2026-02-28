#include "TrajectoryAnimator.hpp"

#include <algorithm>
#include <cmath>

TrajectoryAnimator::TrajectoryAnimator(std::shared_ptr<threepp::Robot> ghost)
    : ghost_(std::move(ghost))
{
}

void TrajectoryAnimator::loadTrajectory(const moveit_msgs::msg::DisplayTrajectory::SharedPtr& msg)
{
    std::lock_guard lock(mutex_);
    points_.clear();
    times_.clear();
    visible_ = false;
    elapsed_ = 0.0f;
    playing_ = false;

    if (msg->trajectory.empty()) return;

    const auto& joint_traj = msg->trajectory.back().joint_trajectory;
    if (joint_traj.points.empty()) return;

    for (const auto& point : joint_traj.points)
    {
        std::vector<float> joints;
        joints.reserve(point.positions.size());
        for (const auto& p : point.positions)
        {
            joints.push_back(static_cast<float>(p));
        }
        points_.push_back(std::move(joints));

        double t = point.time_from_start.sec
                 + point.time_from_start.nanosec * 1e-9;
        times_.push_back(static_cast<float>(t));
    }

    visible_ = true;
    playing_ = true;
    elapsed_ = 0.0f;
}

void TrajectoryAnimator::update(float dt, bool loop)
{
    std::lock_guard lock(mutex_);
    ghost_->visible = visible_;

    if (visible_ && playing_ && points_.size() >= 2 && !times_.empty())
    {
        elapsed_ += 2 * dt;
        const float duration = times_.back();

        if (elapsed_ >= duration)
        {
            if (loop)
            {
                elapsed_ = std::fmod(elapsed_, duration);
            }
            else
            {
                elapsed_ = duration;
                playing_ = false;
            }
        }

        interpolateAndApply();
    }
    else if (visible_ && !points_.empty() && !playing_)
    {
        applyJoints(points_.back());
    }
}

void TrajectoryAnimator::stop()
{
    std::lock_guard lock(mutex_);
    visible_ = false;
    playing_ = false;
}

bool TrajectoryAnimator::isVisible() const
{
    std::lock_guard lock(mutex_);
    return visible_;
}

bool TrajectoryAnimator::isPlaying() const
{
    std::lock_guard lock(mutex_);
    return playing_;
}

void TrajectoryAnimator::applyJoints(const std::vector<float>& joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
    {
        ghost_->setJointValue(i, joints[i]);
    }
}

void TrajectoryAnimator::interpolateAndApply()
{
    size_t nextIdx = 1;
    for (; nextIdx < times_.size(); ++nextIdx)
    {
        if (times_[nextIdx] >= elapsed_)
            break;
    }
    const size_t prevIdx = nextIdx - 1;

    const float t0 = times_[prevIdx];
    const float t1 = times_[nextIdx];
    float alpha = (t1 > t0)
        ? (elapsed_ - t0) / (t1 - t0)
        : 0.0f;
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    const auto& prev = points_[prevIdx];
    const auto& next = points_[nextIdx];
    const size_t numJoints = std::min(prev.size(), next.size());

    for (size_t i = 0; i < numJoints; ++i)
    {
        const float val = prev[i] + alpha * (next[i] - prev[i]);
        ghost_->setJointValue(i, val);
    }
}
