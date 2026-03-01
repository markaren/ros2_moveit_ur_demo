
#ifndef TRAJECTORYANIMATOR_HPP
#define TRAJECTORYANIMATOR_HPP


#include <mutex>
#include <vector>

#include <threepp/loaders/URDFLoader.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

class TrajectoryAnimator {
public:
    explicit TrajectoryAnimator(std::shared_ptr<threepp::Robot> ghost);

    void loadTrajectory(const moveit_msgs::msg::DisplayTrajectory::SharedPtr& msg);
    void update(float dt, bool loop);
    void stop();
    bool isVisible() const;
    bool isPlaying() const;

private:
    void applyJoints(const std::vector<float>& joints);
    void interpolateAndApply();

    std::shared_ptr<threepp::Robot> ghost_;

    mutable std::mutex mutex_;
    std::vector<std::vector<float>> points_;
    std::vector<float> times_;
    float elapsed_{0.0f};
    bool visible_{false};
    bool playing_{false};
    float loopDelay_{1.0f};
    float loopWait_{0.0f};
};

#endif //TRAJECTORYANIMATOR_HPP
