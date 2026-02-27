#ifndef KINEENVIRONMENT_HPP
#define KINEENVIRONMENT_HPP

#include <chrono>
#include <memory>
#include <semaphore>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <threepp/objects/Robot.hpp>

using namespace threepp;

class KineEnvironmentNode : public rclcpp::Node {
public:
    KineEnvironmentNode();

    void publishImage(int textureSize, const uint8_t *pixels) const;

    void run();

    ~KineEnvironmentNode() override;

private:
    std::string urdf_;
    std::shared_ptr<Robot> robot_;
    std::vector<std::string> jointNames_;

    std::vector<float> getRobotJointValuesThreadSafe() const;
    void setRobotJointValuesThreadSafe(const std::vector<float> &values);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread thread_;
    std::binary_semaphore sem_{0};
};

#endif // KINEENVIRONMENT_HPP
