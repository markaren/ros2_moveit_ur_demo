#include "KineEnvironment.hpp"

#include <threepp/cameras/PerspectiveCamera.hpp>
#include <threepp/controls/OrbitControls.hpp>
#include <threepp/core/Clock.hpp>
#include <threepp/helpers/GridHelper.hpp>
#include <threepp/input/IOCapture.hpp>
#include <threepp/lights/DirectionalLight.hpp>
#include <threepp/loaders/AssimpLoader.hpp>
#include <threepp/renderers/GLRenderer.hpp>
#include <threepp/renderers/GLRenderTarget.hpp>
#include <threepp/scenes/Scene.hpp>
#include <threepp/extras/imgui/ImguiContext.hpp>
#include <threepp/loaders/URDFLoader.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace threepp;
using namespace std::chrono_literals;


inline std::shared_ptr<Robot> loadRobot(const std::string &urdf,
                                        std::shared_ptr<Loader<Group> > geometryLoader = nullptr) {
    URDFLoader urdfLoader;
    urdfLoader.setGeometryLoader(geometryLoader);

    const auto share = ament_index_cpp::get_package_share_directory("ur_description");
    return urdfLoader.parse(share, urdf);
}

KineEnvironmentNode::KineEnvironmentNode() : Node("kine_environment_node") {
    declare_parameter<std::string>("robot_description", "");
    get_parameter("robot_description", urdf_);

    RCLCPP_INFO(this->get_logger(), "robot_description size: %zu", urdf_.size());

    // subscription: receive 3-element Float32MultiArray to set joint values
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
            // Current values as base; overwrite only joints present in msg

            if (robot_->numDOF() != jointNames_.size()) return;

            for (size_t i = 0; i < msg->position.size(); ++i) {
                robot_->setJointValue(i, static_cast<float>(msg->position[i]));
            }
        });


    robot_ = loadRobot(urdf_, std::make_shared<AssimpLoader>());
    RCLCPP_INFO(get_logger(), "Loaded URDF robot with %zu DOF", robot_->numDOF());
    robot_->rotation.x = -math::PI / 2; // adjust for threepp coordinate system

    const auto info = robot_->getArticulatedJointInfo();
    std::ranges::transform(info, std::back_inserter(jointNames_),
                           [](const auto &ji) { return ji.name; });


    thread_ = std::thread(&KineEnvironmentNode::run, this);
    sem_.acquire(); //wait until ready
}

void KineEnvironmentNode::run() {
    sem_.release(); //notify that we are ready

    Box3 bb;
    bb.setFromObject(*robot_);
    auto robotSize = bb.getSize();

    Canvas canvas_("Kine Environment");
    GLRenderer renderer_(canvas_.size());

    PerspectiveCamera camera_(60, canvas_.aspect(), 0.1, 1000);
    camera_.position.set(robotSize.x, robotSize.y * 1.5f, robotSize.z * 3.f);

    Scene scene_;
    scene_.background = Color::aliceblue;

    const auto grid = GridHelper::create(20, 10, Color::grey, Color::red);
    scene_.add(grid);

    const auto light = DirectionalLight::create(Color::white);
    light->position.set(1, 1, 1).normalize();
    scene_.add(light);

    constexpr int textureSize = 512;
    GLRenderTarget::Options opts;
    opts.format = Format::RGB;
    opts.anisotropy = 16;
    GLRenderTarget renderTarget(textureSize, textureSize, opts);

    OrbitControls controls(camera_, canvas_);


    std::vector<float> initialVals;
    std::ranges::transform(robot_->getJointRanges(),
                           std::back_inserter(initialVals),
                           [](const auto &range) { return range.mid(); });

    robot_->setJointValues(initialVals);
    scene_.add(robot_);

    // IOCapture capture{};
    // capture.preventMouseEvent = [] { return ImGui::GetIO().WantCaptureMouse; };
    // canvas_.setIOCapture(&capture);

    bool showCollisionGeometry = false;
    robot_->showColliders(showCollisionGeometry);


    Clock clock;
    canvas_.animate([&] {
        const auto dt = clock.getDelta();

        renderer_.render(scene_, camera_);

        if (!rclcpp::ok()) {
            canvas_.close();
        }
    });
}

KineEnvironmentNode::~KineEnvironmentNode() {
    if (thread_.joinable()) {
        thread_.join();
    }
}
