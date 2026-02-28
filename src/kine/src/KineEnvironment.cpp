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
#include <threepp/controls/TransformControls.hpp>
#include <threepp/materials/MeshBasicMaterial.hpp>

using namespace threepp;
using namespace std::chrono_literals;


inline std::shared_ptr<Robot> loadRobot(const std::string& urdf)
{
    URDFLoader urdfLoader;
    urdfLoader.setGeometryLoader(std::make_shared<AssimpLoader>());

    const auto share = ament_index_cpp::get_package_share_directory("ur_description");
    return urdfLoader.parse(share, urdf);
}

KineEnvironmentNode::KineEnvironmentNode() : Node("kine_environment_node")
{
    urdf_ = declare_parameter<std::string>("robot_description", "");

    if (urdf_.empty())
    {
        RCLCPP_FATAL(get_logger(), "robot_description parameter is empty!");
        throw std::runtime_error("robot_description parameter is empty");
    }

    robot_ = loadRobot(urdf_);
    if (!robot_)
    {
        RCLCPP_FATAL(get_logger(), "Failed to load URDF robot from parameter: %s", urdf_.c_str());
        throw std::runtime_error("Failed to load URDF robot");
    }

    auto ghostMaterial = MeshBasicMaterial::create();
    ghostMaterial->transparent = true;
    ghostMaterial->opacity = 0.5f;
    ghostMaterial->color = Color::orange;
    ghostMaterial->depthWrite = false;
    ghost_ = loadRobot(urdf_);
    ghost_->showColliders(false);
    ghost_->traverseType<Mesh>([&](Mesh& m)
    {
        m.setMaterial(ghostMaterial);
    });

    animator_ = std::make_unique<TrajectoryAnimator>(ghost_);

    RCLCPP_INFO(get_logger(), "Loaded URDF robot with %zu DOF", robot_->numDOF());

    const auto info = robot_->getArticulatedJointInfo();
    std::ranges::transform(info, std::back_inserter(jointNames_),
                           [](const auto& ji) { return ji.name; });

    // subscription: receive 3-element Float32MultiArray to set joint values
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
            if (robot_->numDOF() != jointNames_.size()) return;

            for (size_t i = 0; i < msg->position.size(); ++i)
            {
                robot_->setJointValue(i, static_cast<float>(msg->position[i]));
            }
        });

    trajectory_sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
           "display_planned_path", 10,
           [this](moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
           {
               animator_->loadTrajectory(msg);

               RCLCPP_INFO(get_logger(), "Ghost trajectory received");
           });

    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose", 10);


    execute_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "execute_plan", 10);

    thread_ = std::thread(&KineEnvironmentNode::run, this);
}

void KineEnvironmentNode::run()
{
    Canvas canvas("Kine Environment");
    GLRenderer renderer(canvas.size());

    Scene scene;
    scene.background = Color::aliceblue;

    const auto light = DirectionalLight::create(Color::white);
    light->position.set(1, 1, 1).normalize();
    scene.add(light);

    PerspectiveCamera camera(60, canvas.aspect(), 0.1, 1000);

    robot_->rotation.x = -math::PI / 2;
    ghost_->rotation.x = -math::PI / 2;

    scene.add(robot_);
    scene.add(ghost_);

    Box3 bb;
    bb.setFromObject(*robot_);
    auto robotSize = bb.getSize();

    const auto maxSizeComponent = static_cast<unsigned int>(std::round(
        std::max({robotSize.x, robotSize.y, robotSize.z})));
    const auto grid = GridHelper::create(maxSizeComponent * 2, 10, Color::grey, Color::red);
    scene.add(grid);

    camera.position.set(robotSize.x, robotSize.y * 1.5f, robotSize.z * 3.f);

    OrbitControls orbitControls(camera, canvas);

    auto target = ghost_->getObjectByName(jointNames_.back())->clone();
    // target->clone();
    scene.add(target);

    auto transform = robot_->getEndEffectorTransform();
    target->position.setFromMatrixPosition(transform);
    target->quaternion.setFromRotationMatrix(transform);
    TransformControls transformControls(camera, canvas);
    transformControls.attach(*target);
    scene.add(transformControls);

    LambdaEventListener changeListener([&](const Event& event)
    {
        orbitControls.enabled = !std::any_cast<bool>(event.target);
    });

    transformControls.addEventListener("dragging-changed", changeListener);

    KeyAdapter adapter(KeyAdapter::Mode::KEY_PRESSED, [&](const KeyEvent& evt)
    {
        switch (evt.key)
        {
        case Key::Q:
            {
                transformControls.setSpace(transformControls.getSpace() == "local" ? "world" : "local");
                break;
            }
        case Key::W:
            {
                transformControls.setMode("translate");
                break;
            }
        case Key::E:
            {
                transformControls.setMode("rotate");
                break;
            }
        default:
            break;
        }
    });
    canvas.addKeyListener(adapter);

    bool loopGhost = true;
    bool planRequested = false;
    bool showCollisionGeometry = false;
    robot_->showColliders(showCollisionGeometry);
    ImguiFunctionalContext ui(canvas, [&]
    {
        ImGui::SetNextWindowPos({});
        ImGui::SetNextWindowSize({});

        ImGui::Begin("Controls");
        if (ImGui::Checkbox("Show Collision Geometry", &showCollisionGeometry))
        {
            robot_->showColliders(showCollisionGeometry);
        }

        ImGui::Checkbox("Loop ghost animation", &loopGhost);

        if (ImGui::Button("Plan goal"))
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "base_link";

            // The robot is rotated -PI/2 on X, so transform target position back to ROS frame
            const auto& pos = target->position;
            pose_msg.pose.position.x = pos.x;
            pose_msg.pose.position.y = -pos.z; // threepp Z -> ROS -Y
            pose_msg.pose.position.z = pos.y; // threepp Y -> ROS Z

            // Undo the -PI/2 X rotation applied to the robot in the scene
            Quaternion correction;
            correction.setFromAxisAngle({1, 0, 0}, math::PI / 2);

            const auto& q = target->quaternion;

            Quaternion rosQuat;
            rosQuat.multiplyQuaternions(correction, q);

            pose_msg.pose.orientation.x = rosQuat.x;
            pose_msg.pose.orientation.y = rosQuat.y;
            pose_msg.pose.orientation.z = rosQuat.z;
            pose_msg.pose.orientation.w = rosQuat.w;

            target_pose_pub_->publish(pose_msg);
            planRequested = true;
        }

        ImGui::BeginDisabled(!planRequested);
        if (ImGui::Button("Execute"))
        {
            std_msgs::msg::Empty empty_msg;
            execute_pub_->publish(empty_msg);
            planRequested = false;
            animator_->stop();
            RCLCPP_INFO(get_logger(), "Execute requested");
        }
        ImGui::EndDisabled();

        ImGui::End();
    });

    IOCapture capture{};
    capture.preventMouseEvent = [] { return ImGui::GetIO().WantCaptureMouse; };
    canvas.setIOCapture(&capture);

    canvas.onWindowResize([&](const WindowSize& newSize)
    {
        renderer.setSize(newSize);
        camera.aspect = canvas.aspect();
        camera.updateProjectionMatrix();
    });

    Clock clock;
    canvas.animate([&]
    {
        const auto dt = clock.getDelta();

        animator_->update(dt, loopGhost);
        renderer.render(scene, camera);

        ui.render();

        if (!rclcpp::ok())
        {
            canvas.close();
        }
    });

    rclcpp::shutdown();
}


KineEnvironmentNode::~KineEnvironmentNode()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}
