#include "KineEnvironment.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "EETrail.hpp"
#include "KineUI.hpp"
#include "TransformKeyListener.hpp"
#include <threepp/cameras/PerspectiveCamera.hpp>
#include <threepp/controls/OrbitControls.hpp>
#include <threepp/controls/TransformControls.hpp>
#include <threepp/core/Clock.hpp>
#include <threepp/helpers/GridHelper.hpp>
#include <threepp/input/IOCapture.hpp>
#include <threepp/lights/AmbientLight.hpp>
#include <threepp/lights/DirectionalLight.hpp>
#include <threepp/loaders/AssimpLoader.hpp>
#include <threepp/loaders/URDFLoader.hpp>
#include <threepp/materials/MeshBasicMaterial.hpp>
#include <threepp/renderers/GLRenderTarget.hpp>
#include <threepp/renderers/GLRenderer.hpp>
#include <threepp/scenes/Scene.hpp>

using namespace threepp;
using namespace std::chrono_literals;


namespace {
    std::shared_ptr<Robot> loadRobot(const std::string& urdf) {
        URDFLoader urdfLoader;
        urdfLoader.setGeometryLoader(std::make_shared<AssimpLoader>());

        const auto share = ament_index_cpp::get_package_share_directory("ur_description");
        return urdfLoader.parse(share, urdf);
    }

    geometry_msgs::msg::Pose toRosPose(const Vector3& pos, const Quaternion& q) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = pos.x;
        pose.position.y = pos.y;
        pose.position.z = pos.z;
        pose.orientation.x = q.x;
        pose.orientation.y = q.y;
        pose.orientation.z = q.z;
        pose.orientation.w = q.w;
        return pose;
    }

    auto createGhostMaterial(const Color& color) {
        auto ghostMaterial = MeshBasicMaterial::create();
        ghostMaterial->transparent = true;
        ghostMaterial->opacity = 0.5f;
        ghostMaterial->color = color;
        ghostMaterial->depthWrite = false;

        return ghostMaterial;
    }
}// namespace

KineEnvironmentNode::KineEnvironmentNode(): Node("kine_environment_node") {
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
    if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(get_logger(), "robot_state_publisher parameter service not available!");
        throw std::runtime_error("robot_state_publisher parameter service not available");
    }

    urdf_ = parameters_client->get_parameters({"robot_description"})[0].as_string();

    if (urdf_.empty()) {
        RCLCPP_FATAL(get_logger(), "robot_description from robot_state_publisher is empty!");
        throw std::runtime_error("robot_description parameter is empty");
    }

    robot_ = loadRobot(urdf_);
    if (!robot_) {
        RCLCPP_FATAL(get_logger(), "Failed to load URDF robot from parameter: %s", urdf_.c_str());
        throw std::runtime_error("Failed to load URDF robot");
    }

    RCLCPP_INFO(get_logger(), "Loaded URDF robot with %zu DOF", robot_->numDOF());

    const auto info = robot_->getArticulatedJointInfo();
    std::ranges::transform(info, std::back_inserter(jointNames_),
                           [](const auto& ji) { return ji.name; });


    goal_planning_ = declare_parameter<bool>("goal_planning", false);

    if (goal_planning_) {
        ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");

        auto plannerGhostMaterial = createGhostMaterial(Color::orange);
        planner_ghost_ = loadRobot(urdf_);
        planner_ghost_->showColliders(false);
        planner_ghost_->traverseType<Mesh>([&](Mesh& m) {
            m.setMaterial(plannerGhostMaterial);
        });

        auto ikGhostMaterial = createGhostMaterial(Color::blue);
        ik_ghost_ = loadRobot(urdf_);
        ik_ghost_->showColliders(false);
        ik_ghost_->traverseType<Mesh>([&](Mesh& m) {
            m.setMaterial(ikGhostMaterial);
        });

        animator_ = std::make_unique<TrajectoryAnimator>(planner_ghost_);

        trajectory_sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
                "display_planned_path", 10,
                [this](moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) {
                    animator_->loadTrajectory(msg);

                    RCLCPP_INFO(get_logger(), "Ghost trajectory received");
                });

        execute_pub_ = this->create_publisher<std_msgs::msg::Empty>(
                "execute_plan", 10);
    }

    joint_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>(goal_planning_ ? "joint_commands" : "joint_states", 10);

    // subscription: receive 3-element Float32MultiArray to set joint values
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard lock(robot_mutex_);
                if (robot_->numDOF() != jointNames_.size()) return;

                for (size_t i = 0; i < msg->position.size(); ++i) {
                    const auto it = std::ranges::find(jointNames_, msg->name[i]);
                    if (it != jointNames_.end()) {
                        const auto idx = std::distance(jointNames_.begin(), it);
                        robot_->setJointValue(idx, static_cast<float>(msg->position[i]));
                    }
                }
            });

    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10);

    thread_ = std::jthread(&KineEnvironmentNode::run, this);
}

void KineEnvironmentNode::run() {
    Canvas canvas("Kine Environment");
    GLRenderer renderer(canvas.size());

    Object3D::defaultUp = {0, 0, 1}; // ROS convention is Z-up

    Scene scene;
    scene.background = Color::aliceblue;

    PerspectiveCamera camera(60, canvas.aspect(), 0.1, 1000);

    scene.add(AmbientLight::create(0xffffff, 0.4f));

    const auto light = DirectionalLight::create(Color::white, 0.8f);
    light->position.set(1, -1, 1).normalize();
    scene.add(light);

    scene.add(robot_);

    Box3 bb;
    bb.setFromObject(*robot_);
    auto robotSize = bb.getSize();

    const auto maxSizeComponent = static_cast<unsigned int>(std::round(
            std::max({robotSize.x, robotSize.y, robotSize.z})));
    const auto grid = GridHelper::create(maxSizeComponent * 2, 10, Color::grey, Color::red);
    grid->rotation.x = math::PI / 2;
    scene.add(grid);

    // Use Z-up camera to match ROS coordinate convention
    camera.position.set(robotSize.x, -robotSize.z * 3.f, robotSize.z * 1.5f);

    OrbitControls orbitControls(camera, canvas);

    std::shared_ptr<Object3D> targetObject;
    std::unique_ptr<TransformControls> transformControls;
    std::unique_ptr<TransformKeyListener> keyListener;

    LambdaEventListener draggingChanged([&](const Event& event) {
        const bool dragging = std::any_cast<bool>(event.target);
        orbitControls.enabled = !dragging;
        ik_ghost_->visible = dragging;
    });

    LambdaEventListener objectChanged([&](const Event&) {
        requestIK(toRosPose(targetObject->position, targetObject->quaternion));
    });

    if (goal_planning_) {
        scene.add(planner_ghost_);

        scene.add(ik_ghost_);
        ik_ghost_->visible = false;

        targetObject = planner_ghost_->getObjectByName(jointNames_.back())->clone();
        auto targetMaterial = createGhostMaterial(Color::blue);
        targetObject->traverseType<Mesh>([mat = std::move(targetMaterial)](Mesh& m) {
            m.setMaterial(mat);
        });
        scene.add(targetObject);

        auto transform = robot_->getEndEffectorTransform();
        targetObject->position.setFromMatrixPosition(transform);
        targetObject->quaternion.setFromRotationMatrix(transform);

        transformControls = std::make_unique<TransformControls>(camera, canvas);
        transformControls->attach(*targetObject);
        scene.add(*transformControls);

        transformControls->addEventListener("dragging-changed", draggingChanged);
        transformControls->addEventListener("objectChange", objectChanged);

        keyListener = std::make_unique<TransformKeyListener>(transformControls.get());
        canvas.addKeyListener(*keyListener);
    }

    robot_->showColliders(false);
    KineUI ui(canvas, robot_, robot_mutex_, jointNames_, goal_planning_);

    IOCapture capture{};
    capture.preventMouseEvent = [] { return ImGui::GetIO().WantCaptureMouse; };
    canvas.setIOCapture(&capture);

    canvas.onWindowResize([&](const WindowSize& newSize) {
        renderer.setSize(newSize);
        camera.aspect = canvas.aspect();
        camera.updateProjectionMatrix();
    });

    EETrail trail;
    scene.add(trail.object());

    Clock clock;
    canvas.animate([&] {
        const auto dt = clock.getDelta();

        if (animator_) animator_->update(dt, ui.loopGhost());

        if (const auto jointValues = ui.consumeJointChange()) {
            sensor_msgs::msg::JointState js;
            js.header.stamp = this->now();
            js.name = jointNames_;
            js.position.resize(jointValues->size());
            std::ranges::copy(*jointValues, js.position.begin());
            joint_pub_->publish(js);
        }

        if (goal_planning_) {
            if (ui.consumePlanRequest()) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "base_link";
                pose_msg.pose = toRosPose(targetObject->position, targetObject->quaternion);
                target_pose_pub_->publish(pose_msg);
            }

            if (ui.consumeExecuteRequest()) {
                execute_pub_->publish(std_msgs::msg::Empty{});
                animator_->stop();
                RCLCPP_INFO(get_logger(), "Execute requested");
            }

            if (ui.consumeResetGizmoRequest()) {
                const auto transform = robot_->getEndEffectorTransform();
                targetObject->position.setFromMatrixPosition(transform);
                targetObject->quaternion.setFromRotationMatrix(transform);
                ik_ghost_->setJointValues(robot_->jointValues());
            }
        }

        if (ui.showTrail()) {
            Vector3 eePos;
            eePos.setFromMatrixPosition(robot_->getEndEffectorTransform());
            trail.update(clock.elapsedTime, eePos);
        } else if (trail.object().visible) {
            trail.clear();
        }

        renderer.render(scene, camera);
        ui.render();

        if (!rclcpp::ok()) {
            canvas.close();
        }
    });

    rclcpp::shutdown();
}

void KineEnvironmentNode::requestIK(const geometry_msgs::msg::Pose& target_pose) {
    if (!ik_client_->wait_for_service(0s)) return;

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = "ur_manipulator";
    request->ik_request.pose_stamped.header.frame_id = "base_link";
    request->ik_request.pose_stamped.header.stamp = this->now();
    request->ik_request.pose_stamped.pose = target_pose;
    request->ik_request.avoid_collisions = false;

    // Provide current joint state as seed for IK solver
    auto& robot_state = request->ik_request.robot_state.joint_state;
    robot_state.name = jointNames_;
    robot_state.position.resize(ik_ghost_->numDOF());
    for (size_t i = 0; i < ik_ghost_->numDOF(); ++i) {
        robot_state.position[i] = static_cast<double>(ik_ghost_->getJointValue(i));
    }

    ik_client_->async_send_request(
            request,
            [this](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
                const auto& response = future.get();
                if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    const auto& positions = response->solution.joint_state.position;
                    for (size_t i = 0; i < positions.size() && i < ik_ghost_->numDOF(); ++i) {
                        ik_ghost_->setJointValue(i, static_cast<float>(positions[i]));
                    }
                }
            });
}

KineEnvironmentNode::~KineEnvironmentNode() = default;
