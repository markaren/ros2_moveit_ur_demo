
#ifndef KINEUI_HPP
#define KINEUI_HPP

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <threepp/extras/imgui/ImguiContext.hpp>
#include <threepp/objects/Robot.hpp>

using namespace threepp;

class KineUI: public ImguiContext {
public:
    KineUI(
            const Canvas& canvas,
            std::shared_ptr<Robot> robot,
            std::mutex& robotMutex,
            const std::vector<std::string>& jointNames,
            bool goalPlanning)
        : ImguiContext(canvas), robot_(std::move(robot)), robotMutex_(robotMutex), jointNames_(jointNames), goalPlanning_(goalPlanning) {}

    [[nodiscard]] bool loopGhost() const { return loopGhost_; }
    [[nodiscard]] bool showTrail() const { return showTrail_; }


    // Returns modified joint values if any slider changed since last call.
    std::optional<std::vector<float>> consumeJointChange() {
        return std::exchange(pendingJointChange_, std::nullopt);
    }

    // True if "Plan goal" was clicked since last call.
    bool consumePlanRequest() {
        return std::exchange(pendingPlanRequest_, false);
    }

    // True if "Execute" was clicked since last call.
    bool consumeExecuteRequest() {
        return std::exchange(pendingExecuteRequest_, false);
    }

    // True if "Reset gizmo" was clicked since last call.
    bool consumeResetGizmoRequest() {
        return std::exchange(pendingResetGizmoRequest_, false);
    }

protected:
    void onRender() override {
        ImGui::SetNextWindowPos({});
        ImGui::SetNextWindowSize({});

        ImGui::Begin("Controls");

        ImGui::Checkbox("Show EE Trail", &showTrail_);

        if (ImGui::CollapsingHeader("Joints")) {
            std::unique_lock lock(robotMutex_);
            const auto infos = robot_->getArticulatedJointInfo();
            auto jointValues = robot_->jointValues();
            bool jointsChanged = false;
            for (unsigned i = 0; i < robot_->numDOF(); ++i) {
                const auto [min, max] = robot_->getJointRange(i, true);
                if (infos[i].type == Robot::JointType::Revolute) {
                    jointsChanged |= ImGui::SliderAngle(jointNames_[i].c_str(), &jointValues[i], min, max);
                } else {
                    jointsChanged |= ImGui::SliderFloat(jointNames_[i].c_str(), &jointValues[i], min, max);
                }
            }
            lock.unlock();

            if (jointsChanged) {
                pendingJointChange_ = std::move(jointValues);
            }
        }

        if (goalPlanning_) {
            ImGui::Checkbox("Loop ghost animation", &loopGhost_);

            if (ImGui::Button("Plan goal")) {
                pendingPlanRequest_ = true;
                planRequested_ = true;
            }

            ImGui::BeginDisabled(!planRequested_);
            if (ImGui::Button("Execute")) {
                pendingExecuteRequest_ = true;
                planRequested_ = false;
            }
            ImGui::EndDisabled();

            if (ImGui::Button("Reset gizmo")) {
                pendingResetGizmoRequest_ = true;
            }
        }

        ImGui::End();
    }

private:
    std::mutex& robotMutex_;
    std::shared_ptr<Robot> robot_;
    const std::vector<std::string>& jointNames_;
    bool goalPlanning_;

    bool showCollisionGeometry_ = false;
    bool showTrail_ = true;
    bool loopGhost_ = true;
    bool planRequested_ = false;

    std::optional<std::vector<float>> pendingJointChange_;
    bool pendingPlanRequest_ = false;
    bool pendingExecuteRequest_ = false;
    bool pendingResetGizmoRequest_ = false;
};

#endif//KINEUI_HPP
