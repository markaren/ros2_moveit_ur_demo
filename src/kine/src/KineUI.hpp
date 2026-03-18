#ifndef KINEUI_HPP
#define KINEUI_HPP

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <threepp/extras/imgui/ImguiContext.hpp>
#include <threepp/math/Euler.hpp>
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
        : ImguiContext(canvas), robot_(std::move(robot)), robotMutex_(robotMutex), jointNames_(jointNames),
          goalPlanning_(goalPlanning) {
    }

    [[nodiscard]] bool loopGhost() const { return loopGhost_; }
    [[nodiscard]] bool showTrail() const { return showTrail_; }

    // Returns modified joint values if any slider changed since last call.
    std::optional<std::vector<float>> consumeJointChange() {
        return std::exchange(pendingJointChange_, std::nullopt);
    }

    // True if "Plan" was clicked since last call.
    bool consumePlanRequest() {
        return std::exchange(pendingPlanRequest_, false);
    }

    // True if "Execute" was clicked since last call.
    bool consumeExecuteRequest() {
        return std::exchange(pendingExecuteRequest_, false);
    }

    // True if "Plan & Execute" was clicked since last call.
    bool consumePlanAndExecuteRequest() {
        return std::exchange(pendingPlanAndExecuteRequest_, false);
    }

    // True if "Reset gizmo" was clicked since last call.
    bool consumeResetGizmoRequest() {
        return std::exchange(pendingResetGizmoRequest_, false);
    }

    void setBusy(bool busy) { busy_ = busy; }
    void setHasPlan(bool has_plan) { hasPlan_ = has_plan; }
    void setActionStatus(const std::string& status) { action_status_ = status; }

protected:
    void onRender() override {
        ImGui::SetNextWindowPos({});
        ImGui::SetNextWindowSize({});

        ImGui::Begin("Controls");

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

            ImGui::BeginDisabled(busy_);

            if (ImGui::Button("Plan")) {
                pendingPlanRequest_ = true;
            }

            ImGui::SameLine();

            ImGui::BeginDisabled(!hasPlan_);
            if (ImGui::Button("Execute")) {
                pendingExecuteRequest_ = true;
                hasPlan_ = false;
            }
            ImGui::EndDisabled();

            ImGui::SameLine();

            if (ImGui::Button("Plan & Execute")) {
                pendingPlanAndExecuteRequest_ = true;
            }

            ImGui::EndDisabled();

            if (!action_status_.empty()) {
                ImGui::TextUnformatted(action_status_.c_str());
            }

            if (ImGui::Button("Reset gizmo")) {
                pendingResetGizmoRequest_ = true;
            }
        }

        ImGui::Checkbox("Show EE Trail", &showTrail_);
        {
            std::unique_lock lock(robotMutex_);
            const auto transform = robot_->getEndEffectorTransform();
            lock.unlock();

            Vector3 pos;
            pos.setFromMatrixPosition(transform);
            Quaternion q;
            q.setFromRotationMatrix(transform);

            Euler euler;
            euler.setFromQuaternion(q);

            ImGui::Separator();
            ImGui::Text("EE Position (m):");
            ImGui::Text("  x=%.3f  y=%.3f  z=%.3f", pos.x, pos.y, pos.z);
            ImGui::Text("EE Orientation RPY (deg):");
            ImGui::Text("  r=%.1f  p=%.1f  y=%.1f",
                        euler.x * math::RAD2DEG,
                        euler.y * math::RAD2DEG,
                        euler.z * math::RAD2DEG);
        }

        ImGui::End();
    }

private:
    std::mutex& robotMutex_;
    std::shared_ptr<Robot> robot_;
    const std::vector<std::string>& jointNames_;
    bool goalPlanning_;

    bool showTrail_ = true;
    bool loopGhost_ = true;
    bool busy_ = false;
    bool hasPlan_ = false;

    std::string action_status_;

    std::optional<std::vector<float>> pendingJointChange_;
    bool pendingPlanRequest_ = false;
    bool pendingExecuteRequest_ = false;
    bool pendingPlanAndExecuteRequest_ = false;
    bool pendingResetGizmoRequest_ = false;
};

#endif//KINEUI_HPP
