#ifndef EETRAIL_HPP
#define EETRAIL_HPP

#include <deque>

#include <threepp/core/BufferGeometry.hpp>
#include <threepp/materials/LineBasicMaterial.hpp>
#include <threepp/objects/Line.hpp>

using namespace threepp;

class EETrail
{
public:
    explicit EETrail(const Color& color = Color::darkviolet)
    {
        auto attr = FloatBufferAttribute::create(std::vector(kMaxPoints * 3, 0.f), 3);
        attr->setUsage(DrawUsage::Dynamic);

        geom_ = BufferGeometry::create();
        geom_->setAttribute("position", std::move(attr));

        auto mat = LineBasicMaterial::create();
        mat->color = color;

        line_ = Line::create(geom_, mat);
        line_->frustumCulled = false;
        line_->visible = false;
    }

    Object3D& object() { return *line_; }

    void update(float time, const Vector3& pos)
    {
        deque_.push_back({time, pos});

        while (time - deque_.front().time > kDuration)
            deque_.pop_front();

        const auto n = std::min(static_cast<int>(deque_.size()), kMaxPoints);
        auto& arr = geom_->getAttribute<float>("position")->array();
        for (int i = 0; i < n; ++i)
        {
            arr[i * 3]     = deque_[i].pos.x;
            arr[i * 3 + 1] = deque_[i].pos.y;
            arr[i * 3 + 2] = deque_[i].pos.z;
        }
        geom_->getAttribute("position")->needsUpdate();
        geom_->setDrawRange(0, n);
        line_->visible = n >= 2;
    }

    void clear()
    {
        line_->visible = false;
        deque_.clear();
    }

private:
    static constexpr int kMaxPoints = 500;
    static constexpr float kDuration = 3.f;

    struct TrailPoint { float time; Vector3 pos; };
    std::deque<TrailPoint> deque_;

    std::shared_ptr<BufferGeometry> geom_;
    std::shared_ptr<Line> line_;
};

#endif //EETRAIL_HPP
