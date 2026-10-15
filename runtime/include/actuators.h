#pragma once
#include <algorithm>

namespace plant {

    // Generic end-limited actuator. Position 0.0 = min end, 1.0 = max end.
    class LinearActuator {
    public:
        explicit LinearActuator(double travelTimeS, double startPos = 0.0)
            : rate_(1.0 / travelTimeS), pos_(startPos) {
        }

        // direction: +1 toward max, -1 toward min, 0 hold.
        void step(double dt, int direction) {
            if (stuck_ || direction == 0) return;
            pos_ = std::clamp(pos_ + direction * rate_ * dt, 0.0, 1.0);
        }

        double position() const { return pos_; }
        bool atMin() const { return pos_ <= kEnd; }          // limit switch at min end
        bool atMax() const { return pos_ >= 1.0 - kEnd; }    // limit switch at max end
        void setStuck(bool stuck) { stuck_ = stuck; }        // fault injection hook

    private:
        static constexpr double kEnd = 0.01;
        double rate_;
        double pos_;
        bool   stuck_ = false;
    };

} // namespace plant