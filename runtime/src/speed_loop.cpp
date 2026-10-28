#include "speed_loop.h"
#include <algorithm>
#include <cmath>

namespace plant {

    SpeedLoop::SpeedLoop(SpeedLoopConfig cfg) : cfg_(cfg), pid_(cfg.pid) {}

    void SpeedLoop::reset(double currentSpeedMms) {
        pid_.reset();
        ramp_ = std::max(currentSpeedMms, 0.0);
        drive_ = 0.0;
    }

    double SpeedLoop::update(double dt, bool run, double setpointMms, double measuredMms) {
        if (!std::isfinite(dt) || dt <= 0.0) return drive_;

        const double target = (run && std::isfinite(setpointMms))
            ? std::clamp(setpointMms, 0.0, cfg_.maxSetpointMms) : 0.0;

        // Ramp: acceleration and deceleration are limited separately.
        if (target > ramp_) ramp_ = std::min(target, ramp_ + cfg_.accelLimitMms2 * dt);
        else                ramp_ = std::max(target, ramp_ - cfg_.decelLimitMms2 * dt);

        if (cfg_.openLoop) {
            drive_ = std::clamp(ramp_ / cfg_.nominalGainMmsPerPct, -100.0, 100.0);
            return drive_;
        }

        // Stopped and asked to stay stopped: release the drive and forget the integral, so a
        // braking command is not held against a belt that is already at rest.
        if (target == 0.0 && ramp_ == 0.0 && measuredMms <= cfg_.standstillMms) {
            pid_.reset();
            drive_ = 0.0;
            return drive_;
        }

        drive_ = pid_.update(ramp_, measuredMms, dt);
        return drive_;
    }

} // namespace plant
