#include "drive_motor.h"
#include <algorithm>
#include <cmath>

namespace plant {

    void DriveMotorModel::step(double dt, double drivePct, int boxesOnBelt) {
        // A NaN command must not poison the state: treat it as "no drive".
        drive_ = std::isfinite(drivePct) ? std::clamp(drivePct, -100.0, 100.0) : 0.0;
        if (stuck_) { speed_ = 0.0; return; }

        const double target = cfg_.gainMmsPerPct * drive_ - cfg_.loadPerBoxMms * std::max(boxesOnBelt, 0);
        // Exact discretisation of the first-order lag: stable for any dt.
        const double alpha = tau_ > 0.0 ? 1.0 - std::exp(-dt / tau_) : 1.0;
        speed_ += (target - speed_) * alpha;
        speed_ = std::max(speed_, 0.0);
    }

} // namespace plant
