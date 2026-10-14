#include "plant.h"
#include <algorithm>
#include <cmath>

namespace plant {

    void MotorModel::step(double dt, bool run, double setpointMms) {
        if (stuck_) { speed_ = 0.0; return; }
        const double target = run ? setpointMms : 0.0;
        const double alpha = std::min(dt / tau_, 1.0);
        speed_ += (target - speed_) * alpha;
    }

    Plant::Plant(PlantConfig cfg, unsigned seed)
        : cfg_(cfg), motor_(cfg.motorTauS), rng_(seed), heightNoise_(0.0, cfg.heightNoiseMm) {
    }

    void Plant::spawnBox(const BoxSpec& spec) {
        boxes_.push_back({ nextId_++, spec, 0.0 });
    }

    const Box* Plant::boxAt(double posMm) const {
        for (const auto& b : boxes_) {
            const double rear = b.frontMm - b.spec.lengthMm;
            if (b.frontMm >= posMm && rear <= posMm) return &b;
        }
        return nullptr;
    }

    void Plant::step(double dt, const Outputs& out) {
        motor_.step(dt, out.conveyorMotor, out.speedSetpointMms);
        const double v = motor_.speedMms();

        for (auto& b : boxes_)
            b.frontMm = std::min(b.frontMm + v * dt, cfg_.pickupStopMm);

        in_.entrySensor = boxAt(cfg_.entrySensorMm) != nullptr;
        const Box* inspected = boxAt(cfg_.inspectionMm);
        in_.inspectionSensor = inspected != nullptr;

        in_.boxHeightDmm = 0;
        if (inspected) {
            const double measured = inspected->spec.heightMm + heightNoise_(rng_);
            in_.boxHeightDmm = static_cast<std::int16_t>(std::lround(measured * 10.0));
        }
        in_.conveyorSpeedFbMms = static_cast<std::int16_t>(std::lround(v));
    }

} // namespace plant