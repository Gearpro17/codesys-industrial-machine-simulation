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
        : cfg_(cfg),
        motor_(cfg.motorTauS),
        rejectCyl_(cfg.rejectTravelS, 0.0),   // starts retracted
        stacker_(cfg.stackerTravelS, 1.0),    // starts up (home)
        gripper_(cfg.gripperTravelS, 0.0),    // starts open
        rng_(seed),
        heightNoise_(0.0, cfg.heightNoiseMm) {
    }

    void Plant::spawnBox(const BoxSpec& spec) {
        boxes_.push_back({ nextId_++, spec, 0.0 });
    }

    bool Plant::covers(const Box& b, double posMm) {
        return b.frontMm >= posMm && (b.frontMm - b.spec.lengthMm) <= posMm;
    }

    const Box* Plant::boxAt(double posMm) const {
        for (const auto& b : boxes_)
            if (covers(b, posMm)) return &b;
        return nullptr;
    }

    void Plant::step(double dt, const Outputs& out) {
        // --- Belt ---
        motor_.step(dt, out.conveyorMotor, out.speedSetpointMms);
        const double v = motor_.speedMms();
        for (auto& b : boxes_)
            b.frontMm = std::min(b.frontMm + v * dt, cfg_.pickupStopMm);

        // --- Actuators ---
        // Reject cylinder and gripper: single-acting, spring return.
        rejectCyl_.step(dt, out.rejectCylinder ? +1 : -1);
        gripper_.step(dt, out.gripperClose ? +1 : -1);

        // Stacker: separate up/down outputs. Both or neither -> holds position.
        int stackerDir = 0;
        if (out.stackerUp && !out.stackerDown)      stackerDir = +1;
        else if (out.stackerDown && !out.stackerUp) stackerDir = -1;
        stacker_.step(dt, stackerDir);

        // --- Reject: a box at the inspection station is pushed off the belt
        //     once the cylinder is nearly fully extended. ---
        if (rejectCyl_.position() >= 0.9) {
            rejected_ += static_cast<int>(std::erase_if(boxes_, [&](const Box& b) {
                return covers(b, cfg_.inspectionMm);
                }));
        }

        // --- Sensors ---
        in_.entrySensor = boxAt(cfg_.entrySensorMm) != nullptr;
        const Box* inspected = boxAt(cfg_.inspectionMm);
        in_.inspectionSensor = inspected != nullptr;

        in_.boxHeightDmm = 0;
        if (inspected) {
            const double measured = inspected->spec.heightMm + heightNoise_(rng_);
            in_.boxHeightDmm = static_cast<std::int16_t>(std::lround(measured * 10.0));
        }
        in_.conveyorSpeedFbMms = static_cast<std::int16_t>(std::lround(v));

        in_.stackerUpLimit = stacker_.atMax();
        in_.stackerDownLimit = stacker_.atMin();
        in_.gripperClosedFb = gripper_.atMax();
    }

} // namespace plant