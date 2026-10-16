#include "plant.h"
#include <algorithm>
#include <cmath>

namespace plant {

    namespace {
        constexpr double kPickupSensorOffsetMm = 20.0;   // pickup sensor sits just before the stop
        constexpr double kReleasePosition = 0.5;    // gripper position below which the box is released
        constexpr double kRejectedPosition = 0.9;    // cylinder position at which a box is pushed off
    }

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
        heightNoise_(0.0, cfg.heightNoiseMm),
        feederRng_(seed + 1000u),
        feederDist_(cfg.feeder.weights.begin(), cfg.feeder.weights.end()),
        palletPresent_(cfg.startWithPallet) {
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

    void Plant::runFeeder(double dt) {
        const FeederConfig& f = cfg_.feeder;
        if (!f.enabled || f.catalog.empty()) return;
        feederTimerS_ += dt;
        if (feederTimerS_ < f.intervalS) return;

        // Only feed when the belt start is free: the newest box has moved on by minGapMm.
        const bool clear = boxes_.empty() ||
            (boxes_.back().frontMm - boxes_.back().spec.lengthMm) >= f.minGapMm;
        if (!clear) return;

        const std::size_t idx = static_cast<std::size_t>(feederDist_(feederRng_)) % f.catalog.size();
        spawnBox(f.catalog[idx]);
        feederTimerS_ = 0.0;
    }

    void Plant::updatePickAndPlace() {
        // Grab: gripper fully closed, stacker fully down, a box waiting at the stop.
        if (!held_ && stacker_.atMin() && gripper_.atMax() && !boxes_.empty() &&
            boxes_.front().frontMm >= cfg_.pickupStopMm - 0.5) {
            held_ = boxes_.front();
            boxes_.erase(boxes_.begin());
        }

        // Release: once the gripper has opened. Only a release with the stacker at
        // the top, over a free pallet position, places the box. Anything else
        // drops it (box lost) and is counted as a fault condition.
        if (held_ && gripper_.position() < kReleasePosition) {
            if (stacker_.atMax() && palletPresent_ && !palletFull_) {
                ++palletCount_;
                ++placedTotal_;
                if (palletCount_ >= cfg_.palletCapacity) palletFull_ = true;
            }
            else {
                ++dropped_;
            }
            held_.reset();
        }
    }

    void Plant::updatePallet(double dt, const Outputs& out) {
        // A full pallet leaves only when the PLC asks for it.
        if (palletPresent_ && palletFull_ && out.palletRelease) {
            palletPresent_ = false;
            palletFull_ = false;
            palletCount_ = 0;
            ++palletsCompleted_;
            palletTimerS_ = 0.0;
        }
        // A new empty pallet arrives after palletChangeS (unless blocked: fault injection).
        if (!palletPresent_ && !palletArrivalBlocked_) {
            palletTimerS_ += dt;
            if (palletTimerS_ >= cfg_.palletChangeS) palletPresent_ = true;
        }
    }

    void Plant::step(double dt, const Outputs& out) {
        // --- Infeed ---
        runFeeder(dt);

        // --- Belt: each box queues behind the one in front of it ---
        motor_.step(dt, out.conveyorMotor, out.speedSetpointMms);
        const double v = motor_.speedMms();
        double limit = cfg_.pickupStopMm;
        for (auto& b : boxes_) {
            b.frontMm = std::min(b.frontMm + v * dt, limit);
            limit = b.frontMm - b.spec.lengthMm;
        }

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
        if (rejectCyl_.position() >= kRejectedPosition) {
            rejected_ += static_cast<int>(std::erase_if(boxes_, [&](const Box& b) {
                return covers(b, cfg_.inspectionMm);
                }));
        }

        // --- Pick and place, pallet handling ---
        updatePickAndPlace();
        updatePallet(dt, out);

        // --- Sensors ---
        in_.entrySensor = boxAt(cfg_.entrySensorMm) != nullptr;
        const Box* inspected = boxAt(cfg_.inspectionMm);
        in_.inspectionSensor = inspected != nullptr;
        in_.pickupSensor = boxAt(cfg_.pickupStopMm - kPickupSensorOffsetMm) != nullptr;

        in_.boxHeightDmm = 0;
        if (inspected) {
            const double measured = inspected->spec.heightMm + heightNoise_(rng_);
            in_.boxHeightDmm = static_cast<std::int16_t>(std::lround(measured * 10.0));
        }
        in_.conveyorSpeedFbMms = static_cast<std::int16_t>(std::lround(v));

        in_.stackerUpLimit = stacker_.atMax();
        in_.stackerDownLimit = stacker_.atMin();
        in_.gripperClosedFb = gripper_.atMax();
        in_.startButton = panel_.start;
        in_.stopButton = !panel_.stopPressed;     // NC contact
        in_.emergencyStop = !panel_.estopPressed;    // NC contact
        in_.palletPresent = palletPresent_;
        in_.palletFull = palletPresent_ && palletFull_;
    }

} // namespace plant