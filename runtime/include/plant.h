#pragma once
#include "actuators.h"
#include "plant_io.h"
#include <random>
#include <vector>

namespace plant {

    struct BoxSpec {
        int    typeId;
        double heightMm;
        double lengthMm;
    };

    struct Box {
        int     id;
        BoxSpec spec;
        double  frontMm;   // leading edge position along the belt (belt starts at 0)
    };

    struct PlantConfig {
        double entrySensorMm = 300.0;
        double inspectionMm = 1200.0;
        double pickupStopMm = 2000.0;  // mechanical stop: boxes cannot go further
        double motorTauS = 0.4;     // motor/belt time constant
        double heightNoiseMm = 0.3;     // sensor noise, std deviation
        double rejectTravelS = 0.3;     // reject cylinder full stroke
        double stackerTravelS = 1.2;     // stacker full stroke
        double gripperTravelS = 0.5;     // gripper open <-> closed
    };

    // First-order lag: speed follows the target with time constant tau.
    // This is the process the PID will control in Step 4.
    class MotorModel {
    public:
        explicit MotorModel(double tauS) : tau_(tauS) {}
        void step(double dt, bool run, double setpointMms);
        double speedMms() const { return speed_; }
        void setStuck(bool stuck) { stuck_ = stuck; }   // fault injection hook
    private:
        double tau_;
        double speed_ = 0.0;
        bool   stuck_ = false;
    };

    class Plant {
    public:
        explicit Plant(PlantConfig cfg = {}, unsigned seed = 1);

        void spawnBox(const BoxSpec& spec);              // leading edge enters at x = 0
        void step(double dt, const Outputs& out);

        const Inputs& inputs() const { return in_; }
        const std::vector<Box>& boxes()  const { return boxes_; }
        int                     rejectedCount() const { return rejected_; }

        // Fault injection access
        MotorModel& motor() { return motor_; }
        LinearActuator& rejectCylinder() { return rejectCyl_; }
        LinearActuator& stacker() { return stacker_; }
        LinearActuator& gripper() { return gripper_; }

    private:
        static bool covers(const Box& b, double posMm);  // does the box body cover posMm?
        const Box* boxAt(double posMm) const;

        PlantConfig    cfg_;
        MotorModel     motor_;
        LinearActuator rejectCyl_;   // 0 = retracted, 1 = extended
        LinearActuator stacker_;     // 0 = down,      1 = up
        LinearActuator gripper_;     // 0 = open,      1 = closed
        std::mt19937   rng_;
        std::normal_distribution<double> heightNoise_;
        std::vector<Box> boxes_;
        Inputs in_{};
        int nextId_ = 1;
        int rejected_ = 0;
    };

} // namespace plant