#pragma once
#include "actuators.h"
#include "drive_motor.h"
#include "plant_io.h"
#include <optional>
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

    // Automatic infeed: spawns boxes at the belt start, never closer than minGapMm.
    struct FeederConfig {
        bool   enabled = false;     // off by default: tests spawn boxes by hand
        double intervalS = 3.0;       // minimum time between boxes
        double minGapMm = 150.0;     // required free belt behind the previous box
        std::vector<BoxSpec> catalog = { { 1, 120.0, 300.0 },{ 2, 80.0, 300.0 },{ 3, 160.0, 300.0 } };
        std::vector<double>  weights = { 0.70, 0.15, 0.15 };   // one per catalog entry
    };

    enum class MotorMode {
        Ideal,      // built-in controller: speed follows Outputs::speedSetpointMms by itself
        Physical    // drive command in % (setDriveCommand) -> belt speed; needs a SpeedLoop
    };

    struct PlantConfig {
        MotorMode motorMode = MotorMode::Ideal;
        DriveMotorConfig driveMotor;     // used in Physical mode (the lag is motorTauS)
        double entrySensorMm = 300.0;
        double inspectionMm = 1200.0;
        double pickupStopMm = 2000.0;  // mechanical stop: boxes cannot go further
        double motorTauS = 0.4;     // motor/belt time constant
        double heightNoiseMm = 0.3;     // sensor noise, std deviation
        double rejectTravelS = 0.3;     // reject cylinder full stroke
        double stackerTravelS = 1.2;     // stacker full stroke
        double gripperTravelS = 0.5;     // gripper open <-> closed
        int    palletCapacity = 4;       // boxes per pallet
        double palletChangeS = 3.0;     // time for a new empty pallet to arrive
        bool   startWithPallet = true;
        FeederConfig feeder;
    };

    // What the operator is doing at the panel. The plant turns it into the NC/NO
    // contact levels the PLC actually reads (see plant_io.h).
    struct OperatorPanel {
        bool start = false;   // START push-button held
        bool stopPressed = false;   // STOP push-button held
        bool estopPressed = false;   // emergency stop mushroom pressed
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

        // Puts a box at the belt start. If the entry is occupied it queues behind.
        void spawnBox(const BoxSpec& spec);
        void step(double dt, const Outputs& out);
        void setPanel(const OperatorPanel& p) { panel_ = p; }

        // Physical motor mode only: drive command in % (-100..+100), held until changed.
        // Outputs::conveyorMotor / speedSetpointMms are ignored in this mode (the SpeedLoop uses them).
        void setDriveCommand(double pct) { driveCmdPct_ = pct; }
        MotorMode motorMode() const { return cfg_.motorMode; }
        int boxesOnBelt() const { return static_cast<int>(boxes_.size()); }
        double beltSpeedMms() const;

        const Inputs& inputs() const { return in_; }
        const std::vector<Box>& boxes()  const { return boxes_; }

        // Production counters (what the SCADA will show)
        int  spawnedCount()     const { return nextId_ - 1; }
        int  rejectedCount()    const { return rejected_; }
        int  placedTotal()      const { return placedTotal_; }
        int  placedOnPallet()   const { return palletCount_; }
        int  palletsCompleted() const { return palletsCompleted_; }
        int  droppedCount()     const { return dropped_; }
        bool holdingBox()       const { return held_.has_value(); }

        // Fault injection access
        MotorModel& motor() { return motor_; }
        DriveMotorModel& driveMotor() { return driveMotor_; }
        void setMotorStuck(bool stuck) { motor_.setStuck(stuck); driveMotor_.setStuck(stuck); }
        LinearActuator& rejectCylinder() { return rejectCyl_; }
        LinearActuator& stacker() { return stacker_; }
        LinearActuator& gripper() { return gripper_; }
        void blockPalletArrival(bool b) { palletArrivalBlocked_ = b; }   // "missing pallet"

    private:
        static bool covers(const Box& b, double posMm);  // does the box body cover posMm?
        const Box* boxAt(double posMm) const;
        void runFeeder(double dt);
        void updatePickAndPlace();
        void updatePallet(double dt, const Outputs& out);

        PlantConfig    cfg_;
        MotorModel     motor_;
        DriveMotorModel driveMotor_;
        double         driveCmdPct_ = 0.0;
        LinearActuator rejectCyl_;  // 0 = retracted, 1 = extended
        LinearActuator stacker_;     // 0 = down,      1 = up
        LinearActuator gripper_;     // 0 = open,      1 = closed
        std::mt19937   rng_;
        std::normal_distribution<double> heightNoise_;
        std::mt19937   feederRng_;
        std::discrete_distribution<int> feederDist_;
        bool           palletPresent_;

        std::vector<Box>   boxes_;          // on the belt, oldest (front-most) first
        std::optional<Box> held_;           // box currently in the gripper
        Inputs in_{};
        OperatorPanel panel_{};
        double feederTimerS_ = 0.0;
        double palletTimerS_ = 0.0;
        bool   palletFull_ = false;
        bool   palletArrivalBlocked_ = false;
        int nextId_ = 1;
        int rejected_ = 0;
        int dropped_ = 0;
        int palletCount_ = 0;
        int placedTotal_ = 0;
        int palletsCompleted_ = 0;
    };

} // namespace plant