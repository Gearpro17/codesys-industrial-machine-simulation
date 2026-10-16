#pragma once
#include "plant_io.h"
#include "recipes.h"
#include <cstdint>
#include <ostream>
#include <string_view>

namespace plant {

    // plcState values written to holding register HR 1 (keep in sync with docs/io_map.md).
    enum class State : std::int16_t {
        Stopped = 0, Starting = 1, Running = 2, Inspection = 3, Rejecting = 4,
        Transferring = 5, Stacking = 6, PalletFull = 7, PalletChange = 8,
        Fault = 9, EmergencyStop = 10
    };

    enum class FaultCode : std::int16_t {
        None = 0, EmergencyStop = 1, StartupCheck = 2, MotorFault = 3,
        BoxNotAtStation = 4, RejectFailed = 5, TransferTimeout = 6, MissingPallet = 7,
        StackerTimeout = 8, GripperTimeout = 9, UnexpectedBoxAtPickup = 10,
        PalletChangeTimeout = 11
    };

    std::string_view stateName(State s);
    std::string_view faultName(FaultCode f);
    inline std::ostream& operator<<(std::ostream& os, State s) { return os << stateName(s); }
    inline std::ostream& operator<<(std::ostream& os, FaultCode f) { return os << faultName(f); }

    // All the times are in seconds of *scan time* (sum of dt), exactly like TON timers in a PLC.
    struct ControllerConfig {
        int          recipeId = 1;
        std::int16_t runSpeedMms = 500;
        double settleSpeedMms = 20.0;   // conveyor counts as stopped below this
        int    heightSamples = 10;     // average this many scans for the height
        double startupCheckS = 0.5;
        double rejectExtendS = 0.8;
        double rejectRetractS = 0.4;
        double transferTimeoutS = 12.0;
        double stackerTimeoutS = 3.0;
        double gripperTimeoutS = 2.0;
        double releaseDwellS = 0.5;
        double palletFullHoldS = 0.5;
        double palletLeaveTimeoutS = 5.0;
        double palletArriveTimeoutS = 10.0;
        double motorFaultDelayS = 2.0;    // commanded but not moving for this long -> fault
        double faultAckHoldS = 1.0;    // a fault cannot be acknowledged sooner than this
    };

    // The C++ twin of the PLC program: same I/O interface, same state machine.
    // One call to update() is one PLC scan: read Inputs, run the logic, return Outputs.
    // It never touches the clock, so tests run it in simulated time.
    class SoftController {
    public:
        explicit SoftController(ControllerConfig cfg = {});

        Outputs update(const Inputs& in, double dt);

        State     state() const { return state_; }
        FaultCode fault() const { return fault_; }
        bool      setRecipe(int id);                        // false if the id is unknown
        const Recipe& recipe() const { return *recipe_; }

        int    goodCount()       const { return good_; }
        int    rejectCount()     const { return rejects_; }
        int    palletsCompleted() const { return pallets_; }
        double lastHeightMm()    const { return lastHeightMm_; }

    private:
        enum class StackStep { Lower, Grip, Lift, Release };

        void enter(State s);
        void raiseFault(FaultCode code);
        void nextStackStep(StackStep s) { stackStep_ = s; stepTimeS_ = 0.0; }
        void watchMotor(const Inputs& in, double dt);
        void runState(const Inputs& in, bool startEdge);
        void runStacking(const Inputs& in);
        void applyInterlocks(const Inputs& in);
        static Outputs safeOutputs(bool holdGripper);

        ControllerConfig cfg_;
        const Recipe* recipe_ = nullptr;

        State     state_ = State::Stopped;
        FaultCode fault_ = FaultCode::None;
        StackStep stackStep_ = StackStep::Lower;
        double    stateTimeS_ = 0.0;     // time in the current state
        double    stepTimeS_ = 0.0;     // time in the current sub-step
        bool      prevStart_ = false;
        Outputs   out_{};
        Outputs   prevOut_{};
        std::int16_t heartbeat_ = 0;

        // Inspection
        int    samples_ = 0;
        int    heightSumDmm_ = 0;
        double lastHeightMm_ = 0.0;
        // A good box has been sent to the pickup stop and is not yet stacked.
        bool   pickupValidated_ = false;
        bool   palletLeft_ = false;
        double motorOnTimeS_ = 0.0;

        int good_ = 0, rejects_ = 0, pallets_ = 0;
    };

} // namespace plant