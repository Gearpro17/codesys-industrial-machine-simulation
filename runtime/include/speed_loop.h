#pragma once
#include "pid.h"
#include "plant_io.h"

namespace plant {

    struct SpeedLoopConfig {
        double accelLimitMms2 = 800.0;     // setpoint ramp, speeding up
        double decelLimitMms2 = 1000.0;    // setpoint ramp, slowing down
        double maxSetpointMms = 1000.0;
        double standstillMms = 5.0;        // below this, with a zero setpoint, the drive is released
        bool   openLoop = false;           // no feedback: drive = ramped setpoint / nominal gain
        double nominalGainMmsPerPct = 10.0;
        control::PidConfig pid = defaultPid();

        static control::PidConfig defaultPid() {
            control::PidConfig c;
            c.kp = 0.8;                    // % of drive per mm/s of speed error
            c.ki = 2.0;                    // Ti = kp/ki = 0.4 s = motor time constant
            c.kd = 0.0;
            c.outMin = -100.0;             // drive command limits
            c.outMax = 100.0;
            c.derivFilterTauS = 0.05;
            c.maxDtS = 0.5;
            return c;
        }
    };

    // Sits between the PLC and the plant. The PLC still says "run at 500 mm/s"; the loop ramps
    // that setpoint and the PID turns the speed error into a drive command (-100..+100 %).
    class SpeedLoop {
    public:
        explicit SpeedLoop(SpeedLoopConfig cfg = {});

        // Returns the drive command in %. A bad dt (<= 0, NaN, inf) leaves everything unchanged.
        double update(double dt, bool run, double setpointMms, double measuredMms);
        double update(double dt, const Outputs& plc, const Inputs& feedback) {
            return update(dt, plc.conveyorMotor, plc.speedSetpointMms, feedback.conveyorSpeedFbMms);
        }

        // Forget the integral and restart the ramp from the given speed.
        void reset(double currentSpeedMms = 0.0);

        double rampedSetpointMms() const { return ramp_; }
        double driveCommandPct() const { return drive_; }
        const control::PIDController& pid() const { return pid_; }
        const SpeedLoopConfig& config() const { return cfg_; }

    private:
        SpeedLoopConfig cfg_;
        control::PIDController pid_;
        double ramp_ = 0.0;
        double drive_ = 0.0;
    };

} // namespace plant
