#pragma once

namespace plant {

    struct DriveMotorConfig {
        double gainMmsPerPct = 10.0;     // steady-state belt speed per % of drive command
        double loadPerBoxMms = 25.0;     // speed lost per box on the belt, at the same drive
    };

    // Physical motor + belt: the input is a drive command in % (-100..+100), the output is belt
    // speed in mm/s. This is the process the speed PID controls (the ideal MotorModel in plant.h
    // has a built-in controller, so there is nothing for a PID to do).
    //
    //   target = gain * drive - loadPerBox * boxesOnBelt
    //   speed follows target with a first-order lag (time constant tau)
    //
    // Negative drive brakes. The belt only runs forward, so the speed never goes below zero
    // (a braking command at standstill does nothing).
    class DriveMotorModel {
    public:
        explicit DriveMotorModel(double tauS, DriveMotorConfig cfg = {}) : tau_(tauS), cfg_(cfg) {}

        void step(double dt, double drivePct, int boxesOnBelt);

        double speedMms() const { return speed_; }
        double lastDrivePct() const { return drive_; }   // command actually applied (after clamping)
        const DriveMotorConfig& config() const { return cfg_; }
        void setStuck(bool stuck) { stuck_ = stuck; }    // fault injection: belt does not move

    private:
        double tau_;
        DriveMotorConfig cfg_;
        double speed_ = 0.0;
        double drive_ = 0.0;
        bool   stuck_ = false;
    };

} // namespace plant
