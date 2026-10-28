#pragma once
#include <limits>

// General-purpose PID controller. It knows nothing about conveyors, motors or the PLC:
// the process variable can be a speed, a temperature or a pressure, in any unit.
// It never touches the clock either: the caller supplies dt, so tests run in simulated time.
namespace control {

    struct PidConfig {
        // Parallel form:  u = kp*e + ki*integral(e dt) + kd*d(-pv)/dt,  with e = setpoint - pv.
        double kp = 1.0;   // output units per error unit
        double ki = 0.0;   // output units per (error unit * s)
        double kd = 0.0;   // output units per (pv unit / s)

        double outMin = -std::numeric_limits<double>::infinity();
        double outMax = std::numeric_limits<double>::infinity();

        // Time constant of the low-pass filter on the derivative term. 0 = no filtering.
        double derivFilterTauS = 0.0;

        // Conditional integration: the integral stops growing while the output is saturated
        // and the error would push it further into saturation. Switchable only so that the
        // tests (and the docs) can show what happens without it.
        bool antiWindup = true;

        // A dt above this is treated as a scheduling glitch and the call is ignored.
        double maxDtS = std::numeric_limits<double>::infinity();
    };

    class PIDController {
    public:
        // Throws std::invalid_argument for an inconsistent configuration
        // (non-finite gains, outMin > outMax, negative filter time, maxDtS <= 0).
        explicit PIDController(PidConfig cfg = {});

        // One control step. Returns the new output, always inside [outMin, outMax].
        //
        // Bad input is not allowed to corrupt the controller: if dt is not a finite positive
        // number (or exceeds maxDtS), or the setpoint/measurement is NaN or infinite, the call is
        // ignored: the previous output is returned, no state changes and rejectedLastCall() is true.
        double update(double setpoint, double measurement, double dt);

        // Forget everything: integral, derivative filter, previous measurement, output.
        // The next update() has no derivative contribution (no kick from the first sample).
        void reset();

        // Change the settings on the fly. The integral term is kept (it is stored in output
        // units, so a gain change does not make the output jump) and re-clamped to the new limits.
        void setConfig(PidConfig cfg);
        const PidConfig& config() const { return cfg_; }

        double output()  const { return output_; }
        double pTerm()   const { return p_; }
        double iTerm()   const { return i_; }     // integral term, already multiplied by ki
        double dTerm()   const { return d_; }
        bool   saturated()        const { return saturated_; }
        bool   rejectedLastCall() const { return rejected_; }

    private:
        static void validate(const PidConfig& c);
        double clampOut(double u) const;

        PidConfig cfg_;
        double p_ = 0.0, i_ = 0.0, d_ = 0.0;
        double output_ = 0.0;
        double prevMeasurement_ = 0.0;
        double derivFiltered_ = 0.0;     // filtered d(-pv)/dt, in pv units per second
        bool   havePrev_ = false;
        bool   saturated_ = false;
        bool   rejected_ = false;
    };

} // namespace control
