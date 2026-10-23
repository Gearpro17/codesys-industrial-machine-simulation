#include "pid.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control {

    void PIDController::validate(const PidConfig& c) {
        if (!std::isfinite(c.kp) || !std::isfinite(c.ki) || !std::isfinite(c.kd))
            throw std::invalid_argument("PID gains must be finite");
        if (std::isnan(c.outMin) || std::isnan(c.outMax) || c.outMin > c.outMax)
            throw std::invalid_argument("PID output limits: outMin must not exceed outMax");
        if (!std::isfinite(c.derivFilterTauS) || c.derivFilterTauS < 0.0)
            throw std::invalid_argument("PID derivative filter time must be >= 0");
        if (std::isnan(c.maxDtS) || c.maxDtS <= 0.0)
            throw std::invalid_argument("PID maxDtS must be > 0");
    }

    PIDController::PIDController(PidConfig cfg) : cfg_(cfg) {
        validate(cfg_);
        output_ = clampOut(0.0);
    }

    double PIDController::clampOut(double u) const {
        return std::clamp(u, cfg_.outMin, cfg_.outMax);
    }

    void PIDController::reset() {
        p_ = i_ = d_ = 0.0;
        derivFiltered_ = 0.0;
        prevMeasurement_ = 0.0;
        havePrev_ = false;
        saturated_ = false;
        rejected_ = false;
        output_ = clampOut(0.0);
    }

    void PIDController::setConfig(PidConfig cfg) {
        validate(cfg);
        cfg_ = cfg;
        if (cfg_.antiWindup) i_ = clampOut(i_);
        output_ = clampOut(output_);
    }

    double PIDController::update(double setpoint, double measurement, double dt) {
        const bool dtOk = std::isfinite(dt) && dt > 0.0 && dt <= cfg_.maxDtS;
        if (!dtOk || !std::isfinite(setpoint) || !std::isfinite(measurement)) {
            rejected_ = true;
            return output_;
        }
        rejected_ = false;

        const double error = setpoint - measurement;
        p_ = cfg_.kp * error;

        // Derivative on the measurement, not on the error: a setpoint step changes the error
        // instantly but not the measurement, so it cannot produce a derivative kick.
        // The raw derivative of a noisy signal is mostly noise, hence the first-order low-pass.
        if (havePrev_) {
            const double raw = -(measurement - prevMeasurement_) / dt;
            const double alpha = dt / (cfg_.derivFilterTauS + dt);   // 1 when there is no filter
            derivFiltered_ += alpha * (raw - derivFiltered_);
        }
        else {
            derivFiltered_ = 0.0;
        }
        prevMeasurement_ = measurement;
        havePrev_ = true;
        d_ = cfg_.kd * derivFiltered_;

        // Integral with conditional integration (anti-windup).
        const double increment = cfg_.ki * error * dt;
        double iNew = i_ + increment;
        if (cfg_.antiWindup) {
            const double uTry = p_ + iNew + d_;
            const bool pushesFurtherHigh = uTry > cfg_.outMax && increment > 0.0;
            const bool pushesFurtherLow = uTry < cfg_.outMin && increment < 0.0;
            if (pushesFurtherHigh || pushesFurtherLow) iNew = i_;   // saturated: hold the integral
            iNew = clampOut(iNew);                                  // the I term alone never exceeds the range
        }
        i_ = iNew;

        const double u = p_ + i_ + d_;
        output_ = clampOut(u);
        saturated_ = (output_ != u);
        return output_;
    }

} // namespace control
