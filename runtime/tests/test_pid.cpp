// PIDController on its own, against a simulated first-order plant:  T dy/dt = K u - y.
// No conveyor, no PLC: the controller must work for any process variable.
#include "pid.h"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

using control::PIDController;
using control::PidConfig;

namespace {
    constexpr double kDt = 0.010;
    constexpr double kInf = std::numeric_limits<double>::infinity();

    struct FirstOrderPlant {
        double gain = 2.0;      // K
        double tauS = 1.0;      // T
        double y = 0.0;
        void step(double u, double dt = kDt) { y += (gain * u - y) * (1.0 - std::exp(-dt / tauS)); }
    };

    struct Result {
        double peak = 0.0;             // highest y
        double finalY = 0.0;
        double maxAbsU = 0.0;
        double maxI = 0.0;             // largest |integral term| seen
        double settleS = -1.0;         // last time |y - setpoint| exceeded 2 %, or -1 if never settled
    };

    Result simulate(PIDController& pid, FirstOrderPlant& plant, double setpoint, double seconds) {
        Result r;
        const int n = static_cast<int>(std::lround(seconds / kDt));
        double lastOutside = 0.0;
        for (int i = 0; i < n; ++i) {
            const double u = pid.update(setpoint, plant.y, kDt);
            plant.step(u);
            r.peak = std::max(r.peak, plant.y);
            r.maxAbsU = std::max(r.maxAbsU, std::abs(u));
            r.maxI = std::max(r.maxI, std::abs(pid.iTerm()));
            if (std::abs(plant.y - setpoint) > 0.02 * std::abs(setpoint)) lastOutside = (i + 1) * kDt;
        }
        r.finalY = plant.y;
        r.settleS = lastOutside < seconds ? lastOutside : -1.0;
        return r;
    }

    PidConfig pi(double kp, double ki) { PidConfig c; c.kp = kp; c.ki = ki; return c; }
} // namespace

// ---------------------------------------------------------------- step response

TEST(Pid, ProportionalOnlyLeavesTheExpectedSteadyStateError) {
    PIDController pid(pi(2.0, 0.0));
    FirstOrderPlant plant;                                   // K = 2
    const Result r = simulate(pid, plant, 1.0, 10.0);
    EXPECT_NEAR(r.finalY, 4.0 / 5.0, 1e-3);                  // K*Kp / (1 + K*Kp) = 0.8
}

TEST(Pid, IntegralActionRemovesTheSteadyStateError) {
    PIDController pid(pi(2.0, 2.0));
    FirstOrderPlant plant;
    const Result r = simulate(pid, plant, 1.0, 15.0);
    EXPECT_NEAR(r.finalY, 1.0, 1e-3);
}

TEST(Pid, PiStepResponseIsWellDamped) {
    // Ti = T (pole cancellation): first-order closed loop, no overshoot.
    PIDController pid(pi(2.0, 2.0));
    FirstOrderPlant plant;
    const Result r = simulate(pid, plant, 1.0, 15.0);
    EXPECT_LT(r.peak, 1.02);
    EXPECT_GT(r.settleS, 0.0);
    EXPECT_LT(r.settleS, 3.0);
}

TEST(Pid, RejectsALoadDisturbance) {
    PIDController pid(pi(2.0, 2.0));
    FirstOrderPlant plant;
    simulate(pid, plant, 1.0, 10.0);
    ASSERT_NEAR(plant.y, 1.0, 1e-3);
    // Disturbance: the plant suddenly loses 0.5 units of output.
    double worst = 1.0;
    for (int i = 0; i < 1500; ++i) {
        const double u = pid.update(1.0, plant.y, kDt);
        plant.step(u - 0.25);                                // -0.25 input = -0.5 at the output
        worst = std::min(worst, plant.y);
    }
    EXPECT_LT(worst, 1.0);
    EXPECT_GT(worst, 0.5);                                   // the dip is bounded
    EXPECT_NEAR(plant.y, 1.0, 1e-3);                         // and fully recovered
}

TEST(Pid, NegativeErrorDrivesTheOutputDown) {
    PIDController pid(pi(1.0, 0.0));
    EXPECT_DOUBLE_EQ(pid.update(0.0, 5.0, kDt), -5.0);
}

// ---------------------------------------------------------------- output limits, saturation, windup

TEST(Pid, OutputNeverLeavesTheLimits) {
    PidConfig c = pi(50.0, 100.0);                           // far too aggressive on purpose
    c.kd = 5.0;
    c.outMin = -1.0;
    c.outMax = 1.0;
    PIDController pid(c);
    FirstOrderPlant plant;
    std::mt19937 rng(3);
    std::normal_distribution<double> noise(0.0, 0.5);
    for (int i = 0; i < 5000; ++i) {
        const double sp = (i / 500) % 2 ? -3.0 : 3.0;
        const double u = pid.update(sp, plant.y + noise(rng), kDt);
        ASSERT_LE(u, 1.0);
        ASSERT_GE(u, -1.0);
        plant.step(u);
    }
}

TEST(Pid, SaturatedOutputIsReported) {
    PidConfig c = pi(10.0, 0.0);
    c.outMax = 1.0;
    PIDController pid(c);
    pid.update(5.0, 0.0, kDt);
    EXPECT_TRUE(pid.saturated());
    pid.update(0.05, 0.0, kDt);
    EXPECT_FALSE(pid.saturated());
}

TEST(Pid, IntegralStopsGrowingWhileSaturated) {
    PidConfig c = pi(1.0, 5.0);
    c.outMax = 1.0;
    c.outMin = -1.0;
    PIDController pid(c);
    FirstOrderPlant plant;
    plant.gain = 0.5;                                        // cannot reach 10 with |u| <= 1
    simulate(pid, plant, 10.0, 0.5);
    const double iAtSaturation = pid.iTerm();
    simulate(pid, plant, 10.0, 20.0);                        // 20 s of hopeless error
    EXPECT_NEAR(pid.iTerm(), iAtSaturation, 1.0);            // did not keep integrating
    EXPECT_LE(std::abs(pid.iTerm()), 1.0);
}

TEST(Pid, AntiWindupGivesQuickRecoveryFromLongSaturation) {
    // Setpoint 3 needs u = 1.5, but |u| is limited to 1: the output saturates for a long time.
    // Then the setpoint drops to 0.5. With a wound-up integral the output stays saturated high
    // for a long time and the plant barely moves; with anti-windup it comes straight back.
    const auto errorThreeSecondsAfterTheDrop = [](bool antiWindup) {
        PidConfig c = pi(1.0, 4.0);
        c.outMin = -1.0;
        c.outMax = 1.0;
        c.antiWindup = antiWindup;
        PIDController pid(c);
        FirstOrderPlant plant;
        for (int i = 0; i < 1000; ++i) plant.step(pid.update(3.0, plant.y, kDt));   // 10 s saturated
        for (int i = 0; i < 300; ++i) plant.step(pid.update(0.5, plant.y, kDt));
        return std::abs(plant.y - 0.5);
    };
    EXPECT_LT(errorThreeSecondsAfterTheDrop(true), 0.05);
    EXPECT_GT(errorThreeSecondsAfterTheDrop(false), 0.5);
}

TEST(Pid, IntegralStillUnwindsWhenTheErrorReverses) {
    PidConfig c = pi(0.1, 2.0);                                    // P alone cannot saturate: the integral does
    c.outMin = -1.0;
    c.outMax = 1.0;
    PIDController pid(c);
    for (int i = 0; i < 300; ++i) pid.update(5.0, 0.0, kDt);       // saturated high
    const double high = pid.iTerm();
    ASSERT_GT(high, 0.4);
    for (int i = 0; i < 300; ++i) pid.update(-5.0, 0.0, kDt);      // error flips: must integrate down
    EXPECT_LT(pid.iTerm(), high);
}

// ---------------------------------------------------------------- derivative

TEST(Pid, SetpointStepDoesNotKickTheDerivative) {
    PidConfig c = pi(0.0, 0.0);
    c.kd = 100.0;
    PIDController pid(c);
    for (int i = 0; i < 10; ++i) pid.update(0.0, 1.0, kDt);        // measurement constant at 1
    const double u = pid.update(50.0, 1.0, kDt);                    // huge setpoint step
    EXPECT_DOUBLE_EQ(u, 0.0);                                       // derivative acts on the measurement only
    EXPECT_DOUBLE_EQ(pid.dTerm(), 0.0);
}

TEST(Pid, DerivativeOpposesMeasurementMotion) {
    PidConfig c = pi(0.0, 0.0);
    c.kd = 2.0;
    PIDController pid(c);
    pid.update(0.0, 0.0, kDt);
    const double u = pid.update(0.0, 0.1, kDt);                     // pv rising at 10 units/s
    EXPECT_NEAR(u, -20.0, 1e-9);
}

TEST(Pid, FirstSampleHasNoDerivativeContribution) {
    PidConfig c = pi(0.0, 0.0);
    c.kd = 100.0;
    PIDController pid(c);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 1000.0, kDt), 0.0);            // no previous measurement: no spike
}

TEST(Pid, DerivativeFilterAttenuatesMeasurementNoise) {
    const auto noiseOutputStdDev = [](double filterTauS) {
        PidConfig c = pi(0.0, 0.0);
        c.kd = 1.0;
        c.derivFilterTauS = filterTauS;
        PIDController pid(c);
        std::mt19937 rng(11);
        std::normal_distribution<double> noise(0.0, 0.05);
        double sum = 0.0, sumSq = 0.0;
        const int n = 5000;
        for (int i = 0; i < n; ++i) {
            const double u = pid.update(0.0, noise(rng), kDt);
            sum += u; sumSq += u * u;
        }
        const double mean = sum / n;
        return std::sqrt(sumSq / n - mean * mean);
    };
    const double raw = noiseOutputStdDev(0.0);
    const double filtered = noiseOutputStdDev(0.1);
    EXPECT_GT(raw, 5.0);                                             // unfiltered: noise / dt is huge
    EXPECT_LT(filtered, raw / 5.0);
}

TEST(Pid, DerivativeFilterHasTheConfiguredTimeConstant) {
    PidConfig c = pi(0.0, 0.0);
    c.kd = 1.0;
    c.derivFilterTauS = 0.2;
    PIDController pid(c);
    // Measurement ramps at 10 units/s: the filtered derivative should climb to -10 with tau = 0.2 s.
    double pv = 0.0;
    pid.update(0.0, pv, kDt);
    double u = 0.0;
    for (int i = 0; i < 20; ++i) { pv += 10.0 * kDt; u = pid.update(0.0, pv, kDt); }   // 0.2 s = one tau
    EXPECT_NEAR(u, -10.0 * (1.0 - std::exp(-1.0)), 0.5);
    for (int i = 0; i < 200; ++i) { pv += 10.0 * kDt; u = pid.update(0.0, pv, kDt); }
    EXPECT_NEAR(u, -10.0, 0.05);
}

// ---------------------------------------------------------------- reset

TEST(Pid, ResetClearsTheIntegralAndTheOutput) {
    PIDController pid(pi(1.0, 2.0));
    for (int i = 0; i < 200; ++i) pid.update(1.0, 0.0, kDt);
    ASSERT_GT(pid.iTerm(), 0.1);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.iTerm(), 0.0);
    EXPECT_DOUBLE_EQ(pid.output(), 0.0);
    EXPECT_FALSE(pid.saturated());
}

TEST(Pid, AfterResetTheFirstStepIsPureProportional) {
    PidConfig c = pi(2.0, 3.0);
    c.kd = 50.0;
    PIDController pid(c);
    for (int i = 0; i < 100; ++i) pid.update(1.0, 0.2 * i, kDt);
    pid.reset();
    const double u = pid.update(1.0, 0.5, kDt);                      // measurement jumped: still no D kick
    EXPECT_NEAR(u, 2.0 * 0.5 + 3.0 * 0.5 * kDt, 1e-12);
}

TEST(Pid, ResetOutputRespectsLimitsThatExcludeZero) {
    PidConfig c = pi(1.0, 0.0);
    c.outMin = 10.0;
    c.outMax = 20.0;
    PIDController pid(c);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.output(), 10.0);
}

// ---------------------------------------------------------------- bad input

TEST(Pid, BadDtIsIgnoredAndHoldsTheLastOutput) {
    PIDController pid(pi(1.0, 5.0));
    const double good = pid.update(1.0, 0.0, kDt);
    const double integral = pid.iTerm();
    for (double bad : { 0.0, -0.01, kInf, -kInf, std::numeric_limits<double>::quiet_NaN() }) {
        EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, bad), good) << "dt = " << bad;
        EXPECT_TRUE(pid.rejectedLastCall());
        EXPECT_DOUBLE_EQ(pid.iTerm(), integral);                     // state untouched
    }
    pid.update(1.0, 0.0, kDt);
    EXPECT_FALSE(pid.rejectedLastCall());                            // recovers on the next good call
    EXPECT_GT(pid.iTerm(), integral);
}

TEST(Pid, DtAboveMaxDtIsIgnored) {
    PidConfig c = pi(1.0, 5.0);
    c.maxDtS = 0.1;
    PIDController pid(c);
    const double u = pid.update(1.0, 0.0, kDt);
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, 5.0), u);                  // 5 s hiccup: no huge integral jump
    EXPECT_TRUE(pid.rejectedLastCall());
}

TEST(Pid, NonFiniteSetpointOrMeasurementIsIgnored) {
    PidConfig c = pi(1.0, 1.0);
    c.kd = 1.0;
    PIDController pid(c);
    const double u = pid.update(1.0, 0.0, kDt);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(pid.update(nan, 0.0, kDt), u);
    EXPECT_DOUBLE_EQ(pid.update(1.0, nan, kDt), u);
    EXPECT_DOUBLE_EQ(pid.update(1.0, kInf, kDt), u);
    EXPECT_TRUE(std::isfinite(pid.update(1.0, 0.1, kDt)));           // not poisoned
}

TEST(Pid, InconsistentConfigurationIsRejected) {
    PidConfig c;
    c.outMin = 1.0; c.outMax = -1.0;
    EXPECT_THROW(PIDController{ c }, std::invalid_argument);
    c = {}; c.kp = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(PIDController{ c }, std::invalid_argument);
    c = {}; c.derivFilterTauS = -0.1;
    EXPECT_THROW(PIDController{ c }, std::invalid_argument);
    c = {}; c.maxDtS = 0.0;
    EXPECT_THROW(PIDController{ c }, std::invalid_argument);
}

TEST(Pid, ChangingGainsOnTheFlyKeepsTheIntegralTerm) {
    PIDController pid(pi(1.0, 2.0));
    for (int i = 0; i < 100; ++i) pid.update(1.0, 0.0, kDt);
    const double before = pid.iTerm();
    PidConfig c = pid.config();
    c.ki = 10.0;
    pid.setConfig(c);
    EXPECT_DOUBLE_EQ(pid.iTerm(), before);                           // no bump from a gain change
}
