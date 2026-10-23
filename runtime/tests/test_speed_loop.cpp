// SpeedLoop (ramp + PID) against the physical motor model (DriveMotorModel).
#include "drive_motor.h"
#include "plant.h"
#include "speed_loop.h"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

using namespace plant;

namespace {
    constexpr double kDt = 0.010;
    constexpr double kTau = 0.4;

    // Loop + motor, one scan at a time. The loop sees the true speed (no sensor quantisation).
    struct Rig {
        DriveMotorModel motor{ kTau };
        SpeedLoop loop;
        int boxes = 0;
        bool run = false;
        double setpoint = 0.0;
        double time = 0.0;
        double distanceMm = 0.0;

        explicit Rig(SpeedLoopConfig cfg = {}) : loop(cfg) {}

        void scan() {
            const double drive = loop.update(kDt, run, setpoint, motor.speedMms());
            motor.step(kDt, drive, boxes);
            distanceMm += motor.speedMms() * kDt;
            time += kDt;
        }
        void runFor(double seconds) {
            const int n = static_cast<int>(std::lround(seconds / kDt));
            for (int i = 0; i < n; ++i) scan();
        }
        void start(double mms) { run = true; setpoint = mms; }
    };
} // namespace

// ---------------------------------------------------------------- motor model

TEST(DriveMotor, SteadyStateIsGainTimesDrive) {
    DriveMotorModel m(kTau);
    for (int i = 0; i < 500; ++i) m.step(kDt, 50.0, 0);
    EXPECT_NEAR(m.speedMms(), 500.0, 0.5);                           // 10 mm/s per %
}

TEST(DriveMotor, FollowsTheDriveWithTheConfiguredLag) {
    DriveMotorModel m(kTau);
    for (int i = 0; i < 40; ++i) m.step(kDt, 50.0, 0);               // one time constant
    EXPECT_NEAR(m.speedMms(), 500.0 * (1.0 - std::exp(-1.0)), 3.0);
}

TEST(DriveMotor, EveryBoxCostsTwentyFiveMmsAtTheSameDrive) {
    for (int boxes = 0; boxes <= 4; ++boxes) {
        DriveMotorModel m(kTau);
        for (int i = 0; i < 600; ++i) m.step(kDt, 60.0, boxes);
        EXPECT_NEAR(m.speedMms(), 600.0 - 25.0 * boxes, 0.5) << boxes << " boxes";
    }
}

TEST(DriveMotor, NegativeDriveBrakesButNeverReverses) {
    DriveMotorModel m(kTau);
    for (int i = 0; i < 300; ++i) m.step(kDt, 50.0, 0);
    double previous = m.speedMms();
    for (int i = 0; i < 300; ++i) {
        m.step(kDt, -50.0, 0);
        EXPECT_LE(m.speedMms(), previous + 1e-12);
        EXPECT_GE(m.speedMms(), 0.0);
        previous = m.speedMms();
    }
    EXPECT_EQ(m.speedMms(), 0.0);
}

TEST(DriveMotor, BrakingStopsFasterThanCoasting) {
    DriveMotorModel coast(kTau), brake(kTau);
    for (int i = 0; i < 300; ++i) { coast.step(kDt, 50.0, 0); brake.step(kDt, 50.0, 0); }
    for (int i = 0; i < 50; ++i) { coast.step(kDt, 0.0, 0); brake.step(kDt, -100.0, 0); }
    EXPECT_LT(brake.speedMms(), coast.speedMms() * 0.2);
}

TEST(DriveMotor, DriveCommandIsClampedAndNanIsIgnored) {
    DriveMotorModel m(kTau);
    for (int i = 0; i < 800; ++i) m.step(kDt, 500.0, 0);             // absurd command
    EXPECT_NEAR(m.speedMms(), 1000.0, 1.0);                          // limited to 100 %
    m.step(kDt, std::nan(""), 0);
    EXPECT_TRUE(std::isfinite(m.speedMms()));
}

TEST(DriveMotor, StuckBeltDoesNotMove) {
    DriveMotorModel m(kTau);
    m.setStuck(true);
    for (int i = 0; i < 100; ++i) m.step(kDt, 100.0, 0);
    EXPECT_EQ(m.speedMms(), 0.0);
}

TEST(PlantMotorMode, IdealIsTheDefaultAndStillFollowsTheSetpoint) {
    Plant p;
    EXPECT_EQ(p.motorMode(), MotorMode::Ideal);
    Outputs out; out.conveyorMotor = true; out.speedSetpointMms = 500;
    for (int i = 0; i < 300; ++i) p.step(kDt, out);
    EXPECT_NEAR(p.beltSpeedMms(), 500.0, 1.0);
}

TEST(PlantMotorMode, PhysicalModeUsesTheDriveCommandAndIgnoresTheSetpoint) {
    PlantConfig cfg; cfg.motorMode = MotorMode::Physical;
    Plant p(cfg);
    Outputs out; out.conveyorMotor = true; out.speedSetpointMms = 500;
    for (int i = 0; i < 300; ++i) p.step(kDt, out);
    EXPECT_EQ(p.beltSpeedMms(), 0.0);                                 // no drive command: no motion
    p.setDriveCommand(40.0);
    for (int i = 0; i < 300; ++i) p.step(kDt, out);
    EXPECT_NEAR(p.beltSpeedMms(), 400.0, 1.0);
    EXPECT_EQ(p.inputs().conveyorSpeedFbMms, 400);
}

TEST(PlantMotorMode, BoxesOnTheBeltLoadThePhysicalMotor) {
    PlantConfig cfg; cfg.motorMode = MotorMode::Physical;
    Plant p(cfg);
    p.setDriveCommand(50.0);
    p.spawnBox({ 1, 120.0, 300.0 });
    p.spawnBox({ 1, 120.0, 300.0 });
    for (int i = 0; i < 400; ++i) p.step(kDt, Outputs{});
    EXPECT_EQ(p.boxesOnBelt(), 2);
    EXPECT_NEAR(p.beltSpeedMms(), 450.0, 1.0);
}

// ---------------------------------------------------------------- ramp

TEST(SpeedLoop, RampRespectsTheAccelerationLimit) {
    Rig r;
    r.start(500.0);
    double previous = 0.0, maxRate = 0.0;
    for (int i = 0; i < 100; ++i) {
        r.scan();
        maxRate = std::max(maxRate, (r.loop.rampedSetpointMms() - previous) / kDt);
        previous = r.loop.rampedSetpointMms();
    }
    EXPECT_NEAR(maxRate, 800.0, 1e-6);
    EXPECT_DOUBLE_EQ(r.loop.rampedSetpointMms(), 500.0);
}

TEST(SpeedLoop, RampRespectsTheDecelerationLimit) {
    Rig r;
    r.start(500.0);
    r.runFor(2.0);
    r.run = false;
    double previous = r.loop.rampedSetpointMms(), maxRate = 0.0;
    for (int i = 0; i < 100; ++i) {
        r.scan();
        maxRate = std::max(maxRate, (previous - r.loop.rampedSetpointMms()) / kDt);
        previous = r.loop.rampedSetpointMms();
    }
    EXPECT_NEAR(maxRate, 1000.0, 1e-6);
    EXPECT_DOUBLE_EQ(r.loop.rampedSetpointMms(), 0.0);
}

TEST(SpeedLoop, SetpointIsLimitedToTheValidRange) {
    Rig r;
    r.start(5000.0);
    r.runFor(5.0);
    EXPECT_LE(r.loop.rampedSetpointMms(), 1000.0);
    r.start(-300.0);
    r.runFor(3.0);
    EXPECT_GE(r.loop.rampedSetpointMms(), 0.0);
}

// ---------------------------------------------------------------- tracking and load rejection

TEST(SpeedLoop, ReachesAndHoldsTheSetpoint) {
    Rig r;
    r.start(500.0);
    r.runFor(2.0);
    EXPECT_NEAR(r.motor.speedMms(), 500.0, 3.0);
    r.runFor(3.0);
    EXPECT_NEAR(r.motor.speedMms(), 500.0, 1.0);
}

TEST(SpeedLoop, FollowsTheRampClosely) {
    Rig r;
    r.start(500.0);
    double worst = 0.0;
    for (int i = 0; i < 100; ++i) {
        r.scan();
        worst = std::max(worst, std::abs(r.loop.rampedSetpointMms() - r.motor.speedMms()));
    }
    EXPECT_LT(worst, 80.0);                                           // the motor lag alone would be ~320
}

TEST(SpeedLoop, RejectsTheLoadOfBoxesAppearingOnTheBelt) {
    Rig r;
    r.start(500.0);
    r.runFor(3.0);
    ASSERT_NEAR(r.motor.speedMms(), 500.0, 1.0);

    r.boxes = 4;                                                      // -100 mm/s at the same drive
    double worstDip = 0.0;
    for (int i = 0; i < 300; ++i) {
        r.scan();
        worstDip = std::max(worstDip, 500.0 - r.motor.speedMms());
    }
    EXPECT_GT(worstDip, 5.0);                                         // the load is felt...
    EXPECT_LT(worstDip, 70.0);                                        // ...but bounded
    EXPECT_NEAR(r.motor.speedMms(), 500.0, 1.0);                      // and fully rejected
    EXPECT_GT(r.loop.driveCommandPct(), 50.0 + 4 * 2.5 - 1.0);        // the drive made up for the load
}

TEST(SpeedLoop, RecoversWhenTheLoadIsRemoved) {
    Rig r;
    r.start(500.0);
    r.boxes = 5;
    r.runFor(4.0);
    ASSERT_NEAR(r.motor.speedMms(), 500.0, 1.0);
    r.boxes = 0;
    double worstRise = 0.0;
    for (int i = 0; i < 300; ++i) {
        r.scan();
        worstRise = std::max(worstRise, r.motor.speedMms() - 500.0);
    }
    EXPECT_LT(worstRise, 80.0);
    EXPECT_NEAR(r.motor.speedMms(), 500.0, 1.0);
}

TEST(SpeedLoop, OpenLoopSagsByTwentyFiveMmsPerBox) {
    SpeedLoopConfig cfg; cfg.openLoop = true;
    for (int boxes = 0; boxes <= 4; ++boxes) {
        Rig r(cfg);
        r.start(500.0);
        r.boxes = boxes;
        r.runFor(5.0);
        EXPECT_NEAR(r.motor.speedMms(), 500.0 - 25.0 * boxes, 1.0) << boxes << " boxes";
    }
}

TEST(SpeedLoop, ClosedLoopBeatsOpenLoopUnderLoad) {
    SpeedLoopConfig open; open.openLoop = true;
    Rig closed, ff(open);
    for (Rig* r : { &closed, &ff }) { r->start(500.0); r->boxes = 4; r->runFor(5.0); }
    EXPECT_NEAR(closed.motor.speedMms(), 500.0, 1.0);
    EXPECT_NEAR(ff.motor.speedMms(), 400.0, 1.0);
}

// ---------------------------------------------------------------- controlled stop

TEST(SpeedLoop, ControlledStopBrakesToStandstill) {
    Rig r;
    r.start(500.0);
    r.runFor(3.0);
    r.run = false;
    double minDrive = 0.0, previous = r.motor.speedMms();
    double stopTime = -1.0;
    for (int i = 0; i < 300; ++i) {
        r.scan();
        minDrive = std::min(minDrive, r.loop.driveCommandPct());
        ASSERT_LE(r.motor.speedMms(), previous + 1e-9) << "speed rose while stopping";
        ASSERT_GE(r.motor.speedMms(), 0.0);
        previous = r.motor.speedMms();
        if (stopTime < 0.0 && r.motor.speedMms() < 1.0) stopTime = (i + 1) * kDt;
    }
    EXPECT_LT(minDrive, 0.0);                                         // it actively braked
    EXPECT_GT(stopTime, 0.0);
    EXPECT_LT(stopTime, 1.5);
    EXPECT_EQ(r.loop.driveCommandPct(), 0.0);                         // drive released at rest
    EXPECT_EQ(r.loop.pid().iTerm(), 0.0);                             // integral forgotten
}

TEST(SpeedLoop, ControlledStopIsShorterThanCoasting) {
    Rig r;
    r.start(500.0);
    r.runFor(3.0);
    const double before = r.distanceMm;
    r.run = false;
    r.runFor(3.0);
    const double braked = r.distanceMm - before;

    DriveMotorModel coast(kTau);
    for (int i = 0; i < 300; ++i) coast.step(kDt, 50.0, 0);
    double coasted = 0.0;
    for (int i = 0; i < 300; ++i) { coast.step(kDt, 0.0, 0); coasted += coast.speedMms() * kDt; }

    EXPECT_LT(braked, coasted);                                       // ~125 mm for 1000 mm/s^2, vs ~200 mm
    EXPECT_GT(braked, 80.0);                                          // but not an instant stop
}

TEST(SpeedLoop, StaysQuietAtStandstill) {
    Rig r;
    r.runFor(2.0);
    EXPECT_EQ(r.motor.speedMms(), 0.0);
    EXPECT_EQ(r.loop.driveCommandPct(), 0.0);
    r.boxes = 3;                                                      // load on a stopped belt: nothing happens
    r.runFor(2.0);
    EXPECT_EQ(r.motor.speedMms(), 0.0);
}

TEST(SpeedLoop, RestartsCleanlyAfterAStop) {
    Rig r;
    r.start(500.0); r.runFor(3.0);
    r.run = false;  r.runFor(2.0);
    r.start(500.0); r.runFor(3.0);
    EXPECT_NEAR(r.motor.speedMms(), 500.0, 1.0);
}

// ---------------------------------------------------------------- stuck motor

TEST(SpeedLoop, StuckMotorSaturatesTheDriveButTheIntegralDoesNotWindUp) {
    Rig r;
    r.motor.setStuck(true);
    r.start(500.0);
    r.runFor(20.0);
    EXPECT_EQ(r.motor.speedMms(), 0.0);
    EXPECT_EQ(r.loop.driveCommandPct(), 100.0);                       // asking for everything
    EXPECT_TRUE(r.loop.pid().saturated());
    EXPECT_LT(r.loop.pid().iTerm(), 25.0);                            // integral stayed small
}

TEST(SpeedLoop, MotorFreedAfterBeingStuckDoesNotOvershootBadly) {
    const auto peakAfterRelease = [](bool antiWindup) {
        SpeedLoopConfig cfg;
        cfg.pid.antiWindup = antiWindup;
        Rig r(cfg);
        r.motor.setStuck(true);
        r.start(500.0);
        r.runFor(20.0);
        r.motor.setStuck(false);
        double peak = 0.0;
        for (int i = 0; i < 500; ++i) { r.scan(); peak = std::max(peak, r.motor.speedMms()); }
        return peak;
    };
    const double withAw = peakAfterRelease(true);
    const double without = peakAfterRelease(false);
    EXPECT_LT(withAw, 500.0 * 1.10);                                  // within 10 % of the setpoint
    EXPECT_GT(without, withAw + 100.0);                               // without it the belt lurches
}

// ---------------------------------------------------------------- robustness

TEST(SpeedLoop, BadDtLeavesTheLoopUnchanged) {
    Rig r;
    r.start(500.0);
    r.runFor(1.0);
    const double drive = r.loop.driveCommandPct();
    const double ramp = r.loop.rampedSetpointMms();
    for (double bad : { 0.0, -0.01, std::nan("") }) {
        EXPECT_EQ(r.loop.update(bad, true, 500.0, r.motor.speedMms()), drive);
        EXPECT_EQ(r.loop.rampedSetpointMms(), ramp);
    }
}

TEST(SpeedLoop, ResetRestartsTheRampFromTheGivenSpeed) {
    Rig r;
    r.start(500.0);
    r.runFor(3.0);
    r.loop.reset(300.0);
    EXPECT_DOUBLE_EQ(r.loop.rampedSetpointMms(), 300.0);
    EXPECT_EQ(r.loop.pid().iTerm(), 0.0);
    EXPECT_EQ(r.loop.driveCommandPct(), 0.0);
}
