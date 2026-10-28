// The whole machine with the PID in the loop: SoftController (PLC) + SpeedLoop + Plant (physical motor).
#include "controller.h"
#include "plant.h"
#include "speed_loop.h"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

using namespace plant;

namespace {
    constexpr double kDt = 0.010;

    struct PidSim {
        Plant plant;
        SoftController ctrl;
        SpeedLoop loop;
        Inputs  lastIn;
        Outputs lastOut;

        explicit PidSim(PlantConfig cfg = {}, unsigned seed = 1, SpeedLoopConfig lc = {})
            : plant(prepare(cfg), seed), loop(lc) {}

        static PlantConfig prepare(PlantConfig cfg) { cfg.motorMode = MotorMode::Physical; return cfg; }

        void scan() {
            lastIn = plant.inputs();
            lastOut = ctrl.update(lastIn, kDt);
            plant.setDriveCommand(loop.update(kDt, lastOut, lastIn));
            plant.step(kDt, lastOut);
        }
        void run(double seconds) {
            const int n = static_cast<int>(std::lround(seconds / kDt));
            for (int i = 0; i < n; ++i) scan();
        }
        template <class Pred>
        bool runUntil(Pred done, double maxSeconds) {
            const int n = static_cast<int>(std::lround(maxSeconds / kDt));
            for (int i = 0; i < n; ++i) {
                if (done()) return true;
                scan();
            }
            return done();
        }
        void press(OperatorPanel p, double seconds = 0.15) {
            plant.setPanel(p);
            run(seconds);
            plant.setPanel({});
            run(0.05);
        }
        void pressStart() { press({ true, false, false }); }
    };

    PlantConfig feederConfig(std::vector<double> weights, int capacity = 4) {
        PlantConfig cfg;
        cfg.feeder.enabled = true;
        cfg.feeder.minGapMm = 800.0;
        cfg.feeder.weights = std::move(weights);
        cfg.palletCapacity = capacity;
        return cfg;
    }
} // namespace

TEST(MachinePid, OneBoxIsInspectedAndStackedWithThePidInTheLoop) {
    PidSim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.goodCount() == 1; }, 60.0));
    EXPECT_EQ(s.plant.placedTotal(), 1);
    EXPECT_EQ(s.plant.droppedCount(), 0);
    EXPECT_NE(s.ctrl.state(), State::Fault);
}

TEST(MachinePid, WrongBoxIsRejected) {
    PidSim s;
    s.plant.spawnBox({ 2, 80.0, 300.0 });                  // recipe 1 expects 120 mm
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.rejectCount() == 1; }, 60.0));
    EXPECT_EQ(s.plant.rejectedCount(), 1);
    EXPECT_EQ(s.plant.placedTotal(), 0);
}

TEST(MachinePid, BeltStopsAtTheInspectionStationAndTheBoxIsMeasured) {
    PidSim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Inspection; }, 20.0));
    ASSERT_TRUE(s.runUntil([&] { return s.lastIn.conveyorSpeedFbMms <= 20; }, 3.0));   // the PLC's "settled" test
    EXPECT_TRUE(s.plant.inputs().inspectionSensor);                                     // box still on the sensor
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() != State::Inspection; }, 5.0));
    EXPECT_EQ(s.ctrl.state(), State::Transferring);                                     // height was accepted
    EXPECT_NEAR(s.ctrl.lastHeightMm(), 120.0, 1.0);
}

TEST(MachinePid, ProductionRunCompletesAPalletWithoutFaultsOrDroppedBoxes) {
    PidSim s(feederConfig({ 1.0, 0.0, 0.0 }, 2), 4);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.palletsCompleted() == 1; }, 240.0));
    EXPECT_EQ(s.plant.droppedCount(), 0);
    EXPECT_EQ(s.ctrl.goodCount(), 2);
    EXPECT_NE(s.ctrl.state(), State::Fault);
}

TEST(MachinePid, MixedBoxesAreSortedCorrectly) {
    PidSim s(feederConfig({ 0.5, 0.25, 0.25 }, 2), 9);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.rejectCount() >= 1 && s.ctrl.palletsCompleted() >= 1; }, 600.0));
    EXPECT_EQ(s.plant.droppedCount(), 0);
    EXPECT_EQ(s.plant.rejectedCount(), s.ctrl.rejectCount());
    EXPECT_NE(s.ctrl.state(), State::Fault);
}

TEST(MachinePid, BeltHoldsTheRunSpeedWhileBoxesQueueUpOnIt) {
    PlantConfig cfg;
    cfg.feeder.enabled = true;
    cfg.feeder.intervalS = 1.0;                            // keep several boxes on the belt
    cfg.feeder.minGapMm = 150.0;
    PidSim s(cfg);
    // Drive the belt directly through the PLC's run command: START, then watch the speed under load.
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Running; }, 5.0));
    double worstSag = 0.0;
    int samples = 0;
    for (int i = 0; i < 200 && s.ctrl.state() == State::Running; ++i) {
        s.scan();
        if (s.plant.boxesOnBelt() >= 2 && s.loop.rampedSetpointMms() == 500.0) {
            worstSag = std::max(worstSag, 500.0 - s.plant.beltSpeedMms());
            ++samples;
        }
    }
    ASSERT_GT(samples, 0);
    EXPECT_LT(worstSag, 40.0);                             // open loop would sag 50 mm/s with two boxes
}

TEST(MachinePid, OpenLoopBeltSagsWithTheLoadWhileClosedLoopDoesNot) {
    const auto speedWithBoxes = [](bool openLoop) {
        SpeedLoopConfig lc; lc.openLoop = openLoop;
        PlantConfig cfg;
        cfg.feeder.enabled = true;
        cfg.feeder.intervalS = 0.5;
        cfg.feeder.minGapMm = 100.0;
        PidSim s(cfg, 1, lc);
        // No PLC: command the belt directly so the queue builds up behind the mechanical stop.
        Outputs out; out.conveyorMotor = true; out.speedSetpointMms = 500;
        Inputs in;
        for (int i = 0; i < 3000; ++i) {
            in.conveyorSpeedFbMms = static_cast<std::int16_t>(std::lround(s.plant.beltSpeedMms()));
            s.plant.setDriveCommand(s.loop.update(kDt, out, in));
            s.plant.step(kDt, out);
        }
        return std::pair<double, int>{ s.plant.beltSpeedMms(), s.plant.boxesOnBelt() };
    };
    const auto [closedSpeed, closedBoxes] = speedWithBoxes(false);
    const auto [openSpeed, openBoxes] = speedWithBoxes(true);
    ASSERT_GE(closedBoxes, 3);
    ASSERT_GE(openBoxes, 3);
    EXPECT_NEAR(closedSpeed, 500.0, 2.0);
    EXPECT_NEAR(openSpeed, 500.0 - 25.0 * openBoxes, 2.0);
}

TEST(MachinePid, StuckMotorRaisesTheMotorFaultAndTheDriveIsReleased) {
    PidSim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.plant.setMotorStuck(true);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Fault; }, 10.0));
    EXPECT_EQ(s.ctrl.fault(), FaultCode::MotorFault);
    s.run(2.0);
    EXPECT_EQ(s.loop.driveCommandPct(), 0.0);              // the PLC stopped asking: no lingering full drive
    EXPECT_LT(s.loop.pid().iTerm(), 25.0);
}

TEST(MachinePid, MachineRecoversAfterAStuckMotorIsFixed) {
    PidSim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.plant.setMotorStuck(true);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Fault; }, 10.0));
    s.plant.setMotorStuck(false);
    s.run(1.5);
    s.pressStart();                                        // acknowledge
    ASSERT_EQ(s.ctrl.state(), State::Stopped);
    s.pressStart();                                        // restart
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.goodCount() == 1; }, 60.0));
    EXPECT_EQ(s.plant.droppedCount(), 0);
}

TEST(MachinePid, EmergencyStopBringsTheBeltToAStop) {
    PidSim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.plant.beltSpeedMms() > 450.0; }, 10.0));
    s.plant.setPanel({ false, false, true });              // mushroom pressed
    s.run(0.05);
    EXPECT_EQ(s.ctrl.state(), State::EmergencyStop);
    s.run(2.0);
    EXPECT_LT(s.plant.beltSpeedMms(), 1.0);
    EXPECT_EQ(s.loop.driveCommandPct(), 0.0);
}

TEST(MachinePid, IdealMotorModeIsUnaffectedByTheSpeedLoopExistingAtAll) {
    // Step 3 behaviour: Ideal motor, no SpeedLoop in the scan.
    Plant plant;                                           // default config
    SoftController ctrl;
    plant.spawnBox({ 1, 120.0, 300.0 });
    const auto scan = [&] {
        const Inputs in = plant.inputs();
        plant.step(kDt, ctrl.update(in, kDt));
    };
    plant.setPanel({ true, false, false });
    for (int i = 0; i < 15; ++i) scan();
    plant.setPanel({});
    for (int i = 0; i < 6000 && ctrl.goodCount() == 0; ++i) scan();
    EXPECT_EQ(ctrl.goodCount(), 1);
    EXPECT_EQ(plant.motorMode(), MotorMode::Ideal);
}
