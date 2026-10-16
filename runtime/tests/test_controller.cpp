#include "controller.h"
#include "plant.h"
#include <cmath>
#include <gtest/gtest.h>

using namespace plant;

namespace {
    constexpr double kDt = 0.010;

    // ---------------------------------------------------------------- controller alone

    Inputs healthy() {
        Inputs in;                      // emergencyStop and stopButton default to "healthy"
        in.stackerUpLimit = true;
        in.palletPresent = true;
        return in;
    }

    Outputs scans(SoftController& c, const Inputs& in, double seconds) {
        Outputs out;
        const int n = static_cast<int>(std::lround(seconds / kDt));
        for (int i = 0; i < n; ++i) out = c.update(in, kDt);
        return out;
    }

    void pressStart(SoftController& c, Inputs in) {
        in.startButton = false;  scans(c, in, kDt);
        in.startButton = true;   scans(c, in, kDt);
        in.startButton = false;  scans(c, in, kDt);
    }

    SoftController runningController() {
        SoftController c;
        const Inputs in = healthy();
        scans(c, in, kDt);
        pressStart(c, in);
        scans(c, in, 0.6);
        return c;
    }

    // ---------------------------------------------------------------- controller + plant

    struct Sim {
        Plant plant;
        SoftController ctrl;
        Inputs  lastIn;
        Outputs lastOut;

        explicit Sim(PlantConfig cfg = {}, unsigned seed = 1) : plant(cfg, seed) {}

        void scan() {
            lastIn = plant.inputs();
            lastOut = ctrl.update(lastIn, kDt);
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

    int boxesInSystem(const Plant& p) {
        return static_cast<int>(p.boxes().size()) + (p.holdingBox() ? 1 : 0);
    }
} // namespace

// ================================================================ state machine, controller only

TEST(Controller, PowersUpStoppedWithAllOutputsOff) {
    SoftController c;
    const Outputs out = c.update(healthy(), kDt);
    EXPECT_EQ(c.state(), State::Stopped);
    EXPECT_FALSE(out.conveyorMotor);
    EXPECT_FALSE(out.rejectCylinder);
    EXPECT_FALSE(out.stackerUp);
    EXPECT_FALSE(out.stackerDown);
    EXPECT_FALSE(out.gripperClose);
    EXPECT_EQ(out.plcState, 0);
}

TEST(Controller, StartButtonRunsStartupChecksThenRuns) {
    SoftController c;
    const Inputs in = healthy();
    scans(c, in, kDt);
    pressStart(c, in);
    EXPECT_EQ(c.state(), State::Starting);
    scans(c, in, 0.6);
    EXPECT_EQ(c.state(), State::Running);
}

TEST(Controller, RunningDrivesTheConveyorAtTheRunSpeed) {
    SoftController c = runningController();
    const Outputs out = c.update(healthy(), kDt);
    EXPECT_TRUE(out.conveyorMotor);
    EXPECT_EQ(out.speedSetpointMms, 500);
    EXPECT_EQ(out.plcState, 2);
}

TEST(Controller, StartupCheckFailsWithoutPallet) {
    SoftController c;
    Inputs in = healthy();
    in.palletPresent = false;
    scans(c, in, kDt);
    pressStart(c, in);
    scans(c, in, 0.6);
    EXPECT_EQ(c.state(), State::Fault);
    EXPECT_EQ(c.fault(), FaultCode::StartupCheck);
}

TEST(Controller, StartupCheckFailsWithStackerNotAtHome) {
    SoftController c;
    Inputs in = healthy();
    in.stackerUpLimit = false;
    scans(c, in, kDt);
    pressStart(c, in);
    scans(c, in, 0.6);
    EXPECT_EQ(c.fault(), FaultCode::StartupCheck);
}

TEST(Controller, CannotStartWhileStopIsHeld) {
    SoftController c;
    Inputs in = healthy();
    in.stopButton = false;                       // NC contact open = pressed
    scans(c, in, kDt);
    pressStart(c, in);
    EXPECT_EQ(c.state(), State::Stopped);
}

TEST(Controller, EmergencyStopFromRunningCutsAllOutputs) {
    SoftController c = runningController();
    Inputs in = healthy();
    in.emergencyStop = false;
    const Outputs out = c.update(in, kDt);
    EXPECT_EQ(c.state(), State::EmergencyStop);
    EXPECT_EQ(c.fault(), FaultCode::EmergencyStop);
    EXPECT_FALSE(out.conveyorMotor);
    EXPECT_FALSE(out.gripperClose);
    EXPECT_EQ(out.plcState, 10);
}

TEST(Controller, EmergencyStopNeedsReleaseAndReset) {
    SoftController c = runningController();
    Inputs in = healthy();
    in.emergencyStop = false;
    scans(c, in, 0.1);

    pressStart(c, in);                           // reset while still pressed: ignored
    EXPECT_EQ(c.state(), State::EmergencyStop);

    in.emergencyStop = true;                     // released, but no reset yet
    scans(c, in, 0.5);
    EXPECT_EQ(c.state(), State::EmergencyStop);

    pressStart(c, in);                           // released + reset
    EXPECT_EQ(c.state(), State::Stopped);
    EXPECT_EQ(c.fault(), FaultCode::None);
}

TEST(Controller, FaultCannotBeAcknowledgedImmediately) {
    SoftController c;
    Inputs in = healthy();
    in.palletPresent = false;
    scans(c, in, kDt);
    pressStart(c, in);
    scans(c, in, 0.6);
    ASSERT_EQ(c.state(), State::Fault);

    pressStart(c, in);                           // too soon after the fault
    EXPECT_EQ(c.state(), State::Fault);
}

TEST(Controller, FaultIsAcknowledgedWithStartAfterTheHoldTime) {
    SoftController c;
    Inputs in = healthy();
    in.palletPresent = false;
    scans(c, in, kDt);
    pressStart(c, in);
    scans(c, in, 0.6);
    ASSERT_EQ(c.state(), State::Fault);

    scans(c, in, 1.5);
    pressStart(c, in);
    EXPECT_EQ(c.state(), State::Stopped);
    EXPECT_EQ(c.fault(), FaultCode::None);
}

TEST(Controller, HeartbeatCountsEveryScan) {
    SoftController c;
    const Inputs in = healthy();
    const Outputs a = c.update(in, kDt);
    const Outputs b = c.update(in, kDt);
    const Outputs d = c.update(in, kDt);
    EXPECT_EQ(b.heartbeat, a.heartbeat + 1);
    EXPECT_EQ(d.heartbeat, b.heartbeat + 1);
}

TEST(Controller, RecipeSelection) {
    SoftController c;
    EXPECT_FALSE(c.setRecipe(99));
    EXPECT_EQ(c.recipe().id, 1);                 // unchanged by the failed call
    EXPECT_TRUE(c.setRecipe(2));
    EXPECT_EQ(c.update(healthy(), kDt).activeRecipeId, 2);
}

// ================================================================ closed loop: controller + plant

TEST(ClosedLoop, GoodBoxIsInspectedAndStacked) {
    Sim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.goodCount() == 1 && s.ctrl.state() == State::Running; }, 60.0));
    EXPECT_EQ(s.plant.placedTotal(), 1);
    EXPECT_EQ(s.plant.rejectedCount(), 0);
    EXPECT_EQ(s.plant.droppedCount(), 0);
    EXPECT_NEAR(s.ctrl.lastHeightMm(), 120.0, 1.0);
}

TEST(ClosedLoop, WrongHeightBoxIsRejected) {
    Sim s;
    s.plant.spawnBox({ 2, 80.0, 300.0 });
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.rejectCount() == 1 && s.ctrl.state() == State::Running; }, 60.0));
    EXPECT_EQ(s.plant.rejectedCount(), 1);
    EXPECT_EQ(s.plant.placedTotal(), 0);
    EXPECT_EQ(s.ctrl.goodCount(), 0);
    EXPECT_NEAR(s.ctrl.lastHeightMm(), 80.0, 1.0);
}

TEST(ClosedLoop, RecipeChangeChangesWhichBoxIsGood) {
    Sim s;
    ASSERT_TRUE(s.ctrl.setRecipe(2));            // recipe 2 expects 80 mm
    s.plant.spawnBox({ 2, 80.0, 300.0 });
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.goodCount() == 1 && s.ctrl.state() == State::Running; }, 60.0));
    EXPECT_EQ(s.plant.rejectedCount(), 0);
}

TEST(ClosedLoop, ProductionCountsAgreeAndNoBoxIsLost) {
    Sim s(feederConfig({ 0.70, 0.15, 0.15 }), 5);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.palletsCompleted() >= 1; }, 400.0));

    EXPECT_EQ(s.ctrl.goodCount(), s.plant.placedTotal());
    EXPECT_EQ(s.ctrl.rejectCount(), s.plant.rejectedCount());
    EXPECT_EQ(s.plant.droppedCount(), 0);
    EXPECT_NE(s.ctrl.state(), State::Fault);
    // Inventory: every box that was fed is stacked, rejected, dropped or still in the machine.
    EXPECT_EQ(s.plant.spawnedCount(),
        s.plant.placedTotal() + s.plant.rejectedCount() + s.plant.droppedCount() + boxesInSystem(s.plant));
}

TEST(ClosedLoop, FullPalletIsChangedAndProductionResumes) {
    Sim s(feederConfig({ 1.0, 0.0, 0.0 }, 2), 3);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.palletsCompleted() == 1; }, 200.0));
    EXPECT_EQ(s.plant.palletsCompleted(), 1);
    EXPECT_EQ(s.plant.placedOnPallet(), 0);       // new, empty pallet
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.goodCount() >= 3; }, 60.0));
    EXPECT_NE(s.ctrl.state(), State::Fault);
    EXPECT_EQ(s.plant.droppedCount(), 0);
}

TEST(ClosedLoop, EveryWrongBoxIsRejected) {
    Sim s(feederConfig({ 0.0, 1.0, 0.0 }), 2);
    s.pressStart();
    s.run(90.0);
    EXPECT_GE(s.ctrl.rejectCount(), 3);
    EXPECT_EQ(s.ctrl.goodCount(), 0);
    EXPECT_EQ(s.plant.droppedCount(), 0);
    EXPECT_NE(s.ctrl.state(), State::Fault);
}

TEST(ClosedLoop, InterlocksHoldOnEveryScan) {
    Sim s(feederConfig({ 0.70, 0.15, 0.15 }), 11);
    s.pressStart();
    for (int i = 0; i < 12000; ++i) {             // 120 s of production
        s.scan();
        ASSERT_TRUE(!(s.lastOut.stackerUp && s.lastOut.stackerDown));
        ASSERT_TRUE(!(s.lastOut.conveyorMotor && !s.lastIn.stackerUpLimit));
    }
    EXPECT_GT(s.ctrl.goodCount(), 0);
}

TEST(ClosedLoop, StopButtonStopsBetweenBoxesWithoutDroppingAnything) {
    Sim s(feederConfig({ 1.0, 0.0, 0.0 }), 4);
    s.pressStart();
    s.run(20.0);
    s.plant.setPanel({ false, true, false });       // STOP held
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Stopped; }, 60.0));
    EXPECT_EQ(s.plant.droppedCount(), 0);
    EXPECT_FALSE(s.lastOut.conveyorMotor);
}

TEST(ClosedLoop, EmergencyStopStopsEverythingWithinOneScan) {
    Sim s(feederConfig({ 1.0, 0.0, 0.0 }), 4);
    s.pressStart();
    s.run(8.0);
    s.plant.setPanel({ false, false, true });
    s.run(0.03);                                  // plant publishes the input, controller reacts
    EXPECT_EQ(s.ctrl.state(), State::EmergencyStop);
    EXPECT_FALSE(s.lastOut.conveyorMotor);
    EXPECT_FALSE(s.lastOut.stackerUp);
    EXPECT_FALSE(s.lastOut.stackerDown);
}

// ================================================================ faults

TEST(Faults, StuckMotorIsDetectedFromCommandedVersusActualSpeed) {
    Sim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.plant.motor().setStuck(true);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Fault; }, 15.0));
    EXPECT_EQ(s.ctrl.fault(), FaultCode::MotorFault);
    EXPECT_FALSE(s.lastOut.conveyorMotor);
}

TEST(Faults, StackerThatNeverReachesTheBottomTimesOut) {
    Sim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.plant.stacker().setStuck(true);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Fault; }, 60.0));
    EXPECT_EQ(s.ctrl.fault(), FaultCode::StackerTimeout);
}

TEST(Faults, MachineRecoversAfterTheStackerFaultIsCleared) {
    Sim s;
    s.plant.spawnBox({ 1, 120.0, 300.0 });
    s.plant.stacker().setStuck(true);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Fault; }, 60.0));

    s.plant.stacker().setStuck(false);            // the technician fixes the stacker
    s.run(1.5);
    s.pressStart();                               // acknowledge
    ASSERT_EQ(s.ctrl.state(), State::Stopped);
    s.pressStart();                               // restart
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.goodCount() == 1; }, 60.0));
    EXPECT_EQ(s.plant.placedTotal(), 1);
    EXPECT_EQ(s.plant.droppedCount(), 0);
}

TEST(Faults, MissingPalletAtStartIsRefused) {
    PlantConfig cfg;
    cfg.startWithPallet = false;
    Sim s(cfg);
    s.plant.blockPalletArrival(true);
    s.pressStart();
    s.run(2.0);
    EXPECT_EQ(s.ctrl.state(), State::Fault);
    EXPECT_EQ(s.ctrl.fault(), FaultCode::StartupCheck);
}

TEST(Faults, PalletThatNeverArrivesAfterAChangeIsReported) {
    Sim s(feederConfig({ 1.0, 0.0, 0.0 }, 1), 6);
    s.pressStart();
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::PalletChange; }, 120.0));
    s.plant.blockPalletArrival(true);             // the replacement pallet is never delivered
    ASSERT_TRUE(s.runUntil([&] { return s.ctrl.state() == State::Fault; }, 30.0));
    EXPECT_EQ(s.ctrl.fault(), FaultCode::MissingPallet);
}

TEST(Faults, BoxLeftAtPickupWithoutInspectionIsNotStacked) {
    Sim s;
    s.plant.spawnBox({ 2, 80.0, 300.0 });
    s.plant.setPanel({});
    s.run(8.0);                                   // belt off: box has not moved
    EXPECT_FALSE(s.plant.inputs().pickupSensor);
    // Start the controller with a box already parked at the stop (never validated).
    Plant parked;
    parked.spawnBox({ 2, 80.0, 300.0 });
    Outputs belt; belt.conveyorMotor = true; belt.speedSetpointMms = 500;
    for (int i = 0; i < 800; ++i) parked.step(kDt, belt);
    ASSERT_TRUE(parked.inputs().pickupSensor);

    SoftController c;
    Inputs in = parked.inputs();
    scans(c, in, kDt);
    pressStart(c, in);
    scans(c, in, 0.6);
    EXPECT_EQ(c.state(), State::Fault);
    EXPECT_EQ(c.fault(), FaultCode::UnexpectedBoxAtPickup);
}