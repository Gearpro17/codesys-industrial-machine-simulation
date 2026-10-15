#include "plant.h"
#include "recipes.h"
#include <cmath>
#include <gtest/gtest.h>

using namespace plant;

namespace {
    constexpr double kDt = 0.010;

    Outputs runAt(int speedMms) {
        Outputs o;
        o.conveyorMotor = true;
        o.speedSetpointMms = static_cast<std::int16_t>(speedMms);
        return o;
    }

    void runFor(Plant& p, const Outputs& out, double seconds) {
        const int steps = static_cast<int>(std::lround(seconds / kDt));
        for (int i = 0; i < steps; ++i) p.step(kDt, out);
    }

    // Spawn a box and run the belt long enough for it to reach the pickup stop.
    void deliverBox(Plant& p, const BoxSpec& spec) {
        p.spawnBox(spec);
        runFor(p, runAt(500), 8.0);
    }

    // The PLC's stacking sequence: down, close, up, open. Belt stopped.
    void pickAndPlace(Plant& p) {
        Outputs out;
        out.stackerDown = true;                     runFor(p, out, 1.5);   // stroke 1.2 s
        out.gripperClose = true;                    runFor(p, out, 0.7);   // stroke 0.5 s
        out.stackerDown = false; out.stackerUp = true; runFor(p, out, 1.5);
        out.gripperClose = false;                   runFor(p, out, 0.7);
    }
} // namespace

// ---------------------------------------------------------------- motor

TEST(MotorModel, ConvergesToSetpoint) {
    MotorModel m(0.4);
    for (int i = 0; i < 200; ++i) m.step(kDt, true, 500.0);   // 2 s = 5 tau
    EXPECT_GT(m.speedMms(), 490.0);
    EXPECT_LT(m.speedMms(), 500.0);                            // never overshoots
}

TEST(MotorModel, DoesNotJumpInstantly) {
    MotorModel m(0.4);
    m.step(kDt, true, 500.0);
    EXPECT_LT(m.speedMms(), 20.0);
}

TEST(MotorModel, StuckMotorStaysAtZero) {
    MotorModel m(0.4);
    m.setStuck(true);
    for (int i = 0; i < 100; ++i) m.step(kDt, true, 500.0);
    EXPECT_EQ(m.speedMms(), 0.0);
}

// ---------------------------------------------------------------- belt and sensors

TEST(Plant, EntrySensorTripsWhenBoxPasses) {
    Plant p;
    p.spawnBox({ 1, 120.0, 300.0 });
    const Outputs out = runAt(500);
    bool seen = false;
    for (int i = 0; i < 300 && !seen; ++i) {
        p.step(kDt, out);
        seen = p.inputs().entrySensor;
    }
    EXPECT_TRUE(seen);
}

TEST(Plant, HeightMeasuredOnlyAtInspection) {
    Plant p;
    p.spawnBox({ 1, 120.0, 300.0 });
    const Outputs out = runAt(500);
    p.step(kDt, out);
    EXPECT_EQ(p.inputs().boxHeightDmm, 0);                     // no box at the station yet

    bool inspected = false;
    for (int i = 0; i < 400 && !inspected; ++i) {
        p.step(kDt, out);
        inspected = p.inputs().inspectionSensor;
    }
    ASSERT_TRUE(inspected);
    EXPECT_NEAR(p.inputs().boxHeightDmm / 10.0, 120.0, 2.0);   // 2 mm is far above noise
}

TEST(Plant, SameSeedGivesSameReadings) {
    Plant a({}, 42), b({}, 42);
    a.spawnBox({ 1, 120.0, 300.0 });
    b.spawnBox({ 1, 120.0, 300.0 });
    const Outputs out = runAt(500);
    for (int i = 0; i < 300; ++i) {
        a.step(kDt, out);
        b.step(kDt, out);
        EXPECT_EQ(a.inputs().boxHeightDmm, b.inputs().boxHeightDmm);
    }
}

TEST(Plant, BoxStopsAtMechanicalEnd) {
    Plant p;
    p.spawnBox({ 1, 120.0, 300.0 });
    const Outputs out = runAt(500);
    for (int i = 0; i < 1000; ++i) p.step(kDt, out);           // 10 s
    EXPECT_DOUBLE_EQ(p.boxes()[0].frontMm, 2000.0);
}

TEST(Plant, BoxCoastsAfterMotorOff) {
    Plant p;
    p.spawnBox({ 1, 120.0, 300.0 });
    Outputs out = runAt(500);
    for (int i = 0; i < 100; ++i) p.step(kDt, out);            // reach speed
    const double before = p.boxes()[0].frontMm;
    out.conveyorMotor = false;
    for (int i = 0; i < 300; ++i) p.step(kDt, out);
    const double coasted = p.boxes()[0].frontMm - before;
    EXPECT_GT(coasted, 50.0);                                  // physically plausible window
    EXPECT_LT(coasted, 300.0);
}

// ---------------------------------------------------------------- actuators

TEST(Stacker, StartsAtUpLimit) {
    Plant p;
    p.step(kDt, Outputs{});
    EXPECT_TRUE(p.inputs().stackerUpLimit);
    EXPECT_FALSE(p.inputs().stackerDownLimit);
}

TEST(Stacker, TravelsDownInConfiguredTime) {
    Plant p;
    Outputs out; out.stackerDown = true;
    for (int i = 0; i < 60; ++i) p.step(kDt, out);          // 0.6 s: mid-travel
    EXPECT_FALSE(p.inputs().stackerUpLimit);
    EXPECT_FALSE(p.inputs().stackerDownLimit);
    for (int i = 0; i < 70; ++i) p.step(kDt, out);          // 1.3 s total > 1.2 s
    EXPECT_TRUE(p.inputs().stackerDownLimit);
}

TEST(Stacker, BothCommandsHoldPosition) {
    Plant p;
    Outputs out; out.stackerUp = true; out.stackerDown = true;
    for (int i = 0; i < 200; ++i) p.step(kDt, out);
    EXPECT_TRUE(p.inputs().stackerUpLimit);                  // never moved from home
}

TEST(Stacker, StuckNeverReachesLimit) {
    Plant p;
    p.stacker().setStuck(true);
    Outputs out; out.stackerDown = true;
    for (int i = 0; i < 300; ++i) p.step(kDt, out);
    EXPECT_FALSE(p.inputs().stackerDownLimit);               // timeout fault scenario
}

TEST(Gripper, ClosesAndReopens) {
    Plant p;
    Outputs out; out.gripperClose = true;
    for (int i = 0; i < 60; ++i) p.step(kDt, out);
    EXPECT_TRUE(p.inputs().gripperClosedFb);
    out.gripperClose = false;
    for (int i = 0; i < 60; ++i) p.step(kDt, out);
    EXPECT_FALSE(p.inputs().gripperClosedFb);
}

TEST(RejectCylinder, RemovesBoxAtInspectionStation) {
    Plant p;
    p.spawnBox({ 1, 120.0, 300.0 });
    Outputs out = runAt(500);
    for (int i = 0; i < 400 && !p.inputs().inspectionSensor; ++i) p.step(kDt, out);
    ASSERT_TRUE(p.inputs().inspectionSensor);

    out.conveyorMotor = false;
    out.rejectCylinder = true;
    for (int i = 0; i < 50; ++i) p.step(kDt, out);           // 0.5 s > 0.3 s stroke

    EXPECT_TRUE(p.boxes().empty());
    EXPECT_EQ(p.rejectedCount(), 1);
    EXPECT_FALSE(p.inputs().inspectionSensor);
}

TEST(RejectCylinder, NoBoxAtStationMeansNothingRejected) {
    Plant p;
    Outputs out; out.rejectCylinder = true;
    for (int i = 0; i < 50; ++i) p.step(kDt, out);
    EXPECT_EQ(p.rejectedCount(), 0);
}

// ---------------------------------------------------------------- recipes

TEST(Recipe, HeightWithinToleranceMatches) {
    const Recipe r{ 1, "Box M", 120.0, 2.0 };
    EXPECT_TRUE(heightMatches(r, 120.0));
    EXPECT_TRUE(heightMatches(r, 121.9));
    EXPECT_TRUE(heightMatches(r, 118.1));
}

TEST(Recipe, HeightOutsideToleranceRejected) {
    const Recipe r{ 1, "Box M", 120.0, 2.0 };
    EXPECT_FALSE(heightMatches(r, 122.5));
    EXPECT_FALSE(heightMatches(r, 80.0));
}

TEST(Recipe, DefaultTableCoversEveryDefaultBoxType) {
    const PlantConfig cfg;
    for (const auto& box : cfg.feeder.catalog) {
        bool found = false;
        for (const auto& r : defaultRecipes())
            found = found || (r.id == box.typeId && heightMatches(r, box.heightMm));
        EXPECT_TRUE(found) << "no recipe for box type " << box.typeId;
    }
}

TEST(Recipe, NoiseStaysInsideToleranceWindow) {
    // Sensor noise (sigma 0.3 mm) must be far below the 2 mm tolerance, or good
    // boxes would be rejected at random.
    Plant p;
    p.spawnBox({ 1, 120.0, 300.0 });
    const Recipe r = defaultRecipes()[0];
    const Outputs out = runAt(500);
    int samples = 0;
    for (int i = 0; i < 600; ++i) {
        p.step(kDt, out);
        if (p.inputs().inspectionSensor) {
            ++samples;
            EXPECT_TRUE(heightMatches(r, p.inputs().boxHeightDmm / 10.0));
        }
    }
    EXPECT_GT(samples, 0);
}

// ---------------------------------------------------------------- feeder

TEST(Feeder, DisabledByDefault) {
    Plant p;
    runFor(p, runAt(500), 10.0);
    EXPECT_EQ(p.spawnedCount(), 0);
}

TEST(Feeder, SpawnsBoxesAtConfiguredInterval) {
    PlantConfig cfg;
    cfg.feeder.enabled = true;
    cfg.feeder.intervalS = 3.0;
    Plant p(cfg);
    runFor(p, runAt(500), 10.0);
    EXPECT_GE(p.spawnedCount(), 3);
    EXPECT_LE(p.spawnedCount(), 4);
}

TEST(Feeder, BoxesNeverOverlap) {
    PlantConfig cfg;
    cfg.feeder.enabled = true;
    cfg.feeder.intervalS = 1.0;                      // aggressive: forces queueing
    Plant p(cfg);
    const Outputs out = runAt(500);
    for (int i = 0; i < 6000; ++i) {                 // 60 s, nobody picks up
        p.step(kDt, out);
        const auto& b = p.boxes();
        for (std::size_t k = 1; k < b.size(); ++k) {
            const double rearOfPrevious = b[k - 1].frontMm - b[k - 1].spec.lengthMm;
            ASSERT_LE(b[k].frontMm, rearOfPrevious + 1e-9) << "overlap at step " << i;
        }
    }
    EXPECT_GE(p.boxes().size(), 2u);                 // queue built up behind the stop
    EXPECT_DOUBLE_EQ(p.boxes().front().frontMm, 2000.0);
}

TEST(Feeder, SameSeedGivesSameBoxSequence) {
    PlantConfig cfg;
    cfg.feeder.enabled = true;
    cfg.feeder.intervalS = 1.0;
    Plant a(cfg, 7), b(cfg, 7);
    const Outputs out = runAt(500);
    runFor(a, out, 20.0);
    runFor(b, out, 20.0);
    ASSERT_EQ(a.boxes().size(), b.boxes().size());
    for (std::size_t i = 0; i < a.boxes().size(); ++i)
        EXPECT_EQ(a.boxes()[i].spec.typeId, b.boxes()[i].spec.typeId);
}

TEST(Feeder, ProducesMoreThanOneBoxType) {
    PlantConfig cfg;
    cfg.feeder.enabled = true;
    cfg.feeder.intervalS = 0.5;
    cfg.feeder.weights = { 1.0, 1.0, 1.0 };
    Plant p(cfg, 3);
    int seen[4] = { 0, 0, 0, 0 };
    const Outputs out = runAt(500);
    int lastSpawned = 0;
    for (int i = 0; i < 12000; ++i) {                // 120 s, drain with rejects below
        p.step(kDt, out);
        if (p.spawnedCount() != lastSpawned) {
            lastSpawned = p.spawnedCount();
            ++seen[p.boxes().back().spec.typeId];
        }
        if (p.boxes().size() >= 5) break;            // belt full, enough samples
    }
    int kinds = 0;
    for (int t = 1; t <= 3; ++t) kinds += seen[t] > 0 ? 1 : 0;
    EXPECT_GE(kinds, 2);
}

// ---------------------------------------------------------------- pickup, stacking, pallet

TEST(Pickup, SensorTripsWhenBoxReachesStop) {
    Plant p;
    EXPECT_FALSE((p.step(kDt, Outputs{}), p.inputs().pickupSensor));
    deliverBox(p, { 1, 120.0, 300.0 });
    EXPECT_TRUE(p.inputs().pickupSensor);
    EXPECT_DOUBLE_EQ(p.boxes().front().frontMm, 2000.0);
}

TEST(Stacking, FullCyclePlacesBoxOnPallet) {
    Plant p;
    deliverBox(p, { 1, 120.0, 300.0 });
    pickAndPlace(p);
    EXPECT_TRUE(p.boxes().empty());
    EXPECT_FALSE(p.holdingBox());
    EXPECT_EQ(p.placedOnPallet(), 1);
    EXPECT_EQ(p.placedTotal(), 1);
    EXPECT_EQ(p.droppedCount(), 0);
    EXPECT_FALSE(p.inputs().pickupSensor);
}

TEST(Stacking, GripperClosingOnNothingGrabsNothing) {
    Plant p;
    Outputs out;
    out.stackerDown = true;   runFor(p, out, 1.5);
    out.gripperClose = true;  runFor(p, out, 0.7);
    EXPECT_TRUE(p.inputs().gripperClosedFb);
    EXPECT_FALSE(p.holdingBox());
}

TEST(Stacking, GripperClosedAboveTheBoxGrabsNothing) {
    Plant p;
    deliverBox(p, { 1, 120.0, 300.0 });
    Outputs out;
    out.gripperClose = true;  runFor(p, out, 0.7);       // stacker still up
    EXPECT_FALSE(p.holdingBox());
    EXPECT_EQ(p.boxes().size(), 1u);
}

TEST(Stacking, ReleaseAwayFromPalletDropsBox) {
    Plant p;
    deliverBox(p, { 1, 120.0, 300.0 });
    Outputs out;
    out.stackerDown = true;   runFor(p, out, 1.5);
    out.gripperClose = true;  runFor(p, out, 0.7);
    ASSERT_TRUE(p.holdingBox());
    out.gripperClose = false; runFor(p, out, 0.7);       // opened while still down
    EXPECT_FALSE(p.holdingBox());
    EXPECT_EQ(p.droppedCount(), 1);
    EXPECT_EQ(p.placedTotal(), 0);
}

TEST(Pallet, ReportsFullAtCapacity) {
    PlantConfig cfg;
    cfg.palletCapacity = 2;
    Plant p(cfg);
    for (int i = 0; i < 2; ++i) {
        EXPECT_FALSE(p.inputs().palletFull);
        deliverBox(p, { 1, 120.0, 300.0 });
        pickAndPlace(p);
    }
    EXPECT_EQ(p.placedOnPallet(), 2);
    EXPECT_TRUE(p.inputs().palletFull);
    EXPECT_TRUE(p.inputs().palletPresent);
}

TEST(Pallet, FullPalletStaysUntilReleased) {
    PlantConfig cfg;
    cfg.palletCapacity = 1;
    Plant p(cfg);
    deliverBox(p, { 1, 120.0, 300.0 });
    pickAndPlace(p);
    ASSERT_TRUE(p.inputs().palletFull);
    runFor(p, Outputs{}, 10.0);
    EXPECT_TRUE(p.inputs().palletFull);
    EXPECT_EQ(p.palletsCompleted(), 0);
}

TEST(Pallet, ReleaseSendsPalletAwayAndNewOneArrives) {
    PlantConfig cfg;
    cfg.palletCapacity = 1;
    Plant p(cfg);
    deliverBox(p, { 1, 120.0, 300.0 });
    pickAndPlace(p);
    ASSERT_TRUE(p.inputs().palletFull);

    Outputs out; out.palletRelease = true;
    runFor(p, out, 0.1);
    EXPECT_FALSE(p.inputs().palletPresent);
    EXPECT_FALSE(p.inputs().palletFull);
    EXPECT_EQ(p.palletsCompleted(), 1);

    runFor(p, out, 3.2);                                 // palletChangeS = 3 s
    EXPECT_TRUE(p.inputs().palletPresent);
    EXPECT_FALSE(p.inputs().palletFull);
    EXPECT_EQ(p.placedOnPallet(), 0);
    EXPECT_EQ(p.palletsCompleted(), 1);                  // release held high does not repeat
}

TEST(Pallet, MissingPalletMeansBoxIsDropped) {
    PlantConfig cfg;
    cfg.startWithPallet = false;
    Plant p(cfg);
    p.blockPalletArrival(true);                          // fault: no pallet ever arrives
    deliverBox(p, { 1, 120.0, 300.0 });
    EXPECT_FALSE(p.inputs().palletPresent);
    pickAndPlace(p);
    EXPECT_EQ(p.droppedCount(), 1);
    EXPECT_EQ(p.placedTotal(), 0);
}

TEST(Pallet, ArrivesOnItsOwnWhenNotBlocked) {
    PlantConfig cfg;
    cfg.startWithPallet = false;
    Plant p(cfg);
    runFor(p, Outputs{}, 3.2);
    EXPECT_TRUE(p.inputs().palletPresent);
}