#include "plant.h"
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
}

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