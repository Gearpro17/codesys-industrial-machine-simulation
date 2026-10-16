#include "cyclic.h"
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

using namespace plant;
using namespace std::chrono_literals;

TEST(Cyclic, RunsTheTaskTheRequestedNumberOfCycles) {
    CyclicExecutive exec{ 2ms };
    int n = 0;
    exec.run([&] { return ++n < 10; });
    EXPECT_EQ(n, 10);
    EXPECT_EQ(exec.stats().cycles, 10u);
}

TEST(Cyclic, DoesNotRunFasterThanItsPeriod) {
    CyclicExecutive exec{ 5ms };
    int n = 0;
    const auto t0 = std::chrono::steady_clock::now();
    exec.run([&] { return ++n < 11; });           // 11 cycles = 10 full periods between first and last
    const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    EXPECT_GE(elapsed, 49.0);                     // lower bound only: an upper bound would be flaky on a busy machine
}

TEST(Cyclic, CountsOverrunsWhenTheTaskIsSlowerThanThePeriod) {
    CyclicExecutive exec{ 1ms };
    int n = 0;
    exec.run([&] {
        std::this_thread::sleep_for(4ms);         // 4 ms of work in a 1 ms period
        return ++n < 5;
        });
    EXPECT_EQ(exec.stats().cycles, 5u);
    EXPECT_EQ(exec.stats().overruns, 4u);         // the last cycle ends the loop before it is judged
}

TEST(Cyclic, ExecutionTimeStatisticsAreConsistent) {
    CyclicExecutive exec{ 1ms };
    int n = 0;
    exec.run([&] {
        std::this_thread::sleep_for(2ms);
        return ++n < 5;
        });
    const CycleStats& s = exec.stats();
    EXPECT_GE(s.execMinUs, 1900.0);
    EXPECT_LE(s.execMinUs, s.execMeanUs);
    EXPECT_LE(s.execMeanUs, s.execMaxUs);
    EXPECT_GE(s.jitterMaxUs, s.jitterMeanUs);
}