#include "cyclic.h"
#include <algorithm>
#include <cmath>
#include <thread>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <timeapi.h>
#pragma comment(lib, "winmm.lib")
#endif

namespace plant {

    namespace {
        // Windows ticks at ~15.6 ms by default, so a 10 ms period cannot be held without this.
        struct TimerResolution {
#ifdef _WIN32
            TimerResolution() { timeBeginPeriod(1); }
            ~TimerResolution() { timeEndPeriod(1); }
#endif
        };
    } // namespace

    CyclicExecutive::CyclicExecutive(std::chrono::microseconds period, std::chrono::microseconds spinMargin)
        : period_(period), spin_(spinMargin) {
    }

    // Sleep for most of the wait, then spin for the last stretch: sleeping is imprecise,
    // spinning is exact but burns CPU, so do each where it is cheap.
    void CyclicExecutive::waitUntil(Clock::time_point t) const {
        const auto coarse = t - spin_;
        if (Clock::now() < coarse) std::this_thread::sleep_until(coarse);
        while (Clock::now() < t) { /* spin */ }
    }

    void CyclicExecutive::run(const std::function<bool()>& task) {
        [[maybe_unused]] TimerResolution timerResolution;
        using Us = std::chrono::duration<double, std::micro>;

        stats_ = {};
        auto scheduled = Clock::now();
        for (;;) {
            const auto start = Clock::now();
            const double jitter = std::abs(Us(start - scheduled).count());

            const bool again = task();

            const auto end = Clock::now();
            const double exec = Us(end - start).count();

            ++stats_.cycles;
            const double n = static_cast<double>(stats_.cycles);
            stats_.execMinUs = std::min(stats_.execMinUs, exec);
            stats_.execMaxUs = std::max(stats_.execMaxUs, exec);
            stats_.execMeanUs += (exec - stats_.execMeanUs) / n;
            stats_.jitterMaxUs = std::max(stats_.jitterMaxUs, jitter);
            stats_.jitterMeanUs += (jitter - stats_.jitterMeanUs) / n;
            if (!again) break;

            scheduled += period_;
            if (end > scheduled) {          // overrun: count it and re-sync, never "catch up"
                ++stats_.overruns;
                scheduled = end;
            }
            waitUntil(scheduled);
        }
    }

} // namespace plant