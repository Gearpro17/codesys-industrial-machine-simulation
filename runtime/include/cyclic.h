#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>

namespace plant {

    struct CycleStats {
        std::uint64_t cycles = 0;
        std::uint64_t overruns = 0;       // cycles whose work took longer than the period
        double execMinUs = std::numeric_limits<double>::max();   // time spent in the task
        double execMaxUs = 0.0;
        double execMeanUs = 0.0;
        double jitterMaxUs = 0.0;        // |actual start - ideal start|, worst case
        double jitterMeanUs = 0.0;
    };

    // Runs a task at a fixed period, the way a PLC runs its scan cycle.
    // Deadlines are absolute (previous deadline + period), so a late cycle does not
    // shift the whole schedule. This is a soft real-time loop: it measures timing,
    // it does not guarantee it.
    class CyclicExecutive {
    public:
        using Clock = std::chrono::steady_clock;

        explicit CyclicExecutive(std::chrono::microseconds period,
            std::chrono::microseconds spinMargin = std::chrono::microseconds{ 1500 });

        // Calls task() once per period until it returns false.
        void run(const std::function<bool()>& task);

        const CycleStats& stats() const { return stats_; }
        std::chrono::microseconds period() const { return period_; }

    private:
        void waitUntil(Clock::time_point t) const;

        std::chrono::microseconds period_;
        std::chrono::microseconds spin_;
        CycleStats stats_;
    };

} // namespace plant