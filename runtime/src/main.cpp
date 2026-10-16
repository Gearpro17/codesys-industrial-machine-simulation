// Closed-loop demo: SoftController (the "PLC") + Plant (the machine), one scan every 10 ms.
//
//   runtime.exe                    fast mode: runs in simulated time, as quickly as possible
//   runtime.exe --realtime         paced by the CyclicExecutive, prints cycle-time statistics
//   runtime.exe --seconds 120      simulated-time limit (default 600)
#include "controller.h"
#include "cyclic.h"
#include "plant.h"
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string_view>

int main(int argc, char** argv) {
    bool   realtime = false;
    double maxSeconds = 600.0;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--realtime") realtime = true;
        else if (arg == "--seconds" && i + 1 < argc) maxSeconds = std::atof(argv[++i]);
    }

    plant::PlantConfig cfg;
    cfg.feeder.enabled = true;
    cfg.feeder.minGapMm = 800.0;      // gap between boxes must exceed the inspection-to-pickup distance
    cfg.palletCapacity = 4;
    plant::Plant plant(cfg, 1);
    plant::SoftController controller;

    constexpr double dt = 0.010;      // 10 ms scan
    double simTime = 0.0;
    plant::State lastState = controller.state();
    std::cout << std::fixed << std::setprecision(2);

    // One scan: read inputs -> run the control logic -> update the plant.
    // With the real PLC, "read inputs / write outputs" become the Modbus exchange.
    const auto scan = [&]() -> bool {
        const plant::Inputs  in = plant.inputs();
        const plant::Outputs out = controller.update(in, dt);
        plant.step(dt, out);
        simTime += dt;

        plant.setPanel({ simTime >= 1.0 && simTime < 1.2, false, false });   // operator presses START

        if (controller.state() != lastState) {
            lastState = controller.state();
            std::cout << "[" << std::setw(7) << simTime << " s] " << plant::stateName(lastState);
            if (lastState == plant::State::Rejecting || lastState == plant::State::Transferring)
                std::cout << "   (measured height " << controller.lastHeightMm() << " mm, recipe "
                << controller.recipe().name << " expects " << controller.recipe().expectedHeightMm << ")";
            if (lastState == plant::State::Fault)
                std::cout << "   cause: " << plant::faultName(controller.fault());
            std::cout << "\n";
        }
        return simTime < maxSeconds && controller.palletsCompleted() < 2 &&
            controller.state() != plant::State::Fault;
        };

    std::cout << "Palletizer runtime, " << (realtime ? "real-time" : "fast") << " mode, 10 ms scan\n";

    if (realtime) {
        plant::CyclicExecutive executive{ std::chrono::milliseconds(10) };
        executive.run(scan);
        const auto& s = executive.stats();
        std::cout << "\nCycle statistics (" << s.cycles << " cycles, period 10000 us)\n"
            << "  execution time  min/mean/max: " << s.execMinUs << " / " << s.execMeanUs
            << " / " << s.execMaxUs << " us\n"
            << "  start jitter    mean/max:      " << s.jitterMeanUs << " / " << s.jitterMaxUs << " us\n"
            << "  overruns: " << s.overruns << "\n";
    }
    else {
        while (scan()) {}
    }

    std::cout << "\nSummary after " << simTime << " s simulated\n"
        << "  boxes fed       : " << plant.spawnedCount() << "\n"
        << "  stacked (good)  : " << controller.goodCount() << "\n"
        << "  rejected        : " << controller.rejectCount() << "\n"
        << "  pallets done    : " << controller.palletsCompleted() << "\n"
        << "  boxes dropped   : " << plant.droppedCount() << "\n";
    return controller.state() == plant::State::Fault ? 1 : 0;
}