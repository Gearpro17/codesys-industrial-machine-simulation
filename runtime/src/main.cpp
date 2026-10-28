// Closed-loop demo: SoftController (the "PLC") + SpeedLoop (PID) + Plant (the machine), one scan every 10 ms.
//
//   runtime.exe                    fast mode: runs in simulated time, as quickly as possible
//   runtime.exe --realtime         paced by the CyclicExecutive, prints cycle-time statistics
//   runtime.exe --seconds 120      simulated-time limit (default 600)
//   runtime.exe --open-loop        physical motor, ramp only: no PID feedback (compare the speed)
//   runtime.exe --ideal-motor      the Step 3 motor with its built-in controller (no SpeedLoop)
//   runtime.exe --csv run.csv      write one row per scan: time, state, setpoints, speed, drive, ...
#include "controller.h"
#include "cyclic.h"
#include "plant.h"
#include "speed_loop.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>

int main(int argc, char** argv) {
    bool        realtime = false;
    bool        openLoop = false;
    bool        idealMotor = false;
    double      maxSeconds = 600.0;
    std::string csvPath;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--realtime") realtime = true;
        else if (arg == "--open-loop") openLoop = true;
        else if (arg == "--ideal-motor") idealMotor = true;
        else if (arg == "--seconds" && i + 1 < argc) maxSeconds = std::atof(argv[++i]);
        else if (arg == "--csv" && i + 1 < argc) csvPath = argv[++i];
    }
    if (idealMotor && openLoop) {
        std::cerr << "--ideal-motor and --open-loop cannot be combined (the ideal motor has no drive input)\n";
        return 2;
    }

    plant::PlantConfig cfg;
    cfg.feeder.enabled = true;
    cfg.feeder.minGapMm = 800.0;      // gap between boxes must exceed the inspection-to-pickup distance
    cfg.palletCapacity = 4;
    cfg.motorMode = idealMotor ? plant::MotorMode::Ideal : plant::MotorMode::Physical;
    plant::Plant plant(cfg, 1);
    plant::SoftController controller;
    plant::SpeedLoopConfig loopCfg;
    loopCfg.openLoop = openLoop;
    plant::SpeedLoop speedLoop(loopCfg);

    std::ofstream csv;
    if (!csvPath.empty()) {
        csv.open(csvPath);
        if (!csv) { std::cerr << "cannot open " << csvPath << " for writing\n"; return 2; }
        csv << "time_s,state,motor_cmd,setpoint_mms,ramp_mms,speed_mms,speed_fb_mms,drive_pct,boxes_on_belt\n";
    }

    constexpr double dt = 0.010;      // 10 ms scan
    double simTime = 0.0;
    plant::State lastState = controller.state();
    std::cout << std::fixed << std::setprecision(2);

    // Belt speed statistics while the PLC has been asking for motion for a while (settled).
    double motorOnS = 0.0, settledSumMms = 0.0, settledMinMms = 1e9;
    long   settledScans = 0;

    // One scan: read inputs -> run the control logic -> speed loop -> update the plant.
    // With the real PLC, "read inputs / write outputs" become the Modbus exchange.
    const auto scan = [&]() -> bool {
        const plant::Inputs  in = plant.inputs();
        const plant::Outputs out = controller.update(in, dt);
        if (!idealMotor) plant.setDriveCommand(speedLoop.update(dt, out, in));
        plant.step(dt, out);
        simTime += dt;

        plant.setPanel({ simTime >= 1.0 && simTime < 1.2, false, false });   // operator presses START

        const double speed = plant.beltSpeedMms();
        motorOnS = out.conveyorMotor ? motorOnS + dt : 0.0;
        if (motorOnS > 2.0) {
            settledSumMms += speed; ++settledScans;
            settledMinMms = std::min(settledMinMms, speed);
        }
        if (csv) {
            csv << simTime << ',' << plant::stateName(controller.state()) << ',' << out.conveyorMotor << ','
                << out.speedSetpointMms << ',' << (idealMotor ? double(out.speedSetpointMms) : speedLoop.rampedSetpointMms())
                << ',' << speed << ',' << in.conveyorSpeedFbMms << ','
                << (idealMotor ? 0.0 : speedLoop.driveCommandPct()) << ',' << plant.boxesOnBelt() << '\n';
        }

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

    const char* mode = idealMotor ? "ideal motor (no PID)" : openLoop ? "physical motor, OPEN LOOP" : "physical motor, PID speed loop";
    std::cout << "Palletizer runtime, " << (realtime ? "real-time" : "fast") << " mode, 10 ms scan, " << mode << "\n";

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
    if (settledScans > 0)
        std::cout << "  belt speed while running (mm/s): mean " << settledSumMms / settledScans
        << ", min " << settledMinMms << "\n";
    return controller.state() == plant::State::Fault ? 1 : 0;
}
