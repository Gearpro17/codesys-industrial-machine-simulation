#include "plant.h"
#include <iomanip>
#include <iostream>

int main() {
    plant::Plant p;
    plant::Outputs out;
    out.conveyorMotor = true;
    out.speedSetpointMms = 500;
    p.spawnBox({ 1, 120.0, 300.0 });

    constexpr double dt = 0.010;     // 10 ms
    bool stopped = false;
    std::cout << std::fixed << std::setprecision(2);

    for (int i = 0; i < 400; ++i) {
        p.step(dt, out);
        const auto& in = p.inputs();

        if (!stopped && in.inspectionSensor) {
            out.conveyorMotor = false;
            stopped = true;
            std::cout << "--- inspection sensor tripped, motor off ---\n";
        }
        if (i % 10 == 0) {
            std::cout << "t=" << i * dt
                << "  v=" << in.conveyorSpeedFbMms
                << "  front=" << p.boxes()[0].frontMm
                << "  entry=" << in.entrySensor
                << "  insp=" << in.inspectionSensor
                << "  h=" << in.boxHeightDmm / 10.0 << "\n";
        }
    }
}