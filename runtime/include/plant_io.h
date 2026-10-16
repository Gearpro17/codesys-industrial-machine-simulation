#pragma once
#include <cstdint>

namespace plant {

    // Plant -> PLC (what the PLC reads). Scaled integers, as on the Modbus wire.
    //
    // Wiring convention (document this in docs/io_map.md):
    //   stopButton and emergencyStop are NC (normally closed) contacts:
    //   TRUE = healthy / not pressed, FALSE = pressed OR wire broken.
    //   startButton is a normal NO push-button: TRUE = pressed.
    struct Inputs {
        bool startButton = false;
        bool stopButton = true;    // NC
        bool emergencyStop = true;    // NC
        bool entrySensor = false;
        bool inspectionSensor = false;
        bool pickupSensor = false;   // DI 10: a box is waiting at the pickup stop
        bool palletPresent = false;
        bool stackerUpLimit = false;
        bool stackerDownLimit = false;
        bool gripperClosedFb = false;
        bool palletFull = false;
        std::int16_t boxHeightDmm = 0;   // 0.1 mm
        std::int16_t conveyorSpeedFbMms = 0;   // mm/s
        std::int16_t temperatureDc = 0;   // 0.1 degC
        std::int16_t pressureCbar = 0;   // 0.01 bar
    };

    // PLC -> Plant (what the PLC writes).
    struct Outputs {
        bool conveyorMotor = false;
        bool rejectCylinder = false;
        bool stackerUp = false;
        bool stackerDown = false;
        bool gripperClose = false;
        bool palletRelease = false;   // Coil 5: send the full pallet away
        std::int16_t speedSetpointMms = 0;
        std::int16_t plcState = 0;
        std::int16_t activeRecipeId = 0;
        std::int16_t heartbeat = 0;
    };

} // namespace plant