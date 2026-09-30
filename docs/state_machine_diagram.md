# 1.3 State Machine

```mermaid
stateDiagram-v2
    [*] --> STOPPED

    %% Global transitions
    STOPPED --> EMERGENCY_STOP : E-Stop pressed
    STARTING --> EMERGENCY_STOP : E-Stop pressed
    RUNNING --> EMERGENCY_STOP : E-Stop pressed
    INSPECTION --> EMERGENCY_STOP : E-Stop pressed
    REJECTING --> EMERGENCY_STOP : E-Stop pressed
    TRANSFERRING --> EMERGENCY_STOP : E-Stop pressed
    STACKING --> EMERGENCY_STOP : E-Stop pressed
    PALLET_FULL --> EMERGENCY_STOP : E-Stop pressed
    PALLET_CHANGE --> EMERGENCY_STOP : E-Stop pressed
    STOPPING --> EMERGENCY_STOP : E-Stop pressed
    FAULT --> EMERGENCY_STOP : E-Stop pressed

    STARTING --> FAULT : Fault detected
    RUNNING --> FAULT : Fault detected
    INSPECTION --> FAULT : Fault detected
    REJECTING --> FAULT : Fault detected
    TRANSFERRING --> FAULT : Fault detected
    STACKING --> FAULT : Fault detected
    PALLET_FULL --> FAULT : Fault detected
    PALLET_CHANGE --> FAULT : Fault detected
    STOPPING --> FAULT : Fault detected

    %% Recovery transitions
    EMERGENCY_STOP --> STOPPED : E-Stop released + Reset command

    FAULT --> STOPPED : Fault cleared + Acknowledge

    %% Normal operation
    STOPPED --> STARTING : Start button

    STARTING --> RUNNING : Initialization complete

    RUNNING --> INSPECTION : Box detected at inspection sensor

    INSPECTION --> REJECTING : Height outside recipe tolerance
    INSPECTION --> TRANSFERRING : Height within tolerance

    REJECTING --> RUNNING : Reject cycle complete

    TRANSFERRING --> STACKING : Box reaches stacker

    STACKING --> PALLET_FULL : palletFull = TRUE
    STACKING --> RUNNING : Box stacked successfully

    PALLET_FULL --> PALLET_CHANGE : Operator confirms pallet change

    PALLET_CHANGE --> RUNNING : New pallet present and palletFull = FALSE

    RUNNING --> STOPPING : Stop button
    STOPPING --> STOPPED : Conveyor empty and outputs off
```

## Notes

- emergencyStop is wired NC (normally closed).
- FAULT covers motor faults, stacker timeouts, gripper timeouts, heartbeat loss, and sensor plausibility failures.
- Heartbeat (HR3) is monitored for communication timeout detection.
- Stacker uses separate stackerUp and stackerDown outputs with limit-switch interlocks.
- INSPECTION compares boxHeight against the active recipe.
- Required pallet sequence: STACKING → PALLET_FULL → PALLET_CHANGE → RUNNING.
