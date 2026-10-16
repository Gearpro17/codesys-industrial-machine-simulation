#include "controller.h"

namespace plant {

    std::string_view stateName(State s) {
        switch (s) {
        case State::Stopped:       return "STOPPED";
        case State::Starting:      return "STARTING";
        case State::Running:       return "RUNNING";
        case State::Inspection:    return "INSPECTION";
        case State::Rejecting:     return "REJECTING";
        case State::Transferring:  return "TRANSFERRING";
        case State::Stacking:      return "STACKING";
        case State::PalletFull:    return "PALLET_FULL";
        case State::PalletChange:  return "PALLET_CHANGE";
        case State::Fault:         return "FAULT";
        case State::EmergencyStop: return "EMERGENCY_STOP";
        }
        return "?";
    }

    std::string_view faultName(FaultCode f) {
        switch (f) {
        case FaultCode::None:                  return "NONE";
        case FaultCode::EmergencyStop:         return "EMERGENCY_STOP";
        case FaultCode::StartupCheck:          return "STARTUP_CHECK_FAILED";
        case FaultCode::MotorFault:            return "MOTOR_FAULT";
        case FaultCode::BoxNotAtStation:       return "BOX_NOT_AT_STATION";
        case FaultCode::RejectFailed:          return "REJECT_FAILED";
        case FaultCode::TransferTimeout:       return "TRANSFER_TIMEOUT";
        case FaultCode::MissingPallet:         return "MISSING_PALLET";
        case FaultCode::StackerTimeout:        return "STACKER_TIMEOUT";
        case FaultCode::GripperTimeout:        return "GRIPPER_TIMEOUT";
        case FaultCode::UnexpectedBoxAtPickup: return "UNEXPECTED_BOX_AT_PICKUP";
        case FaultCode::PalletChangeTimeout:   return "PALLET_CHANGE_TIMEOUT";
        }
        return "?";
    }

    SoftController::SoftController(ControllerConfig cfg) : cfg_(cfg) {
        if (!setRecipe(cfg.recipeId)) recipe_ = &defaultRecipes().front();
    }

    bool SoftController::setRecipe(int id) {
        for (const auto& r : defaultRecipes()) {
            if (r.id == id) { recipe_ = &r; return true; }
        }
        return false;
    }

    Outputs SoftController::safeOutputs(bool holdGripper) {
        Outputs o;
        o.gripperClose = holdGripper;   // a gripper that opens on a fault drops its load
        return o;
    }

    void SoftController::enter(State s) {
        state_ = s;
        stateTimeS_ = 0.0;
        stepTimeS_ = 0.0;
        switch (s) {
        case State::Inspection:   samples_ = 0; heightSumDmm_ = 0; break;
        case State::Stacking:     stackStep_ = StackStep::Lower; break;
        case State::PalletChange: palletLeft_ = false; break;
        default: break;
        }
    }

    void SoftController::raiseFault(FaultCode code) {
        if (state_ == State::Fault || state_ == State::EmergencyStop) return;
        fault_ = code;
        enter(State::Fault);
    }

    // Commanded speed but no measured motion: the example from the project brief.
    void SoftController::watchMotor(const Inputs& in, double dt) {
        const bool operating = state_ != State::Stopped && state_ != State::Fault &&
            state_ != State::EmergencyStop;
        if (prevOut_.conveyorMotor && operating) motorOnTimeS_ += dt;
        else                                     motorOnTimeS_ = 0.0;

        if (motorOnTimeS_ > cfg_.motorFaultDelayS &&
            in.conveyorSpeedFbMms < 0.1 * prevOut_.speedSetpointMms) {
            raiseFault(FaultCode::MotorFault);
        }
    }

    void SoftController::runStacking(const Inputs& in) {
        switch (stackStep_) {
        case StackStep::Lower:
            out_.stackerDown = true;
            if (in.stackerDownLimit)                    nextStackStep(StackStep::Grip);
            else if (stepTimeS_ > cfg_.stackerTimeoutS) raiseFault(FaultCode::StackerTimeout);
            break;

        case StackStep::Grip:
            out_.gripperClose = true;
            if (in.gripperClosedFb)                     nextStackStep(StackStep::Lift);
            else if (stepTimeS_ > cfg_.gripperTimeoutS) raiseFault(FaultCode::GripperTimeout);
            break;

        case StackStep::Lift:
            out_.gripperClose = true;
            out_.stackerUp = true;
            if (in.stackerUpLimit)                      nextStackStep(StackStep::Release);
            else if (stepTimeS_ > cfg_.stackerTimeoutS) raiseFault(FaultCode::StackerTimeout);
            break;

        case StackStep::Release:
            out_.gripperClose = false;
            if (!in.gripperClosedFb && stepTimeS_ >= cfg_.releaseDwellS) {
                ++good_;
                pickupValidated_ = false;
                enter(in.palletFull ? State::PalletFull : State::Running);
            }
            else if (stepTimeS_ > cfg_.gripperTimeoutS + cfg_.releaseDwellS) {
                raiseFault(FaultCode::GripperTimeout);
            }
            break;
        }
    }

    void SoftController::runState(const Inputs& in, bool startEdge) {
        switch (state_) {

        case State::Stopped:
            if (startEdge && in.emergencyStop && in.stopButton) enter(State::Starting);
            break;

        case State::Starting:
            if (!in.stopButton) { enter(State::Stopped); break; }
            if (stateTimeS_ >= cfg_.startupCheckS) {
                const bool ok = in.stackerUpLimit && !in.gripperClosedFb && in.palletPresent;
                if (!ok)                raiseFault(FaultCode::StartupCheck);
                else if (in.palletFull) enter(State::PalletFull);
                else                    enter(State::Running);
            }
            break;

        case State::Running:
            out_.conveyorMotor = true;
            out_.speedSetpointMms = cfg_.runSpeedMms;
            if (!in.stopButton) { enter(State::Stopped); break; }      // stop is honoured between boxes
            if (in.pickupSensor) {
                // A box waiting at the stop is only expected after a recovery (fault, stop).
                if (pickupValidated_) enter(State::Stacking);
                else                  raiseFault(FaultCode::UnexpectedBoxAtPickup);
            }
            else if (in.inspectionSensor) {
                enter(State::Inspection);
            }
            break;

        case State::Inspection: {
            // Belt off. Wait until the box has stopped, then average the height over N scans.
            const bool settled = stateTimeS_ >= 0.3 && in.conveyorSpeedFbMms <= cfg_.settleSpeedMms;
            if (!settled) break;
            if (!in.inspectionSensor) { raiseFault(FaultCode::BoxNotAtStation); break; }

            heightSumDmm_ += in.boxHeightDmm;
            if (++samples_ < cfg_.heightSamples) break;

            lastHeightMm_ = heightSumDmm_ / (10.0 * samples_);
            if (heightMatches(*recipe_, lastHeightMm_)) {
                pickupValidated_ = true;
                enter(State::Transferring);
            }
            else {
                enter(State::Rejecting);
            }
            break;
        }

        case State::Rejecting:
            out_.rejectCylinder = stateTimeS_ < cfg_.rejectExtendS;
            if (stateTimeS_ >= cfg_.rejectExtendS + cfg_.rejectRetractS) {
                if (in.inspectionSensor) { raiseFault(FaultCode::RejectFailed); }
                else { ++rejects_; enter(State::Running); }
            }
            break;

        case State::Transferring:
            out_.conveyorMotor = true;
            out_.speedSetpointMms = cfg_.runSpeedMms;
            if (in.pickupSensor) {
                if (!in.palletPresent) raiseFault(FaultCode::MissingPallet);
                else                   enter(State::Stacking);
            }
            else if (stateTimeS_ > cfg_.transferTimeoutS) {
                raiseFault(FaultCode::TransferTimeout);
            }
            break;

        case State::Stacking:
            runStacking(in);
            break;

        case State::PalletFull:
            if (stateTimeS_ >= cfg_.palletFullHoldS) enter(State::PalletChange);
            break;

        case State::PalletChange:
            if (!palletLeft_) {
                out_.palletRelease = true;
                if (!in.palletPresent) { palletLeft_ = true; stepTimeS_ = 0.0; }
                else if (stateTimeS_ > cfg_.palletLeaveTimeoutS) raiseFault(FaultCode::PalletChangeTimeout);
            }
            else {
                if (in.palletPresent) { ++pallets_; enter(State::Running); }
                else if (stepTimeS_ > cfg_.palletArriveTimeoutS) raiseFault(FaultCode::MissingPallet);
            }
            break;

        case State::Fault:
            // Acknowledge = START press after the hold time. The cause is re-checked by STARTING.
            if (startEdge && in.emergencyStop && stateTimeS_ >= cfg_.faultAckHoldS) {
                fault_ = FaultCode::None;
                enter(State::Stopped);
            }
            break;

        case State::EmergencyStop:
            // Needs the E-stop released AND a reset (START press).
            if (startEdge && in.emergencyStop) {
                fault_ = FaultCode::None;
                enter(State::Stopped);
            }
            break;
        }
    }

    // Last layer before the outputs leave the controller.
    void SoftController::applyInterlocks(const Inputs& in) {
        if (out_.stackerUp && out_.stackerDown) { out_.stackerUp = false; out_.stackerDown = false; }
        if (!in.stackerUpLimit) out_.conveyorMotor = false;   // belt never runs with the stacker down
        if (!out_.conveyorMotor) out_.speedSetpointMms = 0;
    }

    Outputs SoftController::update(const Inputs& in, double dt) {
        const bool startEdge = in.startButton && !prevStart_;
        prevStart_ = in.startButton;
        stateTimeS_ += dt;
        stepTimeS_ += dt;
        out_ = Outputs{};

        // The emergency stop beats everything and is checked on every scan.
        if (!in.emergencyStop && state_ != State::EmergencyStop) {
            fault_ = FaultCode::EmergencyStop;
            enter(State::EmergencyStop);
        }
        watchMotor(in, dt);
        runState(in, startEdge);

        // Whatever happened this scan, FAULT, EMERGENCY_STOP and STOPPED drive safe outputs
        // immediately: a state that was left this scan must not leak its outputs for one more scan.
        if (state_ == State::Fault)              out_ = safeOutputs(prevOut_.gripperClose);
        else if (state_ == State::EmergencyStop) out_ = safeOutputs(false);
        else if (state_ == State::Stopped)       out_ = safeOutputs(false);

        applyInterlocks(in);

        heartbeat_ = static_cast<std::int16_t>(heartbeat_ + 1);
        out_.plcState = static_cast<std::int16_t>(state_);
        out_.activeRecipeId = static_cast<std::int16_t>(recipe_->id);
        out_.heartbeat = heartbeat_;

        prevOut_ = out_;
        return out_;
    }

} // namespace plant