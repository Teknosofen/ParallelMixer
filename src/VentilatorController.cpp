#include "VentilatorController.hpp"
#include "SerialMuxRouter.hpp"

// ============================================================================
// Constructor
// ============================================================================
VentilatorController::VentilatorController()
    : _muxRouter(nullptr),
      _running(false),
      _alarmFlags(0),
      _cycleTime_us(0),
      _inspTotalTime_us(0),
      _insp1Time_us(0),
      _insp2Time_us(0),
      _inspPauseTime_us(0),
      _expTotalTime_us(0),
      _expNonTrigTime_us(0),
      _expSyncTime_us(0),
      _cycleStartTime_us(0),
      _stateStartTime_us(0),
      _lastBreathTime_us(0),
      _breathPeakPressure(0),
      _breathPlateauPressure(0),
      _breathVolumeDelivered_mL(0),
      _breathWasTriggered(false),
      _targetTotalFlow_slm(0),
      _currentSetFlow_slm(0),
      _targetO2Flow_slm(0),
      _targetAirFlow_slm(0)
{
    setDefaultConfig();
    memset(&_status, 0, sizeof(_status));
    memset(&_outputs, 0, sizeof(_outputs));
    _status.state = VENT_OFF;
}

// ============================================================================
// Lifecycle
// ============================================================================
void VentilatorController::begin(SerialMuxRouter* muxRouter) {
    _muxRouter = muxRouter;
}

void VentilatorController::start() {
    if (_running) return;

    recalculateTiming();
    _running = true;
    _alarmFlags = 0;
    _status.breathCount = 0;

    // Start first breath
    startNewBreath(micros());
}

void VentilatorController::stop() {
    _running = false;
    _status.state = VENT_OFF;

    // Zero all outputs
    _outputs.airValveFlow_slm = 0;
    _outputs.o2ValveFlow_slm = 0;
    _outputs.expValveSetpoint = 0;
    _outputs.expValveAsPressure = false;
    _outputs.expValveClosed = false;
}

// ============================================================================
// Default Configuration
// ============================================================================
void VentilatorController::setDefaultConfig() {
    _config.respRate = 12.0f;
    _config.ieRatio = 0.5f;
    _config.inspPauseFraction = 0.1f;
    _config.insp1Fraction = 1.0f;
    _config.insp2Fraction = 0.0f;
    _config.expNonTrigFraction = 0.5f;
    _config.expSyncFraction = 0.0f;

    _config.tidalVolume_mL = 500.0f;
    _config.maxInspFlow_slm = 60.0f;
    _config.totalFlow_slm = 30.0f;
    _config.useVolumeControl = true;

    _config.maxPressure_mbar = 30.0f;
    _config.peep_mbar = 5.0f;
    _config.pressureRampTime_ms = 100.0f;

    _config.targetFiO2 = 0.21f;

    _config.biasFlow_slm = 2.0f;
    _config.flowTrigger_slm = 2.0f;
    _config.pressureTrigger_mbar = 2.0f;
    _config.triggerEnabled = false;

    _config.highPressureAlarm_mbar = 40.0f;
    _config.lowPressureAlarm_mbar = 3.0f;
    _config.apneaTime_s = 20.0f;
}

void VentilatorController::setConfig(const VentilatorConfig& config) {
    _config = config;
    if (_running) {
        recalculateTiming();
    }
}

// ============================================================================
// Individual Parameter Setters
// ============================================================================
void VentilatorController::setRespRate(float bpm) {
    _config.respRate = constrain(bpm, 4.0f, 60.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setIERatio(float ratio) {
    _config.ieRatio = constrain(ratio, 0.1f, 4.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setTidalVolume(float mL) {
    _config.tidalVolume_mL = constrain(mL, 50.0f, 2000.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setMaxFlow(float slm) {
    _config.maxInspFlow_slm = constrain(slm, 5.0f, 120.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setTotalFlow(float slm) {
    _config.totalFlow_slm = constrain(slm, 1.0f, 120.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setUseVolumeControl(bool use) {
    _config.useVolumeControl = use;
    if (_running) recalculateTiming();
}

void VentilatorController::setMaxPressure(float mbar) {
    _config.maxPressure_mbar = constrain(mbar, 5.0f, 80.0f);
}

void VentilatorController::setPEEP(float mbar) {
    _config.peep_mbar = constrain(mbar, 0.0f, 30.0f);
}

void VentilatorController::setPressureRampTime(float ms) {
    _config.pressureRampTime_ms = constrain(ms, 0.0f, 500.0f);
}

void VentilatorController::setFiO2(float fraction) {
    _config.targetFiO2 = constrain(fraction, 0.21f, 1.0f);
}

void VentilatorController::setInspPauseFraction(float frac) {
    _config.inspPauseFraction = constrain(frac, 0.0f, 0.5f);
    if (_running) recalculateTiming();
}

void VentilatorController::setInsp1Fraction(float frac) {
    _config.insp1Fraction = constrain(frac, 0.0f, 1.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setInsp2Fraction(float frac) {
    _config.insp2Fraction = constrain(frac, 0.0f, 1.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setExpNonTrigFraction(float frac) {
    _config.expNonTrigFraction = constrain(frac, 0.0f, 1.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setExpSyncFraction(float frac) {
    _config.expSyncFraction = constrain(frac, 0.0f, 1.0f);
    if (_running) recalculateTiming();
}

void VentilatorController::setBiasFlow(float slm) {
    _config.biasFlow_slm = constrain(slm, 0.0f, 20.0f);
}

void VentilatorController::setFlowTrigger(float slm) {
    _config.flowTrigger_slm = constrain(slm, 0.5f, 20.0f);
}

void VentilatorController::setPressureTrigger(float mbar) {
    _config.pressureTrigger_mbar = constrain(mbar, 0.5f, 10.0f);
}

void VentilatorController::setTriggerEnabled(bool enabled) {
    _config.triggerEnabled = enabled;
}

// ============================================================================
// State String
// ============================================================================
const char* VentilatorController::getStateString() const {
    switch (_status.state) {
        case VENT_OFF:          return "OFF";
        case VENT_INSP_PHASE1:  return "INSP1";
        case VENT_INSP_PHASE2:  return "INSP2";
        case VENT_INSP_PAUSE:   return "PAUSE";
        case VENT_EXP_NON_TRIG: return "EXP";
        case VENT_EXP_SYNC_WAIT: return "SYNC";
        default:                return "?";
    }
}

// ============================================================================
// Timing Calculations
// ============================================================================
void VentilatorController::recalculateTiming() {
    // Total cycle time from respiratory rate
    _cycleTime_us = (uint32_t)((60.0f / _config.respRate) * 1000000.0f);

    // Split by I:E ratio: Ti / (Ti + Te) = ratio / (1 + ratio)
    float ieFactor = _config.ieRatio / (1.0f + _config.ieRatio);
    _inspTotalTime_us = (uint32_t)(_cycleTime_us * ieFactor);
    _expTotalTime_us = _cycleTime_us - _inspTotalTime_us;

    // Inspiration breakdown
    _inspPauseTime_us = (uint32_t)(_inspTotalTime_us * _config.inspPauseFraction);
    uint32_t activeInspTime_us = _inspTotalTime_us - _inspPauseTime_us;

    // Normalize fractions if they don't sum to 1
    float totalInspFrac = _config.insp1Fraction + _config.insp2Fraction;
    if (totalInspFrac > 0.01f) {
        _insp1Time_us = (uint32_t)(activeInspTime_us * (_config.insp1Fraction / totalInspFrac));
        _insp2Time_us = activeInspTime_us - _insp1Time_us;
    } else {
        _insp1Time_us = activeInspTime_us;
        _insp2Time_us = 0;
    }

    // Expiration breakdown
    float totalExpFrac = _config.expNonTrigFraction + _config.expSyncFraction;
    if (totalExpFrac > 0.01f) {
        _expNonTrigTime_us = (uint32_t)(_expTotalTime_us * (_config.expNonTrigFraction / totalExpFrac));
        _expSyncTime_us = _expTotalTime_us - _expNonTrigTime_us;
    } else {
        _expNonTrigTime_us = _expTotalTime_us;
        _expSyncTime_us = 0;
    }

    // Calculate target flow
    if (_config.useVolumeControl) {
        _targetTotalFlow_slm = calculateFlowFromVt();
    } else {
        _targetTotalFlow_slm = _config.totalFlow_slm;
    }

    // Clamp to max flow
    if (_targetTotalFlow_slm > _config.maxInspFlow_slm) {
        _targetTotalFlow_slm = _config.maxInspFlow_slm;
        _status.flowLimited = true;
    } else {
        _status.flowLimited = false;
    }
}

float VentilatorController::calculateFlowFromVt() const {
    // Active inspiration time in minutes
    float activeInspTime_min = (_insp1Time_us + _insp2Time_us) / 60000000.0f;

    if (activeInspTime_min <= 0.001f) return _config.totalFlow_slm;

    // Flow (L/min) = Volume (L) / Time (min)
    float requiredFlow = (_config.tidalVolume_mL / 1000.0f) / activeInspTime_min;

    return requiredFlow;
}

void VentilatorController::calculateGasFlows(float totalFlow) {
    // Air = 21% O2, Pure O2 = 100%
    float fiO2 = constrain(_config.targetFiO2, 0.21f, 1.0f);

    if (fiO2 <= 0.21f) {
        // Room air only
        _targetO2Flow_slm = 0;
        _targetAirFlow_slm = totalFlow;
    } else if (fiO2 >= 1.0f) {
        // Pure O2
        _targetO2Flow_slm = totalFlow;
        _targetAirFlow_slm = 0;
    } else {
        // Mix: solve O2 + Air = Total, O2*1.0 + Air*0.21 = Total*FiO2
        _targetO2Flow_slm = totalFlow * (fiO2 - 0.21f) / 0.79f;
        _targetAirFlow_slm = totalFlow - _targetO2Flow_slm;
    }

    // Clamp to non-negative
    if (_targetO2Flow_slm < 0) _targetO2Flow_slm = 0;
    if (_targetAirFlow_slm < 0) _targetAirFlow_slm = 0;
}

// ============================================================================
// Main Update Loop
// ============================================================================
void VentilatorController::update(const VentilatorMeasurements& meas, uint32_t currentTime_us) {
    if (!_running) return;

    // Track peak pressure during breath
    if (meas.airwayPressure_mbar > _breathPeakPressure) {
        _breathPeakPressure = meas.airwayPressure_mbar;
    }

    // Accumulate volume (simple integration - could be improved)
    // Volume = flow * time, flow in L/min, need mL
    // For now, just estimate based on set flow

    // Calculate cycle progress
    uint32_t cycleElapsed = currentTime_us - _cycleStartTime_us;
    _status.cycleProgress = (float)cycleElapsed / (float)_cycleTime_us;
    if (_status.cycleProgress > 1.0f) _status.cycleProgress = 1.0f;

    // State machine
    switch (_status.state) {
        case VENT_INSP_PHASE1:
            executeInspPhase1(meas);
            if (checkStateTimeout(currentTime_us, _insp1Time_us)) {
                if (_insp2Time_us > 10000) {  // > 10ms
                    transitionTo(VENT_INSP_PHASE2, currentTime_us);
                } else if (_inspPauseTime_us > 10000) {
                    transitionTo(VENT_INSP_PAUSE, currentTime_us);
                } else {
                    transitionTo(VENT_EXP_NON_TRIG, currentTime_us);
                }
            }
            break;

        case VENT_INSP_PHASE2:
            executeInspPhase2(meas);
            if (checkStateTimeout(currentTime_us, _insp2Time_us)) {
                if (_inspPauseTime_us > 10000) {
                    transitionTo(VENT_INSP_PAUSE, currentTime_us);
                } else {
                    transitionTo(VENT_EXP_NON_TRIG, currentTime_us);
                }
            }
            break;

        case VENT_INSP_PAUSE:
            executeInspPause(meas);
            if (checkStateTimeout(currentTime_us, _inspPauseTime_us)) {
                _breathPlateauPressure = meas.airwayPressure_mbar;
                transitionTo(VENT_EXP_NON_TRIG, currentTime_us);
            }
            break;

        case VENT_EXP_NON_TRIG:
            executeExpNonTrig(meas);
            if (checkStateTimeout(currentTime_us, _expNonTrigTime_us)) {
                if (_expSyncTime_us > 10000 && _config.triggerEnabled) {
                    transitionTo(VENT_EXP_SYNC_WAIT, currentTime_us);
                } else {
                    startNewBreath(currentTime_us);
                }
            }
            break;

        case VENT_EXP_SYNC_WAIT:
            executeExpSyncWait(meas);
            if (checkTrigger(meas)) {
                _breathWasTriggered = true;
                startNewBreath(currentTime_us);
            } else if (checkStateTimeout(currentTime_us, _expSyncTime_us)) {
                startNewBreath(currentTime_us);
            }
            break;

        case VENT_OFF:
        default:
            break;
    }

    // Update alarms
    updateAlarms(meas, currentTime_us);

    // Update status
    _status.calculatedFlow_slm = _targetTotalFlow_slm;
    _status.actualSetFlow_slm = _currentSetFlow_slm;
    _status.peakPressure_mbar = _breathPeakPressure;
    _status.plateauPressure_mbar = _breathPlateauPressure;
    _status.triggered = _breathWasTriggered;
}

// ============================================================================
// State Transitions
// ============================================================================
void VentilatorController::startNewBreath(uint32_t currentTime_us) {
    // Store breath measurements before reset
    _status.measuredVt_mL = _breathVolumeDelivered_mL;

    // Increment breath count
    _status.breathCount++;
    _lastBreathTime_us = currentTime_us;

    // Reset for new breath
    resetBreathMeasurements();
    _cycleStartTime_us = currentTime_us;

    // Start with INSP_PHASE1
    transitionTo(VENT_INSP_PHASE1, currentTime_us);
}

void VentilatorController::transitionTo(VentilatorState newState, uint32_t currentTime_us) {
    _status.state = newState;
    _stateStartTime_us = currentTime_us;
}

bool VentilatorController::checkStateTimeout(uint32_t currentTime_us, uint32_t duration_us) const {
    return (currentTime_us - _stateStartTime_us) >= duration_us;
}

bool VentilatorController::checkTrigger(const VentilatorMeasurements& meas) const {
    if (!_config.triggerEnabled) return false;

    // Flow trigger: inspiratory flow exceeds bias + threshold
    if (meas.inspFlow_slm > (_config.biasFlow_slm + _config.flowTrigger_slm)) {
        return true;
    }

    // Pressure trigger: pressure drops below PEEP - threshold
    if (meas.airwayPressure_mbar < (_config.peep_mbar - _config.pressureTrigger_mbar)) {
        return true;
    }

    return false;
}

void VentilatorController::resetBreathMeasurements() {
    _breathPeakPressure = 0;
    _breathPlateauPressure = 0;
    _breathVolumeDelivered_mL = 0;
    _breathWasTriggered = false;
    _status.pressureLimited = false;
}

// ============================================================================
// State Execution
// ============================================================================
void VentilatorController::executeInspPhase1(const VentilatorMeasurements& meas) {
    // INSP1: Exp valve CLOSED, flow controls pressure
    float setFlow = _targetTotalFlow_slm;

    // Pressure limiting: reduce flow if pressure exceeds limit
    if (meas.airwayPressure_mbar > _config.maxPressure_mbar) {
        float pressureError = meas.airwayPressure_mbar - _config.maxPressure_mbar;
        setFlow -= pressureError * 2.0f;  // Proportional reduction
        if (setFlow < 0) setFlow = 0;
        _status.pressureLimited = true;
    }

    _currentSetFlow_slm = setFlow;
    calculateGasFlows(setFlow);

    // Set outputs
    _outputs.airValveFlow_slm = _targetAirFlow_slm;
    _outputs.o2ValveFlow_slm = _targetO2Flow_slm;
    _outputs.expValveClosed = true;
    _outputs.expValveAsPressure = false;
    _outputs.expValveSetpoint = 0;
}

void VentilatorController::executeInspPhase2(const VentilatorMeasurements& meas) {
    // INSP2: Exp valve as POP-OFF for pressure limiting
    float setFlow = _targetTotalFlow_slm;

    _currentSetFlow_slm = setFlow;
    calculateGasFlows(setFlow);

    _outputs.airValveFlow_slm = _targetAirFlow_slm;
    _outputs.o2ValveFlow_slm = _targetO2Flow_slm;

    // Exp valve acts as pressure relief (pop-off) at maxPressure
    _outputs.expValveClosed = false;
    _outputs.expValveAsPressure = true;
    _outputs.expValveSetpoint = _config.maxPressure_mbar;

    _status.pressureLimited = (meas.airwayPressure_mbar >= _config.maxPressure_mbar - 1);
}

void VentilatorController::executeInspPause(const VentilatorMeasurements& meas) {
    // PAUSE: No flow, valves hold
    _currentSetFlow_slm = 0;

    _outputs.airValveFlow_slm = 0;
    _outputs.o2ValveFlow_slm = 0;
    _outputs.expValveClosed = true;
    _outputs.expValveAsPressure = false;
    _outputs.expValveSetpoint = 0;

    _status.pressureLimited = false;
}

void VentilatorController::executeExpNonTrig(const VentilatorMeasurements& meas) {
    // EXP_NON_TRIG: Passive expiration, maintain PEEP, NO triggering

    // Bias flow during expiration (mixed at current FiO2)
    calculateGasFlows(_config.biasFlow_slm);

    _outputs.airValveFlow_slm = _targetAirFlow_slm;
    _outputs.o2ValveFlow_slm = _targetO2Flow_slm;

    // Exp valve controls PEEP
    _outputs.expValveClosed = false;
    _outputs.expValveAsPressure = true;
    _outputs.expValveSetpoint = _config.peep_mbar;

    _currentSetFlow_slm = _config.biasFlow_slm;
}

void VentilatorController::executeExpSyncWait(const VentilatorMeasurements& meas) {
    // EXP_SYNC_WAIT: Same as non-trig, trigger detection in main update
    executeExpNonTrig(meas);
}

// ============================================================================
// Alarms
// ============================================================================
void VentilatorController::updateAlarms(const VentilatorMeasurements& meas, uint32_t currentTime_us) {
    // High pressure alarm
    if (meas.airwayPressure_mbar > _config.highPressureAlarm_mbar) {
        _alarmFlags |= ALARM_HIGH_PRESSURE;
    }

    // Low pressure alarm (during inspiration)
    if (_status.state == VENT_INSP_PHASE1 || _status.state == VENT_INSP_PHASE2) {
        if (meas.airwayPressure_mbar < _config.lowPressureAlarm_mbar &&
            (currentTime_us - _stateStartTime_us) > 500000) {  // After 500ms
            _alarmFlags |= ALARM_LOW_PRESSURE;
        }
    }

    // Apnea alarm
    if ((currentTime_us - _lastBreathTime_us) > (_config.apneaTime_s * 1000000)) {
        _alarmFlags |= ALARM_APNEA;
    }

    // Low Vt alarm (check at end of inspiration)
    // Could be added when actual volume measurement is available
}
