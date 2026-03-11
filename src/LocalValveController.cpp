#include "LocalValveController.hpp"
#include "SerialMuxRouter.hpp"
#include "ActuatorControl.hpp"

// ============================================================================
// PiecewiseLinearTable
// ============================================================================

PiecewiseLinearTable::PiecewiseLinearTable() : _count(0) {
    memset(_points, 0, sizeof(_points));
}

void PiecewiseLinearTable::load(const LookupPoint* points, uint8_t count) {
    _count = min(count, (uint8_t)MAX_POINTS);
    for (uint8_t i = 0; i < _count; i++) {
        _points[i] = points[i];
    }
}

float PiecewiseLinearTable::lookup(float x) const {
    if (_count == 0) return 0.0f;
    if (_count == 1) return _points[0].y;

    // Clamp to table range
    if (x <= _points[0].x) return _points[0].y;
    if (x >= _points[_count - 1].x) return _points[_count - 1].y;

    // Find segment and interpolate
    for (uint8_t i = 0; i < _count - 1; i++) {
        if (x >= _points[i].x && x <= _points[i + 1].x) {
            float dx = _points[i + 1].x - _points[i].x;
            if (dx < 1e-6f) return _points[i].y;
            float t = (x - _points[i].x) / dx;
            return _points[i].y + t * (_points[i + 1].y - _points[i].y);
        }
    }
    return _points[_count - 1].y;
}

float PiecewiseLinearTable::inverseLookup(float y) const {
    if (_count == 0) return 0.0f;
    if (_count == 1) return _points[0].x;

    // Determine if table is ascending or descending in y
    bool ascending = (_points[_count - 1].y >= _points[0].y);

    if (ascending) {
        if (y <= _points[0].y) return _points[0].x;
        if (y >= _points[_count - 1].y) return _points[_count - 1].x;
    } else {
        if (y >= _points[0].y) return _points[0].x;
        if (y <= _points[_count - 1].y) return _points[_count - 1].x;
    }

    // Find segment where y falls between consecutive points
    for (uint8_t i = 0; i < _count - 1; i++) {
        float y0 = _points[i].y;
        float y1 = _points[i + 1].y;
        bool inRange = ascending ? (y >= y0 && y <= y1) : (y <= y0 && y >= y1);
        if (inRange) {
            float dy = y1 - y0;
            if (fabsf(dy) < 1e-6f) return _points[i].x;
            float t = (y - y0) / dy;
            return _points[i].x + t * (_points[i + 1].x - _points[i].x);
        }
    }
    return _points[_count - 1].x;
}


// ============================================================================
// PressureBandedTable — 2D flow surface with pressure interpolation
// ============================================================================

PressureBandedTable::PressureBandedTable() : _bandCount(0) {
    memset(_refPressure, 0, sizeof(_refPressure));
}

void PressureBandedTable::loadBand(uint8_t bandIndex, float refPressure,
                                    const LookupPoint* points, uint8_t count) {
    if (bandIndex >= MAX_BANDS) return;
    _flowTables[bandIndex].load(points, count);
    _refPressure[bandIndex] = refPressure;
    if (bandIndex >= _bandCount) _bandCount = bandIndex + 1;
}

void PressureBandedTable::findBrackets(float pressure,
                                        uint8_t& lo, uint8_t& hi, float& t) const {
    if (_bandCount <= 1) {
        lo = hi = 0;
        t = 0.0f;
        return;
    }

    // Below lowest band → extrapolate using bands 0 and 1
    if (pressure <= _refPressure[0]) {
        lo = 0;
        hi = 1;
        float dp = _refPressure[1] - _refPressure[0];
        t = (dp > 1e-3f) ? (pressure - _refPressure[0]) / dp : 0.0f;
        return;
    }

    // Above highest band → extrapolate using last two bands
    if (pressure >= _refPressure[_bandCount - 1]) {
        lo = _bandCount - 2;
        hi = _bandCount - 1;
        float dp = _refPressure[hi] - _refPressure[lo];
        t = (dp > 1e-3f) ? (pressure - _refPressure[lo]) / dp : 1.0f;
        return;
    }

    // Find bracketing bands
    for (uint8_t i = 0; i < _bandCount - 1; i++) {
        if (pressure >= _refPressure[i] && pressure <= _refPressure[i + 1]) {
            lo = i;
            hi = i + 1;
            float dp = _refPressure[hi] - _refPressure[lo];
            t = (dp > 1e-3f) ? (pressure - _refPressure[lo]) / dp : 0.0f;
            return;
        }
    }

    // Fallback (should not reach here)
    lo = hi = _bandCount - 1;
    t = 0.0f;
}

float PressureBandedTable::lookup(float valvePercent, float supplyPressure) const {
    if (_bandCount == 0) return 0.0f;
    if (_bandCount == 1) return _flowTables[0].lookup(valvePercent);

    uint8_t lo, hi;
    float t;
    findBrackets(supplyPressure, lo, hi, t);

    float flowLo = _flowTables[lo].lookup(valvePercent);
    float flowHi = _flowTables[hi].lookup(valvePercent);
    return flowLo + t * (flowHi - flowLo);
}

float PressureBandedTable::inverseLookup(float desiredFlow, float supplyPressure) const {
    if (_bandCount == 0) return 0.0f;
    if (_bandCount == 1) return _flowTables[0].inverseLookup(desiredFlow);

    uint8_t lo, hi;
    float t;
    findBrackets(supplyPressure, lo, hi, t);

    // Inverse lookup on each band: desired flow → V%
    float vpctLo = _flowTables[lo].inverseLookup(desiredFlow);
    float vpctHi = _flowTables[hi].inverseLookup(desiredFlow);

    // Interpolate V% between bands
    return vpctLo + t * (vpctHi - vpctLo);
}


// ============================================================================
// PIController
// ============================================================================

PIController::PIController() {
    _config = {0, 0, -1e6f, 1e6f, 0};
    reset();
}

void PIController::configure(const PIConfig& config) {
    _config = config;
}

void PIController::reset() {
    _state.integrator = 0.0f;
    _state.lastOutput = 0.0f;
    _state.error = 0.0f;
    _state.isActive = false;
}

float PIController::update(float setpoint, float measurement, float dt) {
    _state.error = setpoint - measurement;

    // Integrate
    _state.integrator += _config.ki * _state.error * dt;

    // Clamp integrator to output bounds (basic anti-windup)
    _state.integrator = constrain(_state.integrator, _config.outputMin, _config.outputMax);

    // PI output
    float output = _config.kp * _state.error + _state.integrator;

    _state.lastOutput = output;
    return output;
}

void PIController::trackOutput(float actualOutput, bool active) {
    _state.isActive = active;

    if (!active && _config.trackingRate > 0.0f) {
        // Back-calculate: adjust integrator so our output would match actualOutput
        // This prevents windup when this controller is not the active one.
        // Target: kp * error + integrator_new = actualOutput
        // integrator_new = actualOutput - kp * error
        float targetIntegrator = actualOutput - _config.kp * _state.error;

        // Exponential tracking toward target
        float alpha = _config.trackingRate;  // rate per call (~per 10ms)
        _state.integrator += alpha * (targetIntegrator - _state.integrator);
    }
}


// ============================================================================
// InspValveController
// ============================================================================

InspValveController::InspValveController() {
    memset(&_config, 0, sizeof(_config));
    memset(&_status, 0, sizeof(_status));
}

void InspValveController::begin(const InspValveConfig& config) {
    _config = config;
    _flowPI.configure(config.flowPI);
    _pressurePI.configure(config.pressurePI);
    reset();
}

void InspValveController::reset() {
    _flowPI.reset();
    _pressurePI.reset();
    memset(&_status, 0, sizeof(_status));
}

void InspValveController::setCvTable(const LookupPoint* points, uint8_t count) {
    _cvTable.load(points, count);
}

void InspValveController::setFlowBand(uint8_t bandIndex, float refPressure,
                                       const LookupPoint* points, uint8_t count) {
    _flowSurface.loadBand(bandIndex, refPressure, points, count);
}

float InspValveController::computeFeedforward(float desiredFlow_slm,
                                               float supplyPressure_kPa,
                                               float downstreamPressure_mbar) const {
    if (!_config.useFeedforward) return 0.0f;

    // Prefer 2D flow surface if loaded
    if (_flowSurface.getBandCount() > 0) {
        // Inverse lookup on the pressure-banded flow surface:
        // Given desired flow and live supply pressure → V%
        // The V% is used directly as the feedforward output
        return _flowSurface.inverseLookup(desiredFlow_slm, supplyPressure_kPa);
    }

    // Fallback to legacy Cv table
    if (_cvTable.getCount() == 0) return 0.0f;
    // Convert downstream mbar to kPa for consistent units
    float deltaP = supplyPressure_kPa - downstreamPressure_mbar / 10.0f;
    if (deltaP < 0.1f) deltaP = 0.1f;
    float sqrtDeltaP = sqrtf(deltaP);
    float cvNeeded = desiredFlow_slm / sqrtDeltaP;
    return _cvTable.inverseLookup(cvNeeded);
}

float InspValveController::update(const InspValveSetpoints& sp,
                                   const InspValveMeasurements& meas,
                                   float dt) {
    // Feedforward
    float I_ff = computeFeedforward(sp.flow_slm, meas.supplyPressure_kPa,
                                     meas.airwayPressure_mbar);
    _status.feedforwardCurrent_A = I_ff;

    // Flow PI: wants to achieve the flow setpoint
    // Output is a current correction added to feedforward
    float flowOutput = _flowPI.update(sp.flow_slm, meas.flow_slm, dt) + I_ff;
    _status.flowPIOutput_A = flowOutput;

    // Pressure PI: limits current to prevent exceeding pressure limit
    // When pressure is below limit, this controller outputs a large (non-limiting) current.
    // When pressure exceeds limit, it pulls the current down.
    float pressureOutput = _pressurePI.update(sp.pressureLimit_mbar,
                                               meas.airwayPressure_mbar, dt);
    _status.pressurePIOutput_A = pressureOutput;

    // Override control: take the minimum (most restrictive)
    float selectedOutput;
    bool pressureLimiting;
    if (pressureOutput < flowOutput) {
        selectedOutput = pressureOutput;
        pressureLimiting = true;
    } else {
        selectedOutput = flowOutput;
        pressureLimiting = false;
    }

    // Anti-windup tracking: tell each PI what the actual output is
    _flowPI.trackOutput(selectedOutput, !pressureLimiting);
    _pressurePI.trackOutput(selectedOutput, pressureLimiting);

    // Apply cracking current: overcome spring + pressure-dependent dead zone.
    // I_crack = baseOffset + pressCoeff * P_supply_bar
    // When setpoint is zero, output zero (no offset).
    if (sp.flow_slm > 0.01f && selectedOutput > 0.0f) {
        float supplyPressure_bar = meas.supplyPressure_kPa / 100.0f;
        float crackCurrent = _config.crackBaseOffset_A
                           + _config.crackPressCoeff_A_per_bar * supplyPressure_bar;
        selectedOutput += crackCurrent;
    }

    // Clamp to hardware limits
    selectedOutput = constrain(selectedOutput, _config.minCurrent_A, _config.maxCurrent_A);

    _status.outputCurrent_A = selectedOutput;
    _status.pressureLimiting = pressureLimiting;
    _status.active = (sp.flow_slm > 0.01f);

    return selectedOutput;
}


// ============================================================================
// ExpValveController
// ============================================================================

ExpValveController::ExpValveController() {
    memset(&_config, 0, sizeof(_config));
    memset(&_status, 0, sizeof(_status));
}

void ExpValveController::begin(const ExpValveConfig& config) {
    _config = config;
    _pressurePI.configure(config.pressurePI);
    reset();
}

void ExpValveController::reset() {
    _pressurePI.reset();
    memset(&_status, 0, sizeof(_status));
}

void ExpValveController::setPressureTable(const LookupPoint* points, uint8_t count) {
    _pressureTable.load(points, count);
}

float ExpValveController::update(float pressureSetpoint_mbar,
                                  float measuredPressure_mbar,
                                  float dt) {
    // Feedforward from lookup table
    float I_ff = 0.0f;
    if (_config.useFeedforward && _pressureTable.getCount() > 0) {
        I_ff = _pressureTable.lookup(pressureSetpoint_mbar);
    }
    _status.feedforwardCurrent_A = I_ff;

    // Pressure PI correction
    float piOutput = _pressurePI.update(pressureSetpoint_mbar, measuredPressure_mbar, dt);
    _status.pressurePIOutput_A = piOutput;

    // Combine feedforward + PI correction
    float totalCurrent = I_ff + piOutput;

    // Apply cracking current (spring only, no supply pressure dependency for exp valve)
    if (pressureSetpoint_mbar > 0.1f && totalCurrent > 0.0f) {
        totalCurrent += _config.crackBaseOffset_A;
    }

    // Clamp
    totalCurrent = constrain(totalCurrent, _config.minCurrent_A, _config.maxCurrent_A);

    _status.outputCurrent_A = totalCurrent;
    _status.active = true;

    _pressurePI.trackOutput(totalCurrent, true);

    return totalCurrent;
}

float ExpValveController::forceClose() {
    reset();
    // For a normally-open exp valve: max current = fully closed
    // For a normally-closed exp valve: 0 current = fully closed
    // Adjust based on your valve type. Default: max current = closed.
    _status.outputCurrent_A = _config.maxCurrent_A;
    _status.active = false;
    return _config.maxCurrent_A;
}

float ExpValveController::forceOpen() {
    reset();
    _status.outputCurrent_A = _config.minCurrent_A;
    _status.active = false;
    return _config.minCurrent_A;
}


// ============================================================================
// BlowerController
// ============================================================================

BlowerController::BlowerController() {
    memset(&_config, 0, sizeof(_config));
    memset(&_status, 0, sizeof(_status));
}

void BlowerController::begin(const BlowerConfig& config) {
    _config = config;
    _flowPI.configure(config.flowPI);
    _pressurePI.configure(config.pressurePI);
    reset();
}

void BlowerController::reset() {
    _flowPI.reset();
    _pressurePI.reset();
    memset(&_status, 0, sizeof(_status));
}

void BlowerController::setFlowBand(uint8_t bandIndex, float refPaw_mbar,
                                    const LookupPoint* points, uint8_t count) {
    _flowSurface.loadBand(bandIndex, refPaw_mbar, points, count);
}

float BlowerController::computeFeedforward(float desiredFlow_slm, float paw_mbar) const {
    if (!_config.useFeedforward) return 0.0f;
    if (_flowSurface.getBandCount() == 0) return 0.0f;

    // Inverse lookup on the pressure-banded flow surface:
    // Given desired flow and live Paw → PWM%
    return _flowSurface.inverseLookup(desiredFlow_slm, paw_mbar);
}

float BlowerController::update(const BlowerSetpoints& sp,
                                const BlowerMeasurements& meas,
                                float dt) {
    // Feedforward: desired flow + live Paw → PWM%
    float pwm_ff = computeFeedforward(sp.flow_slm, meas.airwayPressure_mbar);
    _status.feedforwardPwmPercent = pwm_ff;

    // Flow PI: wants to achieve the flow setpoint
    // Output is a PWM% correction added to feedforward
    float flowOutput = _flowPI.update(sp.flow_slm, meas.flow_slm, dt) + pwm_ff;
    _status.flowPIOutput = flowOutput;

    // Pressure PI: limits PWM% to prevent exceeding pressure limit
    float pressureOutput = _pressurePI.update(sp.pressureLimit_mbar,
                                               meas.airwayPressure_mbar, dt);
    _status.pressurePIOutput = pressureOutput;

    // Override control: take the minimum (most restrictive)
    float selectedOutput;
    bool pressureLimiting;
    if (pressureOutput < flowOutput) {
        selectedOutput = pressureOutput;
        pressureLimiting = true;
    } else {
        selectedOutput = flowOutput;
        pressureLimiting = false;
    }

    // Anti-windup tracking
    _flowPI.trackOutput(selectedOutput, !pressureLimiting);
    _pressurePI.trackOutput(selectedOutput, pressureLimiting);

    // Clamp to hardware limits
    selectedOutput = constrain(selectedOutput, _config.minPwmPercent, _config.maxPwmPercent);

    _status.outputPwmPercent = selectedOutput;
    _status.pressureLimiting = pressureLimiting;
    _status.active = (sp.flow_slm > 0.01f);

    return selectedOutput;
}


// ============================================================================
// LocalValveController — Top-Level Coordinator
// ============================================================================

LocalValveController::LocalValveController()
    : _muxRouter(nullptr),
      _blowerActuator(nullptr),
      _enabled(false),
      _airMuxChannel(1),
      _o2MuxChannel(2),
      _expMuxChannel(3) {
}

void LocalValveController::begin(SerialMuxRouter* muxRouter,
                                  ActuatorControl* blowerActuator,
                                  const LocalValveControllerConfig& config) {
    _muxRouter = muxRouter;
    _blowerActuator = blowerActuator;
    _airMuxChannel = config.airMuxChannel;
    _o2MuxChannel = config.o2MuxChannel;
    _expMuxChannel = config.expMuxChannel;

    _airValve.begin(config.airValve);
    _o2Valve.begin(config.o2Valve);
    _expValve.begin(config.expValve);
    _blower.begin(config.blower);
}

void LocalValveController::setDefaults() {
    // ======================================================================
    // PLACEHOLDER CONFIGURATIONS — TUNE THESE DURING COMMISSIONING
    // ======================================================================

    LocalValveControllerConfig config;

    // --- Air inspiratory valve ---
    config.airValve.flowPI = {
        .kp = 0.01f,           // V% per slm error
        .ki = 0.1f,            // V% per slm*s error
        .outputMin = -14.0f,   // Allow negative correction to counteract feedforward
        .outputMax = 14.0f,    // Max correction V% (valve saturates ~10-14%)
        .trackingRate = 0.3f   // Anti-windup tracking
    };
    config.airValve.pressurePI = {
        .kp = 0.02f,           // V% per mbar error
        .ki = 0.2f,            // V% per mbar*s error
        .outputMin = 0.0f,     // Pressure PI only limits (output is a ceiling)
        .outputMax = 14.0f,
        .trackingRate = 0.3f
    };
    config.airValve.maxCurrent_A = 14.0f;   // Max V% output
    config.airValve.minCurrent_A = 0.0f;
    config.airValve.crackBaseOffset_A = 0.0f;         // Handled by feedforward table
    config.airValve.crackPressCoeff_A_per_bar = 0.0f; // Handled by feedforward table
    config.airValve.useFeedforward = true;

    // --- O2 inspiratory valve (same structure, possibly different gains) ---
    config.o2Valve = config.airValve;  // Start with same gains

    // --- Expiratory valve ---
    config.expValve.pressurePI = {
        .kp = 0.01f,           // V% per mbar error
        .ki = 0.1f,            // V% per mbar*s error
        .outputMin = 0.0f,
        .outputMax = 14.0f,
        .trackingRate = 0.0f   // No tracking needed (single PI)
    };
    config.expValve.maxCurrent_A = 14.0f;   // Max V% output
    config.expValve.minCurrent_A = 0.0f;
    config.expValve.crackBaseOffset_A = 0.0f;   // No cracking offset in V% mode
    config.expValve.useFeedforward = true;

    // --- Blower (GPIO21 PWM) ---
    config.blower.flowPI = {
        .kp = 0.05f,           // PWM% per slm error
        .ki = 0.5f,            // PWM% per slm*s error
        .outputMin = -100.0f,  // Allow negative correction to counteract feedforward
        .outputMax = 100.0f,   // Max correction PWM%
        .trackingRate = 0.3f   // Anti-windup tracking
    };
    config.blower.pressurePI = {
        .kp = 1.0f,            // PWM% per mbar error
        .ki = 5.0f,            // PWM% per mbar*s error
        .outputMin = 0.0f,     // Pressure PI only limits (ceiling)
        .outputMax = 100.0f,
        .trackingRate = 0.3f
    };
    config.blower.maxPwmPercent = 100.0f;
    config.blower.minPwmPercent = 0.0f;
    config.blower.useFeedforward = true;

    // MUX channel assignments
    config.airMuxChannel = 1;   // MUX_AIR_VALVE
    config.o2MuxChannel = 2;    // MUX_O2_VALVE
    config.expMuxChannel = 3;   // MUX_EXP_VALVE

    begin(_muxRouter, _blowerActuator, config);

    // ======================================================================
    // PRESSURE-BANDED FLOW TABLES (2D flow surface)
    // ======================================================================
    // Each band is a V% → Flow (slm) table at a reference supply pressure.
    // Measured from CC1 sweeps at ~2.6, 4, and 5.5 bar set pressures.
    // Normalized for pressure droop (--normalize).
    // The feedforward does inverse lookup: desired flow + live Psupply → V%.
    //
    // Reduced to ≤12 points per band: dead zone → cracking → active → saturation.

    // Band 0: Psupply ~ 264 kPa (2.64 bar)  (12 pts from 25 raw)
    // x = V%, y = Flow (slm)
    static const LookupPoint airFlowBand0[] = {
        {   0.00f,    -0.00f},  // Dead zone
        {   5.00f,     0.00f},  // Dead zone
        {   5.50f,     0.20f},  // Cracking
        {   6.00f,     2.24f},  // Cracking
        {   6.50f,     9.66f},  // Cracking
        {   7.50f,    28.42f},
        {   8.00f,    40.25f},
        {   8.50f,    53.65f},
        {   9.00f,    70.85f},
        {   9.50f,   105.32f},
        {  10.00f,   111.58f},
        {  12.00f,   112.05f}   // Saturated
    };

    // Band 1: Psupply ~ 401 kPa (4.01 bar)  (12 pts from 25 raw)
    // x = V%, y = Flow (slm)
    static const LookupPoint airFlowBand1[] = {
        {   0.00f,     0.00f},  // Dead zone
        {   5.50f,     0.04f},  // Dead zone
        {   6.00f,     0.64f},  // Cracking
        {   6.50f,     5.83f},  // Cracking
        {   7.00f,    15.55f},  // Cracking
        {   8.00f,    42.45f},
        {   8.50f,    59.23f},
        {   9.00f,    79.64f},
        {   9.50f,   106.67f},
        {  10.00f,   144.83f},
        {  11.00f,   159.60f},  // Saturated
        {  12.00f,   160.50f}   // Saturated
    };

    // Band 2: Psupply ~ 553 kPa (5.53 bar)  (12 pts from 25 raw)
    // x = V%, y = Flow (slm)
    static const LookupPoint airFlowBand2[] = {
        {   0.00f,     0.00f},  // Dead zone
        {   5.50f,     0.03f},  // Dead zone
        {   6.00f,     0.48f},  // Cracking
        {   6.50f,     3.68f},  // Cracking
        {   7.00f,    14.51f},  // Cracking
        {   8.00f,    44.86f},
        {   8.50f,    69.13f},
        {   9.00f,    90.00f},
        {   9.50f,   117.18f},
        {  10.00f,   153.70f},
        {  10.50f,   189.31f},
        {  12.00f,   204.00f}   // Saturated
    };

    _airValve.setFlowBand(0, 264.0f, airFlowBand0, 12);
    _airValve.setFlowBand(1, 401.0f, airFlowBand1, 12);
    _airValve.setFlowBand(2, 553.0f, airFlowBand2, 12);

    // O2 valve: use same tables as air until separately characterized
    _o2Valve.setFlowBand(0, 264.0f, airFlowBand0, 12);
    _o2Valve.setFlowBand(1, 401.0f, airFlowBand1, 12);
    _o2Valve.setFlowBand(2, 553.0f, airFlowBand2, 12);

    // Expiratory valve pressure table: x = PEEP setpoint (mbar), y = current (A)
    // Higher current = more closed = higher held pressure
    static const LookupPoint expPressureTable[] = {
        {0.0f,  0.00f},    // Fully open
        {2.0f,  0.10f},
        {5.0f,  0.25f},    // Typical PEEP
        {10.0f, 0.50f},
        {15.0f, 0.75f},
        {20.0f, 1.00f},
        {30.0f, 1.40f},
        {40.0f, 1.80f},    // High PEEP / pop-off
    };
    _expValve.setPressureTable(expPressureTable,
                               sizeof(expPressureTable) / sizeof(expPressureTable[0]));

    // ======================================================================
    // BLOWER PRESSURE-BANDED FLOW TABLES (2D flow surface)
    // ======================================================================
    // Each band is a PWM% → Flow (slm) table at a reference counter pressure (Paw).
    // Measured from CC4 sweeps at increasing exp valve restriction (V0, V1, V2, V4).
    // The feedforward does inverse lookup: desired flow + live Paw → PWM%.
    //
    // Reduced from 21 to 16 points per band: dead → active → saturation.

    // Band 0: Paw_ref ≈ 8.4 mbar — CC4 sweep, no restriction (exp valve open)
    // Flow range 0–227 slm, saturates at 85% PWM
    static const LookupPoint blowerFlowBand0[] = {
        {  0.00f,    0.00f},   // Dead
        {  5.00f,   11.90f},   // First response
        { 10.00f,   18.21f},
        { 15.00f,   26.46f},
        { 20.00f,   39.41f},
        { 25.00f,   52.83f},
        { 30.00f,   66.34f},
        { 40.00f,   96.70f},
        { 50.00f,  129.25f},
        { 60.00f,  159.24f},
        { 65.00f,  175.06f},
        { 70.00f,  190.16f},
        { 75.00f,  206.56f},
        { 80.00f,  221.14f},
        { 85.00f,  226.41f},   // Saturation onset
        {100.00f,  226.85f},   // Flat at max
    };

    // Band 1: Paw_ref ≈ 13.3 mbar — CC4 sweep, exp V=1%
    // Flow range 0–213 slm, saturates at 85% PWM
    static const LookupPoint blowerFlowBand1[] = {
        {  0.00f,    0.00f},
        {  5.00f,    8.43f},
        { 10.00f,   11.75f},
        { 15.00f,   17.62f},
        { 20.00f,   25.93f},
        { 25.00f,   34.50f},
        { 30.00f,   43.77f},
        { 40.00f,   74.08f},
        { 50.00f,  108.01f},
        { 60.00f,  138.21f},
        { 65.00f,  155.55f},
        { 70.00f,  172.40f},
        { 75.00f,  188.14f},
        { 80.00f,  204.23f},
        { 85.00f,  213.29f},   // Saturation onset
        {100.00f,  213.25f},   // Flat at max
    };

    // Band 2: Paw_ref ≈ 19.0 mbar — CC4 sweep, exp V=2%
    // Flow range 0–190 slm, saturates at 85% PWM
    static const LookupPoint blowerFlowBand2[] = {
        {  0.00f,    0.00f},
        {  5.00f,    8.89f},
        { 10.00f,   11.49f},
        { 15.00f,   17.19f},
        { 20.00f,   24.88f},
        { 25.00f,   32.57f},
        { 30.00f,   40.24f},
        { 40.00f,   57.65f},
        { 50.00f,   79.31f},
        { 60.00f,  105.13f},
        { 65.00f,  124.18f},
        { 70.00f,  142.60f},
        { 75.00f,  161.52f},
        { 80.00f,  177.61f},
        { 85.00f,  190.31f},   // Saturation onset
        {100.00f,  190.03f},   // Flat at max
    };

    // Band 3: Paw_ref ≈ 24.7 mbar — CC4 sweep, exp V=4%
    // Flow range 0–149 slm, saturates at 85% PWM
    static const LookupPoint blowerFlowBand3[] = {
        {  0.00f,    0.00f},
        {  5.00f,    9.00f},
        { 10.00f,   11.50f},
        { 15.00f,   17.21f},
        { 20.00f,   24.95f},
        { 25.00f,   32.33f},
        { 30.00f,   40.10f},
        { 40.00f,   57.19f},
        { 50.00f,   77.69f},
        { 60.00f,   95.05f},
        { 65.00f,  103.99f},
        { 70.00f,  112.79f},
        { 75.00f,  122.59f},
        { 80.00f,  135.57f},
        { 85.00f,  148.32f},   // Saturation onset
        {100.00f,  149.34f},   // Flat at max
    };

    _blower.setFlowBand(0,  8.4f, blowerFlowBand0, 16);
    _blower.setFlowBand(1, 13.3f, blowerFlowBand1, 16);
    _blower.setFlowBand(2, 19.0f, blowerFlowBand2, 16);
    _blower.setFlowBand(3, 24.7f, blowerFlowBand3, 16);

    Serial.println("[LLC] Local valve controllers initialized with pressure-banded flow tables");
    Serial.println("[LLC] Blower: 4 bands (Paw 8.4–24.7 mbar), max ~227 slm");
    Serial.println("[LLC] ⚠️ Calibrate valve tables before clinical use!");
}

void LocalValveController::reset() {
    _airValve.reset();
    _o2Valve.reset();
    _expValve.reset();
    _blower.reset();
}

void LocalValveController::update(const LocalValveControllerSetpoints& sp,
                                   const LocalValveControllerMeasurements& meas,
                                   float dt) {
    if (!_enabled || !_muxRouter) {
        // Pass-through mode: send flow/pressure setpoints directly to MUX
        // (original behavior when downstream controllers handle the LLC)
        if (_muxRouter) {
            if (_airMuxChannel != sp.skipMuxChannel)
                _muxRouter->sendSetFlow(_airMuxChannel, sp.airFlow_slm);
            if (_o2MuxChannel != sp.skipMuxChannel)
                _muxRouter->sendSetFlow(_o2MuxChannel, sp.o2Flow_slm);
            if (_expMuxChannel != sp.skipMuxChannel) {
                if (sp.expValveClosed) {
                    _muxRouter->sendCommand(_expMuxChannel, 'V', 0);
                } else if (sp.expValveActive) {
                    _muxRouter->sendSetPressure(_expMuxChannel, sp.expPressure_mbar);
                }
            }
        }
        return;
    }

    // === Local loop control mode ===

    // --- Air valve ---
    InspValveSetpoints airSP;
    airSP.flow_slm = sp.airFlow_slm;
    airSP.pressureLimit_mbar = sp.maxPressure_mbar;

    InspValveMeasurements airMeas;
    airMeas.flow_slm = meas.airFlow_slm;
    airMeas.supplyPressure_kPa = meas.airSupplyPressure_kPa;
    airMeas.airwayPressure_mbar = meas.airwayPressure_mbar;

    float airCurrent = _airValve.update(airSP, airMeas, dt);
    if (_airMuxChannel != sp.skipMuxChannel)
        sendValveCurrent(_airMuxChannel, airCurrent);

    // --- O2 valve ---
    InspValveSetpoints o2SP;
    o2SP.flow_slm = sp.o2Flow_slm;
    o2SP.pressureLimit_mbar = sp.maxPressure_mbar;

    InspValveMeasurements o2Meas;
    o2Meas.flow_slm = meas.o2Flow_slm;
    o2Meas.supplyPressure_kPa = meas.o2SupplyPressure_kPa;
    o2Meas.airwayPressure_mbar = meas.airwayPressure_mbar;

    float o2Current = _o2Valve.update(o2SP, o2Meas, dt);
    if (_o2MuxChannel != sp.skipMuxChannel)
        sendValveCurrent(_o2MuxChannel, o2Current);

    // --- Expiratory valve ---
    float expCurrent;
    if (sp.expValveClosed) {
        expCurrent = _expValve.forceClose();
    } else if (sp.expValveActive) {
        expCurrent = _expValve.update(sp.expPressure_mbar, meas.airwayPressure_mbar, dt);
    } else {
        expCurrent = _expValve.forceOpen();
    }
    if (_expMuxChannel != sp.skipMuxChannel)
        sendValveCurrent(_expMuxChannel, expCurrent);

    // --- Blower ---
    if (sp.skipBlower) {
        // Characterizer owns blower output — don't touch
    } else if (sp.blowerActive && sp.blowerFlow_slm > 0.01f) {
        BlowerSetpoints blowerSP;
        blowerSP.flow_slm = sp.blowerFlow_slm;
        blowerSP.pressureLimit_mbar = sp.blowerPressureLimit_mbar;

        BlowerMeasurements blowerMeas;
        blowerMeas.flow_slm = meas.blowerFlow_slm;
        blowerMeas.airwayPressure_mbar = meas.airwayPressure_mbar;

        float blowerPwm = _blower.update(blowerSP, blowerMeas, dt);
        sendBlowerPwm(blowerPwm);
    } else {
        _blower.reset();
        sendBlowerPwm(0.0f);
    }
}

void LocalValveController::sendValveCurrent(uint8_t channel, float current_A) {
    if (_muxRouter) {
        // Motor drivers accept V% commands, not current — send as V%
        _muxRouter->sendCommand(channel, 'V', current_A);
    }
}

void LocalValveController::sendBlowerPwm(float pwmPercent) {
    if (_blowerActuator) {
        _blowerActuator->outputToValve(pwmPercent);
    }
}
