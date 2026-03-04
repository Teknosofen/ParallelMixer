#include "LocalValveController.hpp"
#include "SerialMuxRouter.hpp"

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
                                               float supplyPressure_mbar,
                                               float downstreamPressure_mbar) const {
    if (!_config.useFeedforward) return 0.0f;

    // Prefer 2D flow surface if loaded
    if (_flowSurface.getBandCount() > 0) {
        // Inverse lookup on the pressure-banded flow surface:
        // Given desired flow and live supply pressure → V%
        // The V% is used directly as the feedforward output
        return _flowSurface.inverseLookup(desiredFlow_slm, supplyPressure_mbar);
    }

    // Fallback to legacy Cv table
    if (_cvTable.getCount() == 0) return 0.0f;
    float deltaP = supplyPressure_mbar - downstreamPressure_mbar;
    if (deltaP < 1.0f) deltaP = 1.0f;
    float sqrtDeltaP = sqrtf(deltaP);
    float cvNeeded = desiredFlow_slm / sqrtDeltaP;
    return _cvTable.inverseLookup(cvNeeded);
}

float InspValveController::update(const InspValveSetpoints& sp,
                                   const InspValveMeasurements& meas,
                                   float dt) {
    // Feedforward
    float I_ff = computeFeedforward(sp.flow_slm, meas.supplyPressure_mbar,
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
        float supplyPressure_bar = meas.supplyPressure_mbar / 1000.0f;
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
// LocalValveController — Top-Level Coordinator
// ============================================================================

LocalValveController::LocalValveController()
    : _muxRouter(nullptr),
      _enabled(false),
      _airMuxChannel(1),
      _o2MuxChannel(2),
      _expMuxChannel(3) {
}

void LocalValveController::begin(SerialMuxRouter* muxRouter,
                                  const LocalValveControllerConfig& config) {
    _muxRouter = muxRouter;
    _airMuxChannel = config.airMuxChannel;
    _o2MuxChannel = config.o2MuxChannel;
    _expMuxChannel = config.expMuxChannel;

    _airValve.begin(config.airValve);
    _o2Valve.begin(config.o2Valve);
    _expValve.begin(config.expValve);
}

void LocalValveController::setDefaults() {
    // ======================================================================
    // PLACEHOLDER CONFIGURATIONS — TUNE THESE DURING COMMISSIONING
    // ======================================================================

    LocalValveControllerConfig config;

    // --- Air inspiratory valve ---
    config.airValve.flowPI = {
        .kp = 0.05f,           // A per slm error
        .ki = 0.5f,            // A per slm*s error
        .outputMin = 0.0f,
        .outputMax = 2.0f,     // Max correction current
        .trackingRate = 0.3f   // Anti-windup tracking
    };
    config.airValve.pressurePI = {
        .kp = 0.1f,            // A per mbar error
        .ki = 1.0f,            // A per mbar*s error
        .outputMin = 0.0f,
        .outputMax = 2.0f,
        .trackingRate = 0.3f
    };
    config.airValve.maxCurrent_A = 2.0f;
    config.airValve.minCurrent_A = 0.0f;
    config.airValve.crackBaseOffset_A = 0.10f;        // PLACEHOLDER — cracking current at 0 bar supply
    config.airValve.crackPressCoeff_A_per_bar = 0.05f; // PLACEHOLDER — additional A per bar supply
    config.airValve.useFeedforward = true;

    // --- O2 inspiratory valve (same structure, possibly different gains) ---
    config.o2Valve = config.airValve;  // Start with same gains

    // --- Expiratory valve ---
    config.expValve.pressurePI = {
        .kp = 0.05f,           // A per mbar error
        .ki = 0.5f,            // A per mbar*s error
        .outputMin = 0.0f,
        .outputMax = 2.0f,
        .trackingRate = 0.0f   // No tracking needed (single PI)
    };
    config.expValve.maxCurrent_A = 2.0f;
    config.expValve.minCurrent_A = 0.0f;
    config.expValve.crackBaseOffset_A = 0.10f;  // PLACEHOLDER — measure cracking current
    config.expValve.useFeedforward = true;

    // MUX channel assignments
    config.airMuxChannel = 1;   // MUX_AIR_VALVE
    config.o2MuxChannel = 2;    // MUX_O2_VALVE
    config.expMuxChannel = 3;   // MUX_EXP_VALVE

    begin(_muxRouter, config);

    // ======================================================================
    // PRESSURE-BANDED FLOW TABLES (2D flow surface)
    // ======================================================================
    // Each band is a V% → Flow (slm) table at a reference supply pressure.
    // Measured from CC1 sweeps at ~3, 4.5, and 6 bar set pressures.
    // The feedforward does inverse lookup: desired flow + live Psupply → V%.
    //
    // Reduced to ≤16 points per band: dead zone → cracking → active → saturation.

    // Band 0: P_ref = 302 kPa (~3 bar) — CC1 sweep, Psupply 302→186 kPa
    static const LookupPoint airFlowBand0[] = {
        { 0.00f,    0.0f},    // Dead zone
        { 5.00f,    0.0f},    // Still closed
        { 5.40f,    0.23f},   // Just cracking
        { 5.80f,    1.38f},
        { 6.00f,    3.06f},
        { 6.40f,    9.27f},
        { 6.80f,   16.08f},
        { 7.20f,   23.79f},
        { 7.60f,   32.31f},
        { 8.00f,   42.32f},
        { 8.40f,   52.92f},
        { 8.80f,   65.84f},
        { 9.20f,   89.53f},
        { 9.60f,  110.56f},
        {10.00f,  111.79f},   // Saturated (~186 kPa supply)
        {14.00f,  112.39f},   // Flat at max
    };

    // Band 1: P_ref = 451 kPa (~4.5 bar) — CC1 sweep, Psupply 451→298 kPa
    static const LookupPoint airFlowBand1[] = {
        { 0.00f,    0.0f},    // Dead zone
        { 5.00f,    0.0f},    // Still closed
        { 5.50f,    0.17f},   // Just cracking
        { 6.00f,    1.42f},
        { 6.50f,    8.73f},
        { 7.00f,   18.87f},
        { 7.50f,   31.04f},
        { 8.00f,   46.83f},
        { 8.50f,   62.05f},
        { 9.00f,   74.92f},
        { 9.50f,  111.69f},
        {10.00f,  147.76f},
        {10.50f,  157.79f},
        {11.00f,  159.17f},   // Saturated (~298 kPa supply)
        {14.00f,  159.47f},   // Flat at max
    };

    // Band 2: P_ref = 604 kPa (~6 bar) — CC1 sweep, Psupply 604→376 kPa
    static const LookupPoint airFlowBand2[] = {
        { 0.00f,    0.0f},    // Dead zone
        { 5.00f,    0.0f},    // Still closed
        { 5.60f,    0.17f},   // Just cracking
        { 6.00f,    0.91f},
        { 6.40f,    4.08f},
        { 6.80f,   12.89f},
        { 7.20f,   22.48f},
        { 7.60f,   34.40f},
        { 8.00f,   48.91f},
        { 8.40f,   68.54f},
        { 8.80f,   84.81f},
        { 9.20f,  103.18f},
        { 9.60f,  127.14f},
        {10.00f,  160.04f},
        {10.60f,  194.88f},
        {11.20f,  195.50f},   // Saturated (~384 kPa supply)
    };

    _airValve.setFlowBand(0, 302.0f, airFlowBand0,
                          sizeof(airFlowBand0) / sizeof(airFlowBand0[0]));
    _airValve.setFlowBand(1, 451.0f, airFlowBand1,
                          sizeof(airFlowBand1) / sizeof(airFlowBand1[0]));
    _airValve.setFlowBand(2, 604.0f, airFlowBand2,
                          sizeof(airFlowBand2) / sizeof(airFlowBand2[0]));

    // O2 valve: use same tables as air until separately characterized
    _o2Valve.setFlowBand(0, 302.0f, airFlowBand0,
                         sizeof(airFlowBand0) / sizeof(airFlowBand0[0]));
    _o2Valve.setFlowBand(1, 451.0f, airFlowBand1,
                         sizeof(airFlowBand1) / sizeof(airFlowBand1[0]));
    _o2Valve.setFlowBand(2, 604.0f, airFlowBand2,
                         sizeof(airFlowBand2) / sizeof(airFlowBand2[0]));

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

    Serial.println("[LLC] Local valve controllers initialized with pressure-banded flow tables");
    Serial.println("[LLC] ⚠️ Calibrate valve tables before clinical use!");
}

void LocalValveController::reset() {
    _airValve.reset();
    _o2Valve.reset();
    _expValve.reset();
}

void LocalValveController::update(const LocalValveControllerSetpoints& sp,
                                   const LocalValveControllerMeasurements& meas,
                                   float dt) {
    if (!_enabled || !_muxRouter) {
        // Pass-through mode: send flow/pressure setpoints directly to MUX
        // (original behavior when downstream controllers handle the LLC)
        if (_muxRouter) {
            _muxRouter->sendSetFlow(_airMuxChannel, sp.airFlow_slm);
            _muxRouter->sendSetFlow(_o2MuxChannel, sp.o2Flow_slm);
            if (sp.expValveClosed) {
                _muxRouter->sendCommand(_expMuxChannel, 'V', 0);
            } else if (sp.expValveActive) {
                _muxRouter->sendSetPressure(_expMuxChannel, sp.expPressure_mbar);
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
    airMeas.supplyPressure_mbar = meas.airSupplyPressure_mbar;
    airMeas.airwayPressure_mbar = meas.airwayPressure_mbar;

    float airCurrent = _airValve.update(airSP, airMeas, dt);
    sendValveCurrent(_airMuxChannel, airCurrent);

    // --- O2 valve ---
    InspValveSetpoints o2SP;
    o2SP.flow_slm = sp.o2Flow_slm;
    o2SP.pressureLimit_mbar = sp.maxPressure_mbar;

    InspValveMeasurements o2Meas;
    o2Meas.flow_slm = meas.o2Flow_slm;
    o2Meas.supplyPressure_mbar = meas.o2SupplyPressure_mbar;
    o2Meas.airwayPressure_mbar = meas.airwayPressure_mbar;

    float o2Current = _o2Valve.update(o2SP, o2Meas, dt);
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
    sendValveCurrent(_expMuxChannel, expCurrent);
}

void LocalValveController::sendValveCurrent(uint8_t channel, float current_A) {
    if (_muxRouter) {
        _muxRouter->sendSetCurrent(channel, current_A);
    }
}
