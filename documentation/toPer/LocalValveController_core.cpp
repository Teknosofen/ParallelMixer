// LocalValveController_core.cpp
// Extracted from src/LocalValveController.cpp — ParallelMixer project.
//
// Contains the algorithm implementations relevant to inspiratory valve
// linearisation and feed-forward:
//   - PiecewiseLinearTable  (1-D lookup + inverse lookup)
//   - PressureBandedTable   (2-D bilinear interpolation)
//   - PIController          (PI with anti-windup tracking)
//   - InspValveController   (feedforward + flow PI + pressure-limit PI)
//
// The expiratory valve and blower controllers are omitted here.
// For those, see the full src/LocalValveController.cpp in the source repo.

#include "LocalValveController.hpp"

// ============================================================================
// PiecewiseLinearTable — 1-D piecewise linear lookup
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

// Forward lookup: given x, return y.  Clamps at table edges.
float PiecewiseLinearTable::lookup(float x) const {
    if (_count == 0) return 0.0f;
    if (_count == 1) return _points[0].y;

    if (x <= _points[0].x) return _points[0].y;
    if (x >= _points[_count - 1].x) return _points[_count - 1].y;

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

// Inverse lookup: given y, return x.  Table must be monotonic.
// Clamps at table edges.
float PiecewiseLinearTable::inverseLookup(float y) const {
    if (_count == 0) return 0.0f;
    if (_count == 1) return _points[0].x;

    bool ascending = (_points[_count - 1].y >= _points[0].y);

    if (ascending) {
        if (y <= _points[0].y) return _points[0].x;
        if (y >= _points[_count - 1].y) return _points[_count - 1].x;
    } else {
        if (y >= _points[0].y) return _points[0].x;
        if (y <= _points[_count - 1].y) return _points[_count - 1].x;
    }

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
// PressureBandedTable — 2-D flow surface with bilinear interpolation
// ============================================================================
//
// Stores 2–4 PiecewiseLinearTables, each at a reference supply pressure.
// inverseLookup(desiredFlow, livePressure) interpolates — or extrapolates —
// between the nearest two bands to return the required V%.

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

// Find the two bands that bracket `pressure` and compute interpolation weight t.
// t < 0 or t > 1 when extrapolating outside the stored band range.
void PressureBandedTable::findBrackets(float pressure,
                                        uint8_t& lo, uint8_t& hi, float& t) const {
    if (_bandCount <= 1) {
        lo = hi = 0;
        t = 0.0f;
        return;
    }

    // Below lowest band — extrapolate using bands 0 and 1
    if (pressure <= _refPressure[0]) {
        lo = 0; hi = 1;
        float dp = _refPressure[1] - _refPressure[0];
        t = (dp > 1e-3f) ? (pressure - _refPressure[0]) / dp : 0.0f;
        return;
    }

    // Above highest band — extrapolate using last two bands
    if (pressure >= _refPressure[_bandCount - 1]) {
        lo = _bandCount - 2; hi = _bandCount - 1;
        float dp = _refPressure[hi] - _refPressure[lo];
        t = (dp > 1e-3f) ? (pressure - _refPressure[lo]) / dp : 1.0f;
        return;
    }

    // Normal case — find bracketing pair
    for (uint8_t i = 0; i < _bandCount - 1; i++) {
        if (pressure >= _refPressure[i] && pressure <= _refPressure[i + 1]) {
            lo = i; hi = i + 1;
            float dp = _refPressure[hi] - _refPressure[lo];
            t = (dp > 1e-3f) ? (pressure - _refPressure[lo]) / dp : 0.0f;
            return;
        }
    }

    lo = hi = _bandCount - 1;
    t = 0.0f;
}

// Forward lookup: given V% and supply pressure, return expected flow.
float PressureBandedTable::lookup(float valvePercent, float supplyPressure) const {
    if (_bandCount == 0) return 0.0f;
    if (_bandCount == 1) return _flowTables[0].lookup(valvePercent);

    uint8_t lo, hi; float t;
    findBrackets(supplyPressure, lo, hi, t);

    float flowLo = _flowTables[lo].lookup(valvePercent);
    float flowHi = _flowTables[hi].lookup(valvePercent);
    return flowLo + t * (flowHi - flowLo);
}

// Inverse lookup: given desired flow and live supply pressure, return V%.
// This is the core feedforward computation.
//
//   Step 1: findBrackets → lo band, hi band, weight t
//   Step 2: inverseLookup(desiredFlow) on each band → vpctLo, vpctHi
//   Step 3: interpolate V% = vpctLo + t * (vpctHi - vpctLo)
float PressureBandedTable::inverseLookup(float desiredFlow, float supplyPressure) const {
    if (_bandCount == 0) return 0.0f;
    if (_bandCount == 1) return _flowTables[0].inverseLookup(desiredFlow);

    uint8_t lo, hi; float t;
    findBrackets(supplyPressure, lo, hi, t);

    float vpctLo = _flowTables[lo].inverseLookup(desiredFlow);
    float vpctHi = _flowTables[hi].inverseLookup(desiredFlow);
    return vpctLo + t * (vpctHi - vpctLo);
}


// ============================================================================
// PIController — PI with anti-windup integrator tracking
// ============================================================================
//
// Anti-windup strategy: when this controller is NOT the active one (i.e. the
// other controller won the min-select), trackOutput() back-calculates the
// integrator value that would produce the actual output.  This prevents
// integrator wind-up and removes bumps when authority switches back.

PIController::PIController() {
    _config = {0, 0, -1e6f, 1e6f, 0};
    reset();
}

void PIController::configure(const PIConfig& config) {
    _config = config;
}

void PIController::reset() {
    _state.integrator  = 0.0f;
    _state.lastOutput  = 0.0f;
    _state.error       = 0.0f;
    _state.isActive    = false;
}

// Run one PI step.  Returns unclamped PI output.
// Caller: add feedforward, then min-select, then call trackOutput().
float PIController::update(float setpoint, float measurement, float dt) {
    _state.error = setpoint - measurement;

    _state.integrator += _config.ki * _state.error * dt;
    _state.integrator = constrain(_state.integrator, _config.outputMin, _config.outputMax);

    float output = _config.kp * _state.error + _state.integrator;
    _state.lastOutput = output;
    return output;
}

// Called after min-select with the actual output and which controller won.
// When inactive, exponentially tracks integrator toward back-calculated target.
void PIController::trackOutput(float actualOutput, bool active) {
    _state.isActive = active;

    if (!active && _config.trackingRate > 0.0f) {
        // Target integrator: value that would produce actualOutput given current error
        float targetIntegrator = actualOutput - _config.kp * _state.error;
        float alpha = _config.trackingRate;   // ~per 10 ms call
        _state.integrator += alpha * (targetIntegrator - _state.integrator);
    }
}


// ============================================================================
// InspValveController — feedforward + flow PI + pressure-limit PI
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

// Compute feedforward V% from desired flow and supply pressure.
// Prefers the 2-D pressure-banded surface; falls back to legacy Cv table.
float InspValveController::computeFeedforward(float desiredFlow_slm,
                                               float supplyPressure_kPa,
                                               float downstreamPressure_mbar) const {
    if (!_config.useFeedforward) return 0.0f;

    if (_flowSurface.getBandCount() > 0) {
        // 2-D inverse lookup: desired flow + live supply pressure → V%
        return _flowSurface.inverseLookup(desiredFlow_slm, supplyPressure_kPa);
    }

    // Legacy Cv table fallback
    if (_cvTable.getCount() == 0) return 0.0f;
    float deltaP = supplyPressure_kPa - downstreamPressure_mbar / 10.0f;
    if (deltaP < 0.1f) deltaP = 0.1f;
    float cvNeeded = desiredFlow_slm / sqrtf(deltaP);
    return _cvTable.inverseLookup(cvNeeded);
}

// Main update — call at 100 Hz.
// Returns valve current command [A].
//
// Control law:
//   I_ff         = computeFeedforward(desired_flow, supply_pressure)
//   flowOutput   = flowPI(error = flow_sp - flow_meas) + I_ff
//   pressOutput  = pressurePI(error = pressure_limit - paw_meas)
//   selected     = min(flowOutput, pressOutput)      ← override (min-select)
//   output       = selected + crackingCurrent        ← overcome spring
//   output       = clamp(output, minCurrent, maxCurrent)
float InspValveController::update(const InspValveSetpoints& sp,
                                   const InspValveMeasurements& meas,
                                   float dt) {
    // --- Feedforward ---
    float I_ff = computeFeedforward(sp.flow_slm, meas.supplyPressure_kPa,
                                     meas.airwayPressure_mbar);
    _status.feedforwardCurrent_A = I_ff;

    // --- Flow PI ---
    float flowOutput = _flowPI.update(sp.flow_slm, meas.flow_slm, dt) + I_ff;
    _status.flowPIOutput_A = flowOutput;

    // --- Pressure-limit PI ---
    // When Paw is below the limit this outputs a large non-limiting value.
    // When Paw exceeds the limit it pulls the output down.
    float pressureOutput = _pressurePI.update(sp.pressureLimit_mbar,
                                               meas.airwayPressure_mbar, dt);
    _status.pressurePIOutput_A = pressureOutput;

    // --- Override (min-select) ---
    bool pressureLimiting;
    float selectedOutput;
    if (pressureOutput < flowOutput) {
        selectedOutput   = pressureOutput;
        pressureLimiting = true;
    } else {
        selectedOutput   = flowOutput;
        pressureLimiting = false;
    }

    // --- Anti-windup: track integrators to actual output ---
    _flowPI.trackOutput(selectedOutput, !pressureLimiting);
    _pressurePI.trackOutput(selectedOutput, pressureLimiting);

    // --- Cracking current: overcome spring + pressure-dependent offset ---
    // I_crack = I_base + k * P_supply [bar]
    if (sp.flow_slm > 0.01f && selectedOutput > 0.0f) {
        float supplyPressure_bar = meas.supplyPressure_kPa / 100.0f;
        float crackCurrent = _config.crackBaseOffset_A
                           + _config.crackPressCoeff_A_per_bar * supplyPressure_bar;
        selectedOutput += crackCurrent;
    }

    // --- Hardware clamp ---
    selectedOutput = constrain(selectedOutput, _config.minCurrent_A, _config.maxCurrent_A);

    _status.outputCurrent_A   = selectedOutput;
    _status.pressureLimiting  = pressureLimiting;
    _status.active            = (sp.flow_slm > 0.01f);

    return selectedOutput;
}
