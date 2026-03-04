#ifndef LOCAL_VALVE_CONTROLLER_HPP
#define LOCAL_VALVE_CONTROLLER_HPP

#include <Arduino.h>

// Forward declarations
class SerialMuxRouter;

// ============================================================================
// Piecewise-Linear Lookup Table (1D)
// ============================================================================
// Stores (x, y) pairs sorted by x. Interpolates linearly between points.
// Used for valve linearization: x=flow/pressure, y=current (or inverse).

struct LookupPoint {
    float x;
    float y;
};

class PiecewiseLinearTable {
public:
    static const uint8_t MAX_POINTS = 16;

    PiecewiseLinearTable();

    /// Load table from array of points (must be sorted by x, ascending)
    void load(const LookupPoint* points, uint8_t count);

    /// Interpolate: given x, return y
    float lookup(float x) const;

    /// Inverse interpolate: given y, return x (table must be monotonic)
    float inverseLookup(float y) const;

    uint8_t getCount() const { return _count; }

private:
    LookupPoint _points[MAX_POINTS];
    uint8_t _count;
};


// ============================================================================
// Pressure-Banded Flow Table (2D)
// ============================================================================
// Holds 2-4 PiecewiseLinearTables, each mapping V% → Flow at a specific
// reference supply pressure.  Interpolates (and extrapolates) between bands
// for a given live supply pressure.
//
// Use case: valve opening depends on supply pressure, so a single curve
// is insufficient.  Multiple sweeps at different supply pressures give
// a 2D flow surface Q = f(V%, Psupply).

class PressureBandedTable {
public:
    static const uint8_t MAX_BANDS = 4;

    PressureBandedTable();

    /// Load one pressure band.  Bands must be loaded in ascending pressure order.
    /// points: V% → Flow table for this reference pressure.
    void loadBand(uint8_t bandIndex, float refPressure,
                  const LookupPoint* points, uint8_t count);

    /// Forward lookup: given V% and current supply pressure, return expected flow.
    float lookup(float valvePercent, float supplyPressure) const;

    /// Inverse lookup: given desired flow and current supply pressure, return V%.
    /// Interpolates between pressure bands.  Extrapolates outside the range.
    float inverseLookup(float desiredFlow, float supplyPressure) const;

    uint8_t getBandCount() const { return _bandCount; }

private:
    PiecewiseLinearTable _flowTables[MAX_BANDS];   // V% → Flow
    float _refPressure[MAX_BANDS];                  // Reference supply pressure per band
    uint8_t _bandCount;

    /// Find bracketing bands and interpolation weight t ∈ [0,1]
    /// Extrapolates: t<0 if below lowest, t>1 if above highest
    void findBrackets(float pressure, uint8_t& lo, uint8_t& hi, float& t) const;
};


// ============================================================================
// PI Controller with Anti-Windup Tracking
// ============================================================================
// For override control: the inactive controller's integrator is "tracked"
// toward the active controller's output to prevent windup.

struct PIConfig {
    float kp;               // Proportional gain
    float ki;               // Integral gain (per second)
    float outputMin;        // Output lower clamp
    float outputMax;        // Output upper clamp
    float trackingRate;     // Anti-windup tracking time constant (1/s), 0 = disabled
};

struct PIState {
    float integrator;
    float lastOutput;
    float error;
    bool  isActive;         // true if this controller is the one currently driving output
};

class PIController {
public:
    PIController();

    void configure(const PIConfig& config);
    PIConfig getConfig() const { return _config; }

    /// Reset integrator state
    void reset();

    /// Run one PI step.  dt in seconds.
    /// Returns unclamped output (caller does min-select and then calls trackOutput).
    float update(float setpoint, float measurement, float dt);

    /// After min-select: tell this controller what the actual output is,
    /// so the integrator can track (anti-windup).  active=true if this
    /// controller "won" the min-select.
    void trackOutput(float actualOutput, bool active);

    PIState getState() const { return _state; }

private:
    PIConfig _config;
    PIState  _state;
};


// ============================================================================
// Inspiratory Valve Controller (one per valve: air or O2)
// ============================================================================
// Parallel override control:
//   - Flow PI controller:     error = flowSetpoint - flowMeasured  → output = current (A)
//   - Pressure PI controller: error = pressureLimit - pressureMeas → output = current (A)
//   - Final current = min(flowPI_output, pressurePI_output)
//
// Plus feedforward from valve linearization:
//   I_ff = f(desiredFlow, supplyPressure)  from lookup table
//   I_total = I_ff + I_correction (from active PI)

struct InspValveConfig {
    PIConfig flowPI;
    PIConfig pressurePI;
    float    maxCurrent_A;      // Hardware current limit
    float    minCurrent_A;      // Minimum (usually 0)
    float    crackBaseOffset_A;     // Cracking current at zero supply pressure (spring only)
    float    crackPressCoeff_A_per_bar; // Additional cracking current per bar of supply pressure
    bool     useFeedforward;    // Enable valve linearization feedforward
};

struct InspValveMeasurements {
    float flow_slm;             // Measured flow through this valve
    float supplyPressure_mbar;  // Upstream supply pressure
    float airwayPressure_mbar;  // Downstream (patient) pressure
};

struct InspValveSetpoints {
    float flow_slm;             // Desired flow
    float pressureLimit_mbar;   // Max allowed airway pressure
};

struct InspValveStatus {
    float outputCurrent_A;
    float feedforwardCurrent_A;
    float flowPIOutput_A;
    float pressurePIOutput_A;
    bool  pressureLimiting;     // true = pressure PI is driving (flow request would exceed pressure)
    bool  active;               // true = valve should be commanding
};

class InspValveController {
public:
    InspValveController();

    void begin(const InspValveConfig& config);
    void reset();

    /// Set the Cv linearization table (DEPRECATED — use setFlowSurface).
    void setCvTable(const LookupPoint* points, uint8_t count);

    /// Set a pressure-banded flow surface for feedforward.
    /// bandIndex 0..3, loaded in ascending pressure order.
    void setFlowBand(uint8_t bandIndex, float refPressure,
                     const LookupPoint* points, uint8_t count);

    /// Get the flow surface (for status/debug)
    const PressureBandedTable& getFlowSurface() const { return _flowSurface; }

    /// Main update.  dt in seconds.
    /// Returns valve current command (A).
    float update(const InspValveSetpoints& sp, const InspValveMeasurements& meas, float dt);

    InspValveStatus getStatus() const { return _status; }

    // Direct access to PI controllers for tuning
    PIController& flowPI() { return _flowPI; }
    PIController& pressurePI() { return _pressurePI; }

private:
    InspValveConfig   _config;
    InspValveStatus   _status;
    PIController      _flowPI;
    PIController      _pressurePI;
    PiecewiseLinearTable _cvTable;        // Cv(I) for feedforward (legacy)
    PressureBandedTable   _flowSurface;   // V%(Flow, Psupply) for 2D feedforward

    /// Compute feedforward V% from desired flow and supply pressure.
    /// Uses pressure-banded flow surface if loaded, falls back to Cv table.
    float computeFeedforward(float desiredFlow_slm, float supplyPressure_mbar,
                             float downstreamPressure_mbar) const;
};


// ============================================================================
// Expiratory Valve Controller
// ============================================================================
// Single-mode pressure control:
//   - Pressure PI: error = PEEP_setpoint - measured Paw → current
//   - Feedforward from pressure→current lookup table
//
// The exp valve typically operates in reverse: more current = more closed
// (or more current = higher holding pressure, depending on valve type).
// The lookup table handles this.

struct ExpValveConfig {
    PIConfig pressurePI;
    float    maxCurrent_A;
    float    minCurrent_A;
    float    crackBaseOffset_A;     // Cracking current (spring only, no supply pressure dependency)
    bool     useFeedforward;
};

struct ExpValveStatus {
    float outputCurrent_A;
    float feedforwardCurrent_A;
    float pressurePIOutput_A;
    bool  active;
};

class ExpValveController {
public:
    ExpValveController();

    void begin(const ExpValveConfig& config);
    void reset();

    /// Set pressure-to-current linearization table
    /// Points: x = pressure (mbar), y = current (A)
    void setPressureTable(const LookupPoint* points, uint8_t count);

    /// Main update.  dt in seconds.
    /// pressureSetpoint_mbar = desired PEEP or pop-off pressure
    /// measuredPressure_mbar = measured airway pressure
    /// Returns valve current command (A).
    float update(float pressureSetpoint_mbar, float measuredPressure_mbar, float dt);

    /// Force fully closed (returns max current for closed valve, or 0 if normally-closed)
    float forceClose();

    /// Force fully open (for passive expiration)
    float forceOpen();

    ExpValveStatus getStatus() const { return _status; }

    PIController& pressurePI() { return _pressurePI; }

private:
    ExpValveConfig   _config;
    ExpValveStatus   _status;
    PIController     _pressurePI;
    PiecewiseLinearTable _pressureTable;  // pressure → current
};


// ============================================================================
// Local Valve Controller — Top-Level Coordinator
// ============================================================================
// Manages all 3 valves.  Sits between VentilatorController (HLC) outputs
// and SerialMuxRouter (sends current commands to valve drivers).
//
// The VentilatorController outputs:
//   - airValveFlow_slm, o2ValveFlow_slm (inspiratory flow setpoints)
//   - expValveSetpoint (pressure or position), expValveClosed, expValveAsPressure
//   - maxPressure_mbar (airway pressure limit)
//
// This controller converts those to valve currents.

struct LocalValveControllerConfig {
    InspValveConfig airValve;
    InspValveConfig o2Valve;
    ExpValveConfig  expValve;
    uint8_t airMuxChannel;      // MUX address for air valve (default MUX_AIR_VALVE=1)
    uint8_t o2MuxChannel;       // MUX address for O2 valve  (default MUX_O2_VALVE=2)
    uint8_t expMuxChannel;      // MUX address for exp valve (default MUX_EXP_VALVE=3)
};

struct LocalValveControllerMeasurements {
    // Inspiratory (from SFM3505 + ABP2 + ELVH)
    float airFlow_slm;              // Bus 0 flow
    float o2Flow_slm;               // Bus 1 flow (or same if shared)
    float airSupplyPressure_mbar;   // Bus 0 ABP2
    float o2SupplyPressure_mbar;    // Bus 1 ABP2
    float airwayPressure_mbar;      // ELVH (Paw)
};

struct LocalValveControllerSetpoints {
    // From VentilatorController outputs
    float airFlow_slm;
    float o2Flow_slm;
    float maxPressure_mbar;     // Inspiratory pressure limit

    // Expiratory
    float expPressure_mbar;     // PEEP or pop-off setpoint
    bool  expValveClosed;       // Force exp valve closed
    bool  expValveActive;       // Enable exp valve pressure control
};

class LocalValveController {
public:
    LocalValveController();

    void begin(SerialMuxRouter* muxRouter, const LocalValveControllerConfig& config);

    /// Set default placeholder configs (for initial commissioning)
    void setDefaults();

    /// Reset all controllers (call when ventilator starts/stops)
    void reset();

    /// Enable/disable the local loop controllers.
    /// When disabled, setpoints are passed through as flow commands to MUX
    /// (original behavior for when downstream controllers exist).
    void setEnabled(bool enabled) { _enabled = enabled; }
    bool isEnabled() const { return _enabled; }

    /// Main update — call at control rate (100 Hz).
    /// Reads setpoints, measurements, computes valve currents, sends to MUX.
    void update(const LocalValveControllerSetpoints& sp,
                const LocalValveControllerMeasurements& meas,
                float dt);

    // Status getters
    InspValveStatus getAirValveStatus() const { return _airValve.getStatus(); }
    InspValveStatus getO2ValveStatus() const  { return _o2Valve.getStatus(); }
    ExpValveStatus  getExpValveStatus() const  { return _expValve.getStatus(); }

    // Direct access for tuning
    InspValveController& airValve() { return _airValve; }
    InspValveController& o2Valve()  { return _o2Valve; }
    ExpValveController&  expValve() { return _expValve; }

private:
    SerialMuxRouter*   _muxRouter;
    bool               _enabled;

    InspValveController _airValve;
    InspValveController _o2Valve;
    ExpValveController  _expValve;

    uint8_t _airMuxChannel;
    uint8_t _o2MuxChannel;
    uint8_t _expMuxChannel;

    /// Send current command to a valve via MUX
    void sendValveCurrent(uint8_t channel, float current_A);
};

#endif // LOCAL_VALVE_CONTROLLER_HPP
