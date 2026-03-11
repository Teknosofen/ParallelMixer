#ifndef VALVE_CHARACTERIZER_HPP
#define VALVE_CHARACTERIZER_HPP

#include <Arduino.h>

// Forward declarations
class SerialMuxRouter;
class ActuatorControl;
struct SensorData;

// ============================================================================
// Valve / Blower / Exp Characterizer — Automated sweep for actuator mapping
// ============================================================================
// Ramps control signal (V% or PWM%) from 0 to max in small steps on a selected
// channel, recording flow and pressure at each step.
//
// Three characterization modes:
//   ch 1,2 (Insp valves): Sweep V% → measure flow + Psupply.
//       Output: V% → Flow table at measured Psupply (for PressureBandedTable).
//       Run at different supply pressures to build 2D surface.
//
//   ch 4 (Blower, GPIO21 PWM): Sweep PWM% → measure flow (bus0) + Paw (ELVH).
//       Output: PWM% → Flow table at measured counter pressure.
//       Run at different counter pressures/resistances to build 2D surface.
//
//   ch 3 (Exp valve): Sweep V% → measure Paw (ELVH) + flow (bus0).
//       Output: V% → Paw table at measured flow.
//       Run at different flows (set via LF) to build 2D surface.
//
// Usage:
//   CC<channel>[,maxV[,stepV[,settleMs]]]   Start characterization
//   CX                                       Abort characterization

enum CharMode {
    CHARMODE_INSP_VALVE = 0,    // ch 1,2: V% → flow (at Psupply)
    CHARMODE_BLOWER,            // ch 4:   PWM% → flow (at counter pressure)
    CHARMODE_EXP_VALVE          // ch 3:   V% → Paw (at flow)
};

struct CharacterizationConfig {
    uint8_t  muxChannel;        // 1=air, 2=O2, 3=exp, 4=blower(PWM)
    float    maxVoltage;        // Max control signal (V% or PWM%)
    float    stepVoltage;       // Control signal increment per step
    uint32_t settleTime_ms;     // Wait time after setting control signal (ms)
    uint8_t  samplesPerStep;    // Number of sensor readings to average
};

struct CharacterizationPoint {
    float voltage_V;
    float flow_slm;
    float supplyPressure_kPa;   // ABP2 returns kPa
    float pawPressure_mbar;     // ELVH returns mbar
    float cv;                   // Computed Cv = flow / sqrt(deltaP_kPa)
};

enum CharState {
    CHAR_IDLE = 0,
    CHAR_RAMP_SETTLE,       // Waiting for flow to stabilize after voltage change
    CHAR_RAMP_SAMPLE,       // Accumulating sensor samples
    CHAR_DONE               // Sweep complete
};

class ValveCharacterizer {
public:
    ValveCharacterizer();

    /// Attach the MUX router and optional blower actuator (for direct PWM)
    void begin(SerialMuxRouter* muxRouter, ActuatorControl* blowerActuator = nullptr);

    /// Start a characterization sweep.  Returns false if already running or invalid channel.
    bool start(const CharacterizationConfig& config);

    /// Abort a running sweep, set output to zero
    void abort();

    /// Call every main-loop iteration.  Needs current sensor data from both buses.
    /// Returns true while characterization is in progress.
    bool update(const SensorData& bus0, const SensorData& bus1, float pawPressure_mbar);

    bool isRunning() const { return _state != CHAR_IDLE; }

    /// Return MUX channel being characterized (1-4), or 0 if idle
    uint8_t getActiveChannel() const { return isRunning() ? _config.muxChannel : 0; }

    CharState getState() const { return _state; }

private:
    SerialMuxRouter*       _muxRouter;
    ActuatorControl*       _blowerActuator;   // For GPIO21 direct PWM (ch4)
    CharacterizationConfig _config;
    CharMode               _charMode;
    CharState              _state;

    // Sweep state
    float    _currentVoltage;
    uint16_t _currentStep;
    uint16_t _totalSteps;
    uint32_t _stateEntryTime_ms;

    // Sampling accumulator
    float    _sumFlow;
    float    _sumPressure;
    float    _sumPaw;
    uint8_t  _sampleCount;

    // Data point count (for table output at end)
    uint16_t _pointCount;
    static const uint16_t MAX_POINTS = 200;
    CharacterizationPoint _points[MAX_POINTS];

    /// Get flow reading for the configured channel
    float getFlow(const SensorData& bus0, const SensorData& bus1) const;

    /// Get supply pressure reading for the configured channel
    float getSupplyPressure(const SensorData& bus0, const SensorData& bus1) const;

    /// Set control signal on the configured channel (V% for MUX, PWM% for blower)
    void setOutput(float percent);

    /// Print a single data row (format depends on mode)
    void printDataRow(const CharacterizationPoint& pt);

    /// Print the final table in code-ready format (format depends on mode)
    void printSummaryTable();

    /// Print the header at start
    void printHeader();
};

#endif // VALVE_CHARACTERIZER_HPP
