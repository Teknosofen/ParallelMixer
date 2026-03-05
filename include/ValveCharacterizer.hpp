#ifndef VALVE_CHARACTERIZER_HPP
#define VALVE_CHARACTERIZER_HPP

#include <Arduino.h>

// Forward declarations
class SerialMuxRouter;
struct SensorData;

// ============================================================================
// Valve Characterizer — Automated V% sweep for inspiratory valve mapping
// ============================================================================
// Ramps V% from 0 to maxVoltage in small steps on a selected MUX channel,
// recording flow and supply pressure at each step.  Output is a raw flow table
// (V% → Flow at measured Psupply) for use in PressureBandedTable.
//
// Run multiple sweeps at different supply pressures to populate 2-3 bands.
//
// Usage:
//   CC<channel>[,maxV[,stepV[,settleMs]]]   Start characterization
//   CX                                       Abort characterization
//
// Only MUX channels 1 (air) and 2 (O2) are supported.
//   Channel 1 → Bus 0 sensors (sfm3505_air_flow, supply_pressure)
//   Channel 2 → Bus 1 sensors (sfm3505_air_flow, supply_pressure)

struct CharacterizationConfig {
    uint8_t  muxChannel;        // 1 = air, 2 = O2
    float    maxVoltage;        // Max voltage to ramp to (V)
    float    stepVoltage;       // Voltage increment per step (V)
    uint32_t settleTime_ms;     // Wait time after setting voltage (ms)
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

    /// Attach the MUX router used to send voltage commands
    void begin(SerialMuxRouter* muxRouter);

    /// Start a characterization sweep.  Returns false if already running or invalid channel.
    bool start(const CharacterizationConfig& config);

    /// Abort a running sweep, set voltage to zero
    void abort();

    /// Call every main-loop iteration.  Needs current sensor data from both buses.
    /// Returns true while characterization is in progress.
    bool update(const SensorData& bus0, const SensorData& bus1, float pawPressure_mbar);

    bool isRunning() const { return _state != CHAR_IDLE; }

    CharState getState() const { return _state; }

private:
    SerialMuxRouter*       _muxRouter;
    CharacterizationConfig _config;
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

    /// Set voltage on the configured MUX channel
    void setVoltage(float voltage);

    /// Print a single data row
    void printDataRow(const CharacterizationPoint& pt);

    /// Print the final Cv table in code-ready format
    void printCvTable();

    /// Print the header at start
    void printHeader();
};

#endif // VALVE_CHARACTERIZER_HPP
