#ifndef VENTILATOR_CONTROLLER_HPP
#define VENTILATOR_CONTROLLER_HPP

#include <Arduino.h>

// Forward declarations
class SerialMuxRouter;

// ============================================================================
// Ventilator States
// ============================================================================
enum VentilatorState {
    VENT_OFF = 0,
    // Inspiration group
    VENT_INSP_PHASE1,       // Active insp, exp valve CLOSED, flow controls pressure
    VENT_INSP_PHASE2,       // Active insp, exp valve as POP-OFF for pressure limit
    VENT_INSP_PAUSE,        // Inspiratory hold, no flow, valves closed
    // Expiration group
    VENT_EXP_NON_TRIG,      // Early exp, no triggering allowed (anti self-trigger)
    VENT_EXP_SYNC_WAIT      // Late exp, waiting for trigger (SIMV window)
};

// ============================================================================
// Ventilator Configuration
// ============================================================================
struct VentilatorConfig {
    // Timing
    float respRate;             // BPM (default 12)
    float ieRatio;              // I:E as decimal (default 0.5 = 1:2)
    float inspPauseFraction;    // 0-0.5 (default 0.1)
    float insp1Fraction;        // 0-1 (default 1.0)
    float insp2Fraction;        // 0-1 (default 0 = disabled)
    float expNonTrigFraction;   // 0-1 (default 0.5)
    float expSyncFraction;      // 0-1 (default 0 = disabled)

    // Volume/Flow
    float tidalVolume_mL;       // Target Vt (default 500)
    float maxInspFlow_slm;      // Max flow limit (default 60)
    float totalFlow_slm;        // Direct flow setting (default 30)
    bool useVolumeControl;      // true = calc from Vt, false = use totalFlow

    // Pressure
    float maxPressure_mbar;     // PIP limit (default 30)
    float peep_mbar;            // PEEP (default 5)
    float pressureRampTime_ms;  // Rise time (default 100)

    // Gas
    float targetFiO2;           // 0.21-1.0 (default 0.21)

    // Trigger
    float biasFlow_slm;         // Continuous exp flow (default 2)
    float flowTrigger_slm;      // Flow above bias to trigger (default 2)
    float pressureTrigger_mbar; // Pressure drop to trigger (default 2)
    bool triggerEnabled;        // Enable triggering (default false)

    // Alarms
    float highPressureAlarm_mbar;   // default 40
    float lowPressureAlarm_mbar;    // default 3
    float apneaTime_s;              // default 20
};

// ============================================================================
// Ventilator Measurements (input from sensors)
// ============================================================================
struct VentilatorMeasurements {
    float airwayPressure_mbar;
    float inspFlow_slm;
    float expFlow_slm;
    float deliveredO2_percent;
};

// ============================================================================
// Ventilator Outputs (setpoints for LLC/actuators)
// ============================================================================
struct VentilatorOutputs {
    float airValveFlow_slm;     // Target flow for air valve (MUX 1)
    float o2ValveFlow_slm;      // Target flow for O2 valve (MUX 2)
    float expValveSetpoint;     // Exp valve: pressure setpoint or position
    bool expValveAsPressure;    // true = pressure mode, false = position mode
    bool expValveClosed;        // true = force closed (during insp1)
};

// ============================================================================
// Ventilator Status
// ============================================================================
struct VentilatorStatus {
    VentilatorState state;
    uint32_t breathCount;
    float cycleProgress;        // 0.0 - 1.0
    float calculatedFlow_slm;
    float actualSetFlow_slm;
    float peakPressure_mbar;
    float plateauPressure_mbar;
    float measuredVt_mL;
    bool pressureLimited;
    bool flowLimited;
    bool triggered;
};

// ============================================================================
// Ventilator Controller Class (HLC - High Level Controller)
// ============================================================================
class VentilatorController {
public:
    VentilatorController();

    // Lifecycle
    void begin(SerialMuxRouter* muxRouter);
    void start();
    void stop();
    bool isRunning() const { return _running; }

    // Main update - call from loop
    void update(const VentilatorMeasurements& meas, uint32_t currentTime_us);

    // Configuration
    void setConfig(const VentilatorConfig& config);
    VentilatorConfig getConfig() const { return _config; }
    void setDefaultConfig();

    // Individual parameter setters
    void setRespRate(float bpm);
    void setIERatio(float ratio);
    void setTidalVolume(float mL);
    void setMaxFlow(float slm);
    void setTotalFlow(float slm);
    void setUseVolumeControl(bool use);
    void setMaxPressure(float mbar);
    void setPEEP(float mbar);
    void setPressureRampTime(float ms);
    void setFiO2(float fraction);
    void setInspPauseFraction(float frac);
    void setInsp1Fraction(float frac);
    void setInsp2Fraction(float frac);
    void setExpNonTrigFraction(float frac);
    void setExpSyncFraction(float frac);
    void setBiasFlow(float slm);
    void setFlowTrigger(float slm);
    void setPressureTrigger(float mbar);
    void setTriggerEnabled(bool enabled);

    // Status & Outputs
    VentilatorStatus getStatus() const { return _status; }
    VentilatorOutputs getOutputs() const { return _outputs; }
    VentilatorState getState() const { return _status.state; }
    const char* getStateString() const;

    // Alarms
    bool hasAlarm() const { return _alarmFlags != 0; }
    uint8_t getAlarmFlags() const { return _alarmFlags; }
    void clearAlarms() { _alarmFlags = 0; }

    // Alarm bit flags
    static const uint8_t ALARM_HIGH_PRESSURE  = 0x01;
    static const uint8_t ALARM_LOW_PRESSURE   = 0x02;
    static const uint8_t ALARM_APNEA          = 0x04;
    static const uint8_t ALARM_LOW_VT         = 0x08;

private:
    VentilatorConfig _config;
    VentilatorStatus _status;
    VentilatorOutputs _outputs;
    SerialMuxRouter* _muxRouter;
    bool _running;
    uint8_t _alarmFlags;

    // Timing (calculated from config, in microseconds)
    uint32_t _cycleTime_us;
    uint32_t _inspTotalTime_us;
    uint32_t _insp1Time_us;
    uint32_t _insp2Time_us;
    uint32_t _inspPauseTime_us;
    uint32_t _expTotalTime_us;
    uint32_t _expNonTrigTime_us;
    uint32_t _expSyncTime_us;

    // Cycle tracking
    uint32_t _cycleStartTime_us;
    uint32_t _stateStartTime_us;
    uint32_t _lastBreathTime_us;

    // Breath measurements
    float _breathPeakPressure;
    float _breathPlateauPressure;
    float _breathVolumeDelivered_mL;
    bool _breathWasTriggered;

    // Control variables
    float _targetTotalFlow_slm;
    float _currentSetFlow_slm;
    float _targetO2Flow_slm;
    float _targetAirFlow_slm;

    // Internal methods
    void recalculateTiming();
    void calculateGasFlows(float totalFlow);
    float calculateFlowFromVt() const;

    void startNewBreath(uint32_t currentTime_us);
    void transitionTo(VentilatorState newState, uint32_t currentTime_us);
    bool checkStateTimeout(uint32_t currentTime_us, uint32_t duration_us) const;
    bool checkTrigger(const VentilatorMeasurements& meas) const;

    void executeInspPhase1(const VentilatorMeasurements& meas);
    void executeInspPhase2(const VentilatorMeasurements& meas);
    void executeInspPause(const VentilatorMeasurements& meas);
    void executeExpNonTrig(const VentilatorMeasurements& meas);
    void executeExpSyncWait(const VentilatorMeasurements& meas);

    void updateAlarms(const VentilatorMeasurements& meas, uint32_t currentTime_us);
    void resetBreathMeasurements();
};

#endif // VENTILATOR_CONTROLLER_HPP
