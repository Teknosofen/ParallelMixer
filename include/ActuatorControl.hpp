#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <Arduino.h>
#include "Wire.h"

// Forward declaration
class SerialMuxRouter;

#define I2Cadr_MCP4725 0x60

enum ControllerMode {
  PID_CONTROL = 0,
  VALVE_SET_VALUE_CONTROL = 1,
  SINE_CONTROL = 2,
  STEP_CONTROL = 3,
  TRIANGLE_CONTROL = 4,
  SWEEP_CONTROL = 5
};

struct PIDConfig {
  float p_gain;
  float i_gain;
  float d_gain;
  float valve_offset;
};

struct SignalGeneratorConfig {
  float offset;           // Offset as percentage (0.0-100.0%)
  float amplitude;        // Amplitude as percentage (0.0-100.0%)
  float period_seconds;   // Period time in seconds
};

struct SweepConfig {
  float start_freq;       // Start frequency in Hz
  float stop_freq;        // Stop frequency in Hz
  float sweep_time;       // Total sweep duration in seconds
  bool logarithmic;       // true = logarithmic sweep, false = linear sweep
};

struct ControlState {
  float error;
  float integrator;
  float output;
  float valve_signal;  // Percentage (0-100%)
};

class ActuatorControl {
public:
  ActuatorControl();

  void initialize();
  
  // Control mode management
  void setControllerMode(ControllerMode mode);
  ControllerMode getControllerMode() const;
  
  // PID control
  void setPIDConfig(const PIDConfig& config);
  PIDConfig getPIDConfig() const;
  void resetPIDState();
  float updatePID(float flow_reference, float flow_measured);  // Returns percentage (0-100%)
  
  // Signal generator control
  void setSignalGeneratorConfig(const SignalGeneratorConfig& config);
  SignalGeneratorConfig getSignalGeneratorConfig() const;
  float updateSignalGenerator();  // Returns percentage (0-100%)

  // Sweep generator control
  void setSweepConfig(const SweepConfig& config);
  SweepConfig getSweepConfig() const;
  float updateSweepGenerator();   // Returns percentage (0-100%)
  
  // Direct valve control (percentage-based interface)
  void setValveControlSignal(float percent);  // 0.0-100.0%
  float getValveControlSignal() const;        // Returns 0.0-100.0%
  
  // Output method selection
  void setExternalPWM(bool external);
  bool isExternalPWM() const;

  // Get current control state
  ControlState getControlState() const;

  // Serial MUX router communication
  // Note: Set the router pointer and address before using serial methods
  void setSerialMuxRouter(SerialMuxRouter* router, uint8_t muxAddress = 0);
  uint8_t getMuxAddress() const;

  // Send commands via MUX router
  bool sendSetCurrent(float value);
  bool sendSetVoltage(float value);
  bool sendSetFlow(float value);
  bool sendSetPressure(float value);
  bool sendSetBlowerRPM(float value);
  bool sendSerialCommand(char command, float value);  // Generic send

  // Get measurements from MUX router (non-blocking, uses cached data)
  float getRemoteCurrent() const;
  float getRemoteBlowerRPM() const;
  float getRemoteActualFlow() const;
  float getRemoteActualPressure() const;

  // Stale detection for remote data
  bool isRemoteCurrentStale(uint32_t max_age_ms = 100) const;
  bool isRemoteBlowerRPMStale(uint32_t max_age_ms = 100) const;
  bool isRemoteActualFlowStale(uint32_t max_age_ms = 100) const;
  bool isRemoteActualPressureStale(uint32_t max_age_ms = 100) const;

  // Main execution method - call this from the main loop
  void execute(float flow_reference, float flow_measured, int quiet_mode);
  
private:
  ControllerMode _controller_mode;
  bool _external_pwm;

  // PID state
  PIDConfig _pid_config;
  ControlState _control_state;

  // Signal generator state
  SignalGeneratorConfig _sig_gen_config;
  uint16_t _index_in_period;
  uint32_t _period_start_time_us;  // Microseconds timestamp of period start
  float _valve_signal_externally_set;  // Percentage (0-100%)
  float _valve_signal_generated;      // Percentage (0-100%)
  float _last_sent_valve_signal;      // Track last sent value to avoid redundant sends

  // Sweep generator state
  SweepConfig _sweep_config;
  uint32_t _sweep_start_time_us;      // Microseconds timestamp of sweep start
  float _sweep_phase;                  // Accumulated phase for sweep (0.0 to 1.0 per cycle)

  // Serial MUX router pointer and address
  SerialMuxRouter* _serialMuxRouter;
  uint8_t _muxAddress;

  // Output methods (now accepts percentage)
  void outputToValve(float signal_percent);
  void analogOutMCP4725(uint16_t dac_output);

  // Signal generators (phase is 0.0 to 1.0, returns percentage)
  float generateSine(float phase);
  float generateStep(float phase);
  float generateTriangle(float phase);
  float generateSweep();  // Frequency sweep using offset/amplitude

  // Conversion helpers - hardware abstraction
  // These methods encapsulate knowledge about hardware resolution (12-bit = 0-4095)
  uint16_t percentToHardware(float percent) const;
  float hardwareToPercent(uint16_t hardware) const;
};

#endif