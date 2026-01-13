#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <Arduino.h>
#include "Wire.h"

// Forward declaration
class SerialActuatorReader;

#define I2Cadr_MCP4725 0x60

enum ControllerMode {
  PID_CONTROL = 0,
  VALVE_SET_VALUE_CONTROL = 1,
  SINE_CONTROL = 2,
  STEP_CONTROL = 3,
  TRIANGLE_CONTROL = 4
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

struct ControlState {
  float error;
  float integrator;
  float output;
  uint16_t valve_signal;
};

class ActuatorControl {
public:
  ActuatorControl(uint8_t valve_ctrl_pin);
  
  void initialize();
  
  // Control mode management
  void setControllerMode(ControllerMode mode);
  ControllerMode getControllerMode() const;
  
  // PID control
  void setPIDConfig(const PIDConfig& config);
  PIDConfig getPIDConfig() const;
  void resetPIDState();
  uint16_t updatePID(float flow_reference, float flow_measured);
  
  // Signal generator control
  void setSignalGeneratorConfig(const SignalGeneratorConfig& config);
  SignalGeneratorConfig getSignalGeneratorConfig() const;
  uint16_t updateSignalGenerator();
  
  // Direct valve control (percentage-based interface)
  void setValveControlSignal(float percent);  // 0.0-100.0%
  float getValveControlSignal() const;        // Returns 0.0-100.0%
  
  // Output method selection
  void setExternalPWM(bool external);
  bool isExternalPWM() const;

  // Get current control state
  ControlState getControlState() const;

  // Serial actuator communication (Serial1) - forwarded to SerialActuatorReader
  // These methods are kept for backward compatibility
  // Note: Set the serial reader pointer before using these methods
  void setSerialActuatorReader(SerialActuatorReader* reader);
  bool sendSerialCommand(char command, float value);
  bool readSerialResponse(char& command, float& value, uint32_t timeout_ms = 100);
  bool readSerialMeasurement(char& command, float& value, uint32_t timeout_ms = 100);
  void clearSerialBuffer();

  // Main execution method - call this from the main loop
  void execute(float flow_reference, float flow_measured, int quiet_mode);
  
private:
  uint8_t _valve_ctrl_pin;
  ControllerMode _controller_mode;
  bool _external_pwm;

  // PID state
  PIDConfig _pid_config;
  ControlState _control_state;

  // Signal generator state
  SignalGeneratorConfig _sig_gen_config;
  uint16_t _index_in_period;
  uint32_t _period_start_time_us;  // Microseconds timestamp of period start
  uint16_t _valve_signal_externally_set;
  uint16_t _valve_signal_generated;

  // Serial actuator reader pointer (for forwarding serial calls)
  SerialActuatorReader* _serialActuatorReader;

  // Output methods
  void outputToValve(uint16_t signal);
  void analogOutMCP4725(uint16_t dac_output);

  // Signal generators (phase is 0.0 to 1.0)
  uint16_t generateSine(float phase);
  uint16_t generateStep(float phase);
  uint16_t generateTriangle(float phase);

  // Conversion helpers - hardware abstraction
  // These methods encapsulate knowledge about hardware resolution (12-bit = 0-4095)
  uint16_t percentToHardware(float percent) const;
  float hardwareToPercent(uint16_t hardware) const;
};

#endif