#include "ActuatorControl.hpp"
#include "math.h"

ActuatorControl::ActuatorControl(uint8_t valve_ctrl_pin)
  : _valve_ctrl_pin(valve_ctrl_pin),
    _controller_mode(VALVE_SET_VALUE_CONTROL),
    _external_pwm(true),
    _index_in_period(0),
    _valve_signal_externally_set(0),
    _valve_signal_generated(0) {
  
  // Default PID configuration
  _pid_config.p_gain = 1.0;
  _pid_config.i_gain = 1.0;
  _pid_config.d_gain = 0.0;
  _pid_config.valve_offset = 0.0;
  
  // Default signal generator configuration (percentage-based)
  _sig_gen_config.offset = 25.0;      // 25% (was 1024/4095)
  _sig_gen_config.amplitude = 6.25;   // 6.25% (was 256/4095)
  _sig_gen_config.samples_per_period = 256;
  
  // Reset control state
  resetPIDState();
}

void ActuatorControl::initialize() {
  // Nothing specific needed for initialization
}

void ActuatorControl::setControllerMode(ControllerMode mode) {
  _controller_mode = mode;
  _index_in_period = 0;  // Reset period counter on mode change
}

ControllerMode ActuatorControl::getControllerMode() const {
  return _controller_mode;
}

void ActuatorControl::setPIDConfig(const PIDConfig& config) {
  _pid_config = config;
}

PIDConfig ActuatorControl::getPIDConfig() const {
  return _pid_config;
}

void ActuatorControl::resetPIDState() {
  _control_state.error = 0.0;
  _control_state.integrator = 0.0;
  _control_state.output = 0.0;
  _control_state.valve_signal = 0;
}

uint16_t ActuatorControl::updatePID(float flow_reference, float flow_measured) {
  _control_state.error = flow_reference - flow_measured;
  _control_state.integrator += _control_state.error;
  
  _control_state.output = _pid_config.p_gain * _control_state.error + 
                          _pid_config.i_gain * _control_state.integrator;
  
  uint16_t valve_signal = uint16_t(_control_state.output + _pid_config.valve_offset);
  
  // Clamp to valid range
  if (valve_signal > 4095) valve_signal = 4095;
  
  // Zero output if reference is zero
  if (flow_reference == 0) {
    valve_signal = 0;
    _control_state.integrator = 0;
  }
  
  _control_state.valve_signal = valve_signal;
  outputToValve(valve_signal);
  
  return valve_signal;
}

void ActuatorControl::setSignalGeneratorConfig(const SignalGeneratorConfig& config) {
  _sig_gen_config = config;
  _index_in_period = 0;  // Reset period counter on config change
}

SignalGeneratorConfig ActuatorControl::getSignalGeneratorConfig() const {
  return _sig_gen_config;
}

uint16_t ActuatorControl::updateSignalGenerator() {
  _index_in_period++;
  if (_index_in_period >= _sig_gen_config.samples_per_period) {
    _index_in_period = 0;
  }
  
  uint16_t signal = 0;
  
  switch (_controller_mode) {
    case SINE_CONTROL:
      signal = generateSine();
      break;
    case STEP_CONTROL:
      signal = generateStep();
      break;
    case TRIANGLE_CONTROL:
      signal = generateTriangle();
      break;
    default:
      signal = 0;
      break;
  }
  
  _valve_signal_generated = signal;
  outputToValve(signal);
  
  return signal;
}

uint16_t ActuatorControl::generateSine() {
  // Calculate sine wave: offset ± amplitude/2
  // Formula generates values from offset-amplitude/2 to offset+amplitude/2
  float signal_percent = _sig_gen_config.amplitude / 2.0 *
                         (1.0 + sin(6.28 * (float)_index_in_period / (float)_sig_gen_config.samples_per_period));
  float total_percent = signal_percent + _sig_gen_config.offset;

  // Convert percentage to hardware units
  return percentToHardware(total_percent);
}

uint16_t ActuatorControl::generateStep() {
  // Generate step wave alternating between two levels
  float signal_percent;

  if (_index_in_period <= _sig_gen_config.samples_per_period / 2) {
    signal_percent = _sig_gen_config.offset + _sig_gen_config.amplitude;
  } else {
    signal_percent = _sig_gen_config.offset;
  }

  // Convert percentage to hardware units
  return percentToHardware(signal_percent);
}

uint16_t ActuatorControl::generateTriangle() {
  float signal_percent;

  if (_index_in_period < (float)_sig_gen_config.samples_per_period / 2.0) {
    // Rising ramp
    signal_percent = _sig_gen_config.offset +
                     _sig_gen_config.amplitude * (_index_in_period * 2.0 / (float)_sig_gen_config.samples_per_period);
  } else {
    // Falling ramp
    signal_percent = _sig_gen_config.offset +
                     _sig_gen_config.amplitude *
                     (1 - (_index_in_period * 2.0 - (float)_sig_gen_config.samples_per_period) / (float)_sig_gen_config.samples_per_period);
  }

  // Convert percentage to hardware units
  return percentToHardware(signal_percent);
}

void ActuatorControl::setValveControlSignal(float percent) {
  // Convert percentage to hardware units and store
  uint16_t signal = percentToHardware(percent);
  _valve_signal_externally_set = signal;

  if (_controller_mode == VALVE_SET_VALUE_CONTROL) {
    outputToValve(signal);
  }
}

float ActuatorControl::getValveControlSignal() const {
  // Return current valve signal as percentage
  uint16_t hardware_value;

  if (_controller_mode == VALVE_SET_VALUE_CONTROL) {
    hardware_value = _valve_signal_externally_set;
  } else if (_controller_mode == PID_CONTROL) {
    hardware_value = _control_state.valve_signal;
  } else {
    hardware_value = _valve_signal_generated;
  }

  return hardwareToPercent(hardware_value);
}

void ActuatorControl::setExternalPWM(bool external) {
  _external_pwm = external;
}

bool ActuatorControl::isExternalPWM() const {
  return _external_pwm;
}

ControlState ActuatorControl::getControlState() const {
  return _control_state;
}

void ActuatorControl::outputToValve(uint16_t signal) {
  if (_external_pwm) {
    analogOutMCP4725(signal);
  } else {
    analogWrite(_valve_ctrl_pin, signal);
  }
}

void ActuatorControl::analogOutMCP4725(uint16_t dac_output) {
  Wire.beginTransmission(I2Cadr_MCP4725);
  Wire.write(64);
  Wire.write(dac_output >> 4);
  Wire.write((dac_output & 15) << 4);
  Wire.endTransmission();
}

void ActuatorControl::execute(float flow_reference, float flow_measured, int quiet_mode) {
  switch (_controller_mode) {
    case PID_CONTROL:
      updatePID(flow_reference, flow_measured);
      break;
      
    case VALVE_SET_VALUE_CONTROL:
      // Valve signal is already set, nothing to do except output if needed
      if (quiet_mode == 3) {
        Serial.print("Set value ");
        Serial.println(getValveControlSignal());
      }
      break;
      
    case SINE_CONTROL:
      updateSignalGenerator();
      if (quiet_mode == 3) {
        Serial.print("Sine ");
        Serial.println(getValveControlSignal());
      }
      break;
      
    case STEP_CONTROL:
      updateSignalGenerator();
      if (quiet_mode == 3) {
        Serial.print("pulse ");
        Serial.println(getValveControlSignal());
      }
      break;
      
    case TRIANGLE_CONTROL:
      updateSignalGenerator();
      if (quiet_mode == 3) {
        Serial.print("Triangle ");
        Serial.println(getValveControlSignal());
      }
      break;
  }
}

// ============================================================================
// Conversion Helpers - Hardware Abstraction
// ============================================================================
// These methods encapsulate knowledge about the hardware:
// - 12-bit DAC resolution (0-4095 for MCP4725)
// - Internal Arduino PWM (0-255 for 8-bit, 0-4095 for 12-bit)
// The rest of the code works with percentages (0.0-100.0%)
// ============================================================================

uint16_t ActuatorControl::percentToHardware(float percent) const {
  // Clamp to valid range
  if (percent < 0.0) percent = 0.0;
  if (percent > 100.0) percent = 100.0;

  // Convert percentage to 12-bit hardware value (0-4095)
  return (uint16_t)((percent * 4095.0) / 100.0);
}

float ActuatorControl::hardwareToPercent(uint16_t hardware) const {
  // Convert 12-bit hardware value (0-4095) to percentage
  return (hardware * 100.0) / 4095.0;
}

// ============================================================================
// Serial Actuator Communication (Serial1)
// ============================================================================
// Protocol: <CHR>FLOAT\n
// Example: "V50.5\n" - Set valve to 50.5%
// Example: "P123.4\n" - Pressure reading 123.4 kPa
// ============================================================================

bool ActuatorControl::sendSerialCommand(char command, float value) {
  // Send command: <CHR>FLOAT\n
  // Note: Caller is responsible for value validation/limiting
  String cmd = String(command) + String(value, 2) + "\n";
  Serial1.print(cmd);

  // Optional: Add debug output
  #ifdef DEBUG_SERIAL_ACTUATOR
  Serial.printf("[ActuatorControl] Sent to Serial1: %s", cmd.c_str());
  #endif

  return true;
}

bool ActuatorControl::readSerialResponse(char& command, float& value, uint32_t timeout_ms) {
  // Read response from Serial1 with timeout
  // Expected format: <CHR>FLOAT\n

  uint32_t start_time = millis();
  String response = "";

  while (millis() - start_time < timeout_ms) {
    if (Serial1.available()) {
      char c = Serial1.read();

      if (c == '\n') {
        // End of message, parse it
        if (response.length() >= 2) {
          command = response[0];
          value = response.substring(1).toFloat();

          #ifdef DEBUG_SERIAL_ACTUATOR
          Serial.printf("[ActuatorControl] Received from Serial1: %c%.2f\n", command, value);
          #endif

          return true;
        }
        return false;  // Malformed message
      } else {
        response += c;

        // Prevent buffer overflow
        if (response.length() > 32) {
          return false;
        }
      }
    }

    // Small delay to prevent tight polling
    delayMicroseconds(100);
  }

  // Timeout occurred
  #ifdef DEBUG_SERIAL_ACTUATOR
  Serial.println("[ActuatorControl] Serial1 read timeout");
  #endif

  return false;
}

bool ActuatorControl::readSerialMeasurement(char& command, float& value, uint32_t timeout_ms) {
  // Alias for readSerialResponse - same protocol for measurements
  // Kept as separate method for semantic clarity
  return readSerialResponse(command, value, timeout_ms);
}

void ActuatorControl::clearSerialBuffer() {
  // Clear any pending data in Serial1 buffer
  while (Serial1.available()) {
    Serial1.read();
  }
}