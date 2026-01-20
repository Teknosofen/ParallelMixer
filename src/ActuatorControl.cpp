#include "ActuatorControl.hpp"
#include "SerialActuatorReader.hpp"
#include "math.h"

ActuatorControl::ActuatorControl(uint8_t valve_ctrl_pin)
  : _valve_ctrl_pin(valve_ctrl_pin),
    _controller_mode(VALVE_SET_VALUE_CONTROL),
    _external_pwm(true),
    _index_in_period(0),
    _period_start_time_us(0),
    _valve_signal_externally_set(0.0),
    _valve_signal_generated(0.0),
    _sweep_start_time_us(0),
    _sweep_phase(0.0),
    _serialActuatorReader(nullptr) {

  // Default PID configuration
  _pid_config.p_gain = 1.0;
  _pid_config.i_gain = 1.0;
  _pid_config.d_gain = 0.0;
  _pid_config.valve_offset = 0.0;

  // Default signal generator configuration (percentage-based)
  _sig_gen_config.offset = 25.0;      // 25% (was 1024/4095)
  _sig_gen_config.amplitude = 10.0;   // 10.0%
  _sig_gen_config.period_seconds = 1.0; // 1 second period

  // Default sweep configuration
  _sweep_config.start_freq = 0.1;     // 0.1 Hz
  _sweep_config.stop_freq = 10.0;     // 10 Hz
  _sweep_config.sweep_time = 10.0;    // 10 seconds
  _sweep_config.logarithmic = false;  // Linear sweep by default

  // Reset control state
  resetPIDState();
}

void ActuatorControl::initialize() {
  // Nothing specific needed for initialization
}

void ActuatorControl::setControllerMode(ControllerMode mode) {
  _controller_mode = mode;
  _index_in_period = 0;  // Reset period counter on mode change
  _period_start_time_us = 0;  // Reset period timer
  _sweep_start_time_us = 0;   // Reset sweep timer
  _sweep_phase = 0.0;         // Reset sweep phase
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
  _control_state.valve_signal = 0.0;
}

float ActuatorControl::updatePID(float flow_reference, float flow_measured) {
  _control_state.error = flow_reference - flow_measured;
  _control_state.integrator += _control_state.error;

  _control_state.output = _pid_config.p_gain * _control_state.error +
                          _pid_config.i_gain * _control_state.integrator;

  float valve_signal = _control_state.output + _pid_config.valve_offset;

  // Clamp to valid percentage range (0-100%)
  if (valve_signal < 0.0) valve_signal = 0.0;
  if (valve_signal > 100.0) valve_signal = 100.0;

  // Zero output if reference is zero
  if (flow_reference == 0) {
    valve_signal = 0.0;
    _control_state.integrator = 0.0;
  }

  _control_state.valve_signal = valve_signal;
  outputToValve(valve_signal);

  return valve_signal;
}

void ActuatorControl::setSignalGeneratorConfig(const SignalGeneratorConfig& config) {
  _sig_gen_config = config;
  _index_in_period = 0;  // Reset period counter on config change
  _period_start_time_us = 0;  // Reset period timer
}

SignalGeneratorConfig ActuatorControl::getSignalGeneratorConfig() const {
  return _sig_gen_config;
}

void ActuatorControl::setSweepConfig(const SweepConfig& config) {
  _sweep_config = config;
  _sweep_start_time_us = 0;  // Reset sweep timer on config change
  _sweep_phase = 0.0;        // Reset sweep phase
}

SweepConfig ActuatorControl::getSweepConfig() const {
  return _sweep_config;
}

float ActuatorControl::updateSignalGenerator() {
  // Initialize period start time if not set
  if (_period_start_time_us == 0) {
    _period_start_time_us = micros();
  }

  // Calculate elapsed time in period
  uint32_t elapsed_us = micros() - _period_start_time_us;
  uint32_t period_us = (uint32_t)(_sig_gen_config.period_seconds * 1000000.0);

  // Reset period if it has completed
  if (elapsed_us >= period_us) {
    _period_start_time_us = micros();
    elapsed_us = 0;
  }

  // Calculate normalized position in period (0.0 to 1.0)
  float phase = (float)elapsed_us / (float)period_us;

  float signal = 0.0;

  switch (_controller_mode) {
    case SINE_CONTROL:
      signal = generateSine(phase);
      break;
    case STEP_CONTROL:
      signal = generateStep(phase);
      break;
    case TRIANGLE_CONTROL:
      signal = generateTriangle(phase);
      break;
    default:
      signal = 0.0;
      break;
  }

  _valve_signal_generated = signal;
  outputToValve(signal);

  return signal;
}

float ActuatorControl::updateSweepGenerator() {
  float signal = generateSweep();
  _valve_signal_generated = signal;
  outputToValve(signal);
  return signal;
}

float ActuatorControl::generateSine(float phase) {
  // Calculate sine wave: offset ± amplitude/2
  // Formula generates values from offset-amplitude/2 to offset+amplitude/2
  // phase is 0.0 to 1.0 representing position in the period
  float signal_percent = _sig_gen_config.amplitude / 2.0 *
                         (1.0 + sin(6.28318530718 * phase));  // 2*PI*phase
  float total_percent = signal_percent + _sig_gen_config.offset;

  // Clamp to valid percentage range
  if (total_percent < 0.0) total_percent = 0.0;
  if (total_percent > 100.0) total_percent = 100.0;

  return total_percent;
}

float ActuatorControl::generateStep(float phase) {
  // Generate step wave alternating between two levels
  // phase is 0.0 to 1.0 representing position in the period
  float signal_percent;

  if (phase <= 0.5) {
    signal_percent = _sig_gen_config.offset + _sig_gen_config.amplitude;
  } else {
    signal_percent = _sig_gen_config.offset;
  }

  // Clamp to valid percentage range
  if (signal_percent < 0.0) signal_percent = 0.0;
  if (signal_percent > 100.0) signal_percent = 100.0;

  return signal_percent;
}

float ActuatorControl::generateTriangle(float phase) {
  // Generate triangle wave
  // phase is 0.0 to 1.0 representing position in the period
  float signal_percent;

  if (phase < 0.5) {
    // Rising ramp (first half of period)
    signal_percent = _sig_gen_config.offset +
                     _sig_gen_config.amplitude * (phase * 2.0);
  } else {
    // Falling ramp (second half of period)
    signal_percent = _sig_gen_config.offset +
                     _sig_gen_config.amplitude * (2.0 - phase * 2.0);
  }

  // Clamp to valid percentage range
  if (signal_percent < 0.0) signal_percent = 0.0;
  if (signal_percent > 100.0) signal_percent = 100.0;

  return signal_percent;
}

float ActuatorControl::generateSweep() {
  // Initialize sweep start time if not set
  if (_sweep_start_time_us == 0) {
    _sweep_start_time_us = micros();
    _sweep_phase = 0.0;
  }

  // Calculate elapsed time in sweep
  uint32_t current_time_us = micros();
  uint32_t elapsed_us = current_time_us - _sweep_start_time_us;
  float elapsed_seconds = (float)elapsed_us / 1000000.0;
  uint32_t sweep_duration_us = (uint32_t)(_sweep_config.sweep_time * 1000000.0);

  // Reset sweep if it has completed
  if (elapsed_us >= sweep_duration_us) {
    _sweep_start_time_us = micros();
    _sweep_phase = 0.0;
    elapsed_seconds = 0.0;
  }

  // Calculate normalized position in sweep (0.0 to 1.0)
  float sweep_position = elapsed_seconds / _sweep_config.sweep_time;

  // Calculate instantaneous frequency based on sweep type
  float current_freq;
  if (_sweep_config.logarithmic) {
    // Logarithmic sweep: frequency increases exponentially
    // f(t) = f_start * (f_stop/f_start)^(t/T)
    float ratio = _sweep_config.stop_freq / _sweep_config.start_freq;
    current_freq = _sweep_config.start_freq * pow(ratio, sweep_position);
  } else {
    // Linear sweep: frequency increases linearly
    // f(t) = f_start + (f_stop - f_start) * (t/T)
    current_freq = _sweep_config.start_freq +
                   (_sweep_config.stop_freq - _sweep_config.start_freq) * sweep_position;
  }

  // Calculate time since last call (approximate using control interval)
  // We accumulate phase to maintain continuous sine wave
  static uint32_t last_time_us = 0;
  uint32_t dt_us = (last_time_us == 0) ? 10000 : (current_time_us - last_time_us);
  last_time_us = current_time_us;
  float dt_seconds = (float)dt_us / 1000000.0;

  // Accumulate phase: delta_phase = frequency * dt
  _sweep_phase += current_freq * dt_seconds;

  // Keep phase in 0.0 to 1.0 range
  while (_sweep_phase >= 1.0) {
    _sweep_phase -= 1.0;
  }

  // Generate sine wave at current phase using existing offset/amplitude
  // Use same formula as generateSine()
  float signal_percent = _sig_gen_config.amplitude / 2.0 *
                         (1.0 + sin(6.28318530718 * _sweep_phase));  // 2*PI*phase
  float total_percent = signal_percent + _sig_gen_config.offset;

  // Clamp to valid percentage range
  if (total_percent < 0.0) total_percent = 0.0;
  if (total_percent > 100.0) total_percent = 100.0;

  return total_percent;
}

void ActuatorControl::setValveControlSignal(float percent) {
  // Clamp to valid range
  if (percent < 0.0) percent = 0.0;
  if (percent > 100.0) percent = 100.0;

  _valve_signal_externally_set = percent;

  if (_controller_mode == VALVE_SET_VALUE_CONTROL) {
    outputToValve(percent);
  }
}

float ActuatorControl::getValveControlSignal() const {
  // Return current valve signal as percentage (already in %)
  if (_controller_mode == VALVE_SET_VALUE_CONTROL) {
    return _valve_signal_externally_set;
  } else if (_controller_mode == PID_CONTROL) {
    return _control_state.valve_signal;
  } else {
    return _valve_signal_generated;
  }
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

void ActuatorControl::outputToValve(float signal_percent) {
  // Send valve command via serial to external actuator (percentage-based)
  char actuatorCMD = 'V';
  sendSerialCommand(actuatorCMD, signal_percent);
  // Serial.println("valve CMD " + String(signal_percent) + "% " + String(millis()));

  // Legacy hardware output (commented out - now using serial actuator)
  // Convert percentage to hardware units only for legacy hardware
  // uint16_t hardware_signal = percentToHardware(signal_percent);
  // if (_external_pwm) {
  //   analogOutMCP4725(hardware_signal);
  // } else {
  //   analogWrite(_valve_ctrl_pin, hardware_signal);
  // }
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
      // Output the set valve signal (ensure serial actuator receives updates)
      outputToValve(_valve_signal_externally_set);
      // Note: quiet_mode == 3 output is now handled in outputData() function
      break;

    case SINE_CONTROL:
      updateSignalGenerator();
      // Note: quiet_mode == 3 output is now handled in outputData() function
      break;

    case STEP_CONTROL:
      updateSignalGenerator();
      // Note: quiet_mode == 3 output is now handled in outputData() function
      break;

    case TRIANGLE_CONTROL:
      updateSignalGenerator();
      // Note: quiet_mode == 3 output is now handled in outputData() function
      break;

    case SWEEP_CONTROL:
      updateSweepGenerator();
      // Note: quiet_mode == 3 output is now handled in outputData() function
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

  // Convert percentage to 12-bit hardware value (0-4095) with rounding
  return (uint16_t)((percent * 4095.0 / 100.0) + 0.5);
}

float ActuatorControl::hardwareToPercent(uint16_t hardware) const {
  // Convert 12-bit hardware value (0-4095) to percentage
  return (hardware * 100.0) / 4095.0;
}

// ============================================================================
// Serial Actuator Communication - Forwarded to SerialActuatorReader
// ============================================================================
// These methods forward calls to the SerialActuatorReader instance
// Kept for backward compatibility with existing code
// ============================================================================

void ActuatorControl::setSerialActuatorReader(SerialActuatorReader* reader) {
  _serialActuatorReader = reader;
}

bool ActuatorControl::sendSerialCommand(char command, float value) {
  if (_serialActuatorReader == nullptr) {
    return false;  // No serial reader configured
  }
  return _serialActuatorReader->sendSerialCommand(command, value);
}

bool ActuatorControl::readSerialResponse(char& command, float& value, uint32_t timeout_ms) {
  if (_serialActuatorReader == nullptr) {
    return false;  // No serial reader configured
  }
  return _serialActuatorReader->readSerialResponse(command, value, timeout_ms);
}

bool ActuatorControl::readSerialMeasurement(char& command, float& value, uint32_t timeout_ms) {
  if (_serialActuatorReader == nullptr) {
    return false;  // No serial reader configured
  }
  return _serialActuatorReader->readSerialMeasurement(command, value, timeout_ms);
}

void ActuatorControl::clearSerialBuffer() {
  if (_serialActuatorReader == nullptr) {
    return;  // No serial reader configured
  }
  _serialActuatorReader->clearBuffer();
}