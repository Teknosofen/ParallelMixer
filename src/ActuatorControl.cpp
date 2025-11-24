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
  
  // Default signal generator configuration
  _sig_gen_config.offset = 1024;
  _sig_gen_config.amplitude = 256;
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
  float signal = (float)_sig_gen_config.amplitude / 2.0 * 
                 (1.0 + sin(6.28 * (float)_index_in_period / (float)_sig_gen_config.samples_per_period));
  return uint16_t(signal + _sig_gen_config.offset);
}

uint16_t ActuatorControl::generateStep() {
  if (_index_in_period <= _sig_gen_config.samples_per_period / 2) {
    return _sig_gen_config.offset + _sig_gen_config.amplitude;
  } else {
    return _sig_gen_config.offset;
  }
}

uint16_t ActuatorControl::generateTriangle() {
  uint16_t signal;
  
  if (_index_in_period < (float)_sig_gen_config.samples_per_period / 2.0) {
    // Rising ramp
    signal = _sig_gen_config.offset + 
             (float)_sig_gen_config.amplitude * (_index_in_period * 2.0 / (float)_sig_gen_config.samples_per_period);
  } else {
    // Falling ramp
    signal = _sig_gen_config.offset + 
             (float)_sig_gen_config.amplitude * 
             (1 - (_index_in_period * 2.0 - (float)_sig_gen_config.samples_per_period) / (float)_sig_gen_config.samples_per_period);
  }
  
  return signal;
}

void ActuatorControl::setValveControlSignal(uint16_t signal) {
  if (signal > 4095) signal = 4095;
  _valve_signal_externally_set = signal;
  
  if (_controller_mode == VALVE_SET_VALUE_CONTROL) {
    outputToValve(signal);
  }
}

uint16_t ActuatorControl::getValveControlSignal() const {
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