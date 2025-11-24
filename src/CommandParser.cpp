#include "CommandParser.hpp"
#include "ActuatorControl.hpp"

CommandParser::CommandParser() 
  : _command_complete(false), _context(nullptr) {
}

void CommandParser::begin(unsigned long baud_rate) {
  Serial.begin(baud_rate);
  _command_str.reserve(64);
}

void CommandParser::update() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if ((inChar == '\n') || (inChar == '\r')) {
      if (_command_str.length() > 0) {
        _command_complete = true;
      }
    } else {
      _command_str += inChar;
    }
  }
}

bool CommandParser::isCommandReady() const {
  return _command_complete;
}

void CommandParser::setContext(void* context) {
  _context = context;
}

bool CommandParser::handleCommand(String& command) {
  if (!_command_complete) {
    return false;
  }
  
  command = _command_str;
  _command_str = "";
  _command_complete = false;
  
  return true;
}

void CommandParser::sendOK() {
  Serial.println(" OK");
}

void CommandParser::sendOK(const String& value) {
  Serial.print(value);
  Serial.println(" OK");
}

void CommandParser::sendError() {
  Serial.println("Err");
}

void CommandParser::printHelp() {
  Serial.println("Available commands:");
  Serial.println("--2020-06-25-------");
  Serial.println("N = 0 for aNalog flow reference");
  Serial.println("T for sampl Time [us] int");
  Serial.println("Q = 1 for quiet, Q = 0 verbose");
  Serial.println("Q = 3 for special, Q = 4 for abbreviated");
  Serial.println("Q = 5 outputs A1 analogue in");
  Serial.println("Q = 6 outputs Flow, SupplyP, FusedFlow2");
  Serial.println("Verbose data: dP, Flow, SupplyP, Fused Flow");
  Serial.println("Z zeroes controller integrator");
  Serial.println("E selects Ext (1 default) 12 bitar/ Int (0) 10 bitar PWM");
  Serial.println("F for flow ref value [L/min] float");
  Serial.println("I for int const [-] float");
  Serial.println("P for propo const [-] float");
  Serial.println("D for integrat const [-] float");
  Serial.println("C for controller mode");
  Serial.println("  0 = PI controller, 1 = Valve set value");
  Serial.println("  2 = Sine, 3 = step, 4 = Triangle");
  Serial.println("O for Offset in [DAC steps], Int");
  Serial.println("A for Amplitude in [DAC steps], int");
  Serial.println("S for Samples per period of pulses or sine, int");
  Serial.println("V for manual setting of valve output [12 bit DAC counts 0..4095]");
  Serial.println("R for rise time could be added another day");
  Serial.println("! lists current settings");
}

void CommandParser::printSettings(const SystemConfig& config, int controller_mode,
                                  int offset, int amplitude, int samples_per_period,
                                  uint16_t valve_signal) {
  Serial.print("T= ");
  Serial.println(config.delta_t);
  Serial.print("Q= ");
  Serial.println(config.quiet_mode);
  Serial.print("C= ");
  Serial.println(controller_mode);
  Serial.print("O= ");
  Serial.println(offset);
  Serial.print("A= ");
  Serial.println(amplitude);
  Serial.print("S= ");
  Serial.println(samples_per_period);
  Serial.print("V= ");
  Serial.println(valve_signal);
  Serial.println("");
}

// Global serialEvent function required by Arduino framework
void serialEvent() {
  // This will be called automatically by Arduino
  // The actual implementation is in CommandParser::update()
}

void CommandParser::processCommands(SystemConfig& config, ActuatorControl& actuator) {
  if (!_command_complete) {
    return;
  }
  
  String cmdStr = _command_str;
  _command_str = "";
  _command_complete = false;
  
  cmdStr.trim();
  int16_t cmdLength = cmdStr.length();
  
  if (cmdLength == 0) {
    return;
  }
  
  char cmd = cmdStr[0];
  String params = "";
  
  if (cmdLength > 1) {
    params = cmdStr.substring(1);
  }
  
  switch (cmd) {
    case 'N': case 'n':  // Analog/digital flow reference selection
      if (params.length() > 0) {
        config.flow_setting_is_analog = (params.toInt() != 0);
        Serial.print(config.flow_setting_is_analog);
        sendOK();
      } else {
        Serial.print("N= ");
        Serial.println(config.flow_setting_is_analog);
      }
      break;
      
    case 'F': case 'f':  // Flow reference value
      if (params.length() > 0) {
        config.digital_flow_reference = params.toFloat();
        Serial.print(config.digital_flow_reference, 4);
        sendOK();
      } else {
        Serial.print("F-ref= ");
        Serial.println(config.digital_flow_reference, 4);
      }
      break;
      
    case 'T': case 't':  // Sampling time
      if (params.length() > 0) {
        config.delta_t = params.toInt();
        if (config.delta_t < 0) config.delta_t = 0;
        if (config.delta_t > 32767) config.delta_t = 32767;
        Serial.print("T= ");
        Serial.print(config.delta_t);
        sendOK();
      } else {
        Serial.print("Dt= ");
        Serial.println(config.delta_t);
      }
      break;
      
    case 'Q': case 'q':  // Quiet mode
      if (params.length() > 0) {
        config.quiet_mode = params.toInt();
        if (config.quiet_mode > 6) config.quiet_mode = 1;
        Serial.print("Q= ");
        Serial.print(config.quiet_mode);
        sendOK();
      } else {
        Serial.print("Q= ");
        Serial.println(config.quiet_mode);
      }
      break;
      
    case 'I': case 'i':  // Integral gain
      {
        PIDConfig pidConfig = actuator.getPIDConfig();
        if (params.length() > 0) {
          pidConfig.i_gain = params.toFloat();
          actuator.setPIDConfig(pidConfig);
          actuator.resetPIDState();
          Serial.print(pidConfig.i_gain, 4);
          sendOK();
        } else {
          Serial.print("I= ");
          Serial.println(pidConfig.i_gain, 4);
        }
      }
      break;
      
    case 'P': case 'p':  // Proportional gain
      {
        PIDConfig pidConfig = actuator.getPIDConfig();
        if (params.length() > 0) {
          pidConfig.p_gain = params.toFloat();
          actuator.setPIDConfig(pidConfig);
          Serial.print(pidConfig.p_gain, 4);
          sendOK();
        } else {
          Serial.print("P= ");
          Serial.println(pidConfig.p_gain, 4);
        }
      }
      break;
      
    case 'D': case 'd':  // Derivative gain
      {
        PIDConfig pidConfig = actuator.getPIDConfig();
        if (params.length() > 0) {
          pidConfig.d_gain = params.toFloat();
          actuator.setPIDConfig(pidConfig);
          Serial.print(pidConfig.d_gain, 4);
          sendOK();
        } else {
          Serial.print("D= ");
          Serial.println(pidConfig.d_gain, 4);
        }
      }
      break;
      
    case 'E': case 'e':  // External/internal PWM
      if (params.length() > 0) {
        actuator.setExternalPWM(params.toInt() != 0);
        sendOK();
      } else {
        Serial.print("E= ");
        Serial.println(actuator.isExternalPWM());
      }
      break;
      
    case 'Z': case 'z':  // Zero controller
      actuator.resetPIDState();
      Serial.println("Z OK");
      break;
      
    case 'C': case 'c':  // Controller mode
      if (params.length() > 0) {
        int mode = params.toInt();
        if (mode >= 0 && mode <= 4) {
          actuator.setControllerMode((ControllerMode)mode);
          Serial.print(mode);
          sendOK();
        } else {
          sendError();
        }
      } else {
        Serial.print("C= ");
        Serial.println(actuator.getControllerMode());
      }
      break;
      
    case 'O': case 'o':  // Offset
      {
        SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
        if (params.length() > 0) {
          int offset = params.toInt();
          if (offset < 0) offset = 0;
          if (offset > 4095) offset = 4095;
          sigConfig.offset = offset;
          actuator.setSignalGeneratorConfig(sigConfig);
          Serial.print("O= ");
          Serial.print(sigConfig.offset);
          sendOK();
        } else {
          Serial.print("O= ");
          Serial.println(sigConfig.offset);
        }
      }
      break;
      
    case 'A': case 'a':  // Amplitude
      {
        SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
        if (params.length() > 0) {
          int amplitude = params.toInt();
          if (amplitude < 0) amplitude = 0;
          if (amplitude > 4095) amplitude = 4095;
          sigConfig.amplitude = amplitude;
          actuator.setSignalGeneratorConfig(sigConfig);
          Serial.print("A= ");
          Serial.print(sigConfig.amplitude);
          sendOK();
        } else {
          Serial.print("A= ");
          Serial.println(sigConfig.amplitude);
        }
      }
      break;
      
    case 'V': case 'v':  // Valve control signal
      if (params.length() > 0) {
        int signal = params.toInt();
        if (signal < 0) signal = 0;
        if (signal > 4095) signal = 4095;
        actuator.setValveControlSignal(signal);
        Serial.print("V= ");
        Serial.print(signal);
        sendOK();
      } else {
        Serial.print("V= ");
        Serial.println(actuator.getValveControlSignal());
      }
      break;
      
    case 'S': case 's':  // Samples per period
      {
        SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
        if (params.length() > 0) {
          int samples = params.toInt();
          if (samples < 0) samples = 0;
          if (samples > 65535) samples = 65535;
          sigConfig.samples_per_period = samples;
          actuator.setSignalGeneratorConfig(sigConfig);
          Serial.print("S= ");
          Serial.print(sigConfig.samples_per_period);
          sendOK();
        } else {
          Serial.print("S= ");
          Serial.println(sigConfig.samples_per_period);
        }
      }
      break;
      
    case '?':  // Help
      printHelp();
      break;
      
    case '!':  // Print settings
      {
        SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
        printSettings(config, actuator.getControllerMode(),
                     sigConfig.offset, sigConfig.amplitude,
                     sigConfig.samples_per_period,
                     actuator.getValveControlSignal());
      }
      break;
      
    default:
      sendError();
      break;
  }
}