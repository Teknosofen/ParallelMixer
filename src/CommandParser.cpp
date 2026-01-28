#include "CommandParser.hpp"
#include "ActuatorControl.hpp"

CommandParser::CommandParser() 
  : _command_complete(false), _context(nullptr) {
}

void CommandParser::begin(unsigned long baud_rate) {
  // Serial is already initialized in setup(), just configure the parser
  _command_str.reserve(64);

  // Only print if Serial is connected (non-blocking check)
  if (Serial) {
    Serial.println("CommandParser initialized");
  }
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
  Serial.println("-------------------");
  Serial.println("T for GUI/serial output interval [us] int");
  Serial.println("X for control system execution interval [us] int");
  Serial.println("Q = 0 verbose, Q = 1 quiet");
  Serial.println("Q = 3 special (mode+actuator), Q = 4 abbreviated");
  Serial.println("Q = 5 outputs Flow, SupplyP, Air");
  Serial.println("Q = 6 outputs Bus0/Bus1 SFM3505 data");
  Serial.println("Q = 7 high-speed TSV (SupplyP, Air, Valve, Current)");
  Serial.println("Verbose data: dP, Flow, SupplyP, Fused Flow");
  Serial.println("Z zeroes controller integrator");
  Serial.println("E selects Ext (1 default) 12 bit / Int (0) 10 bit PWM");
  Serial.println("F for flow ref value [L/min] float");
  Serial.println("I for integral const [-] float");
  Serial.println("P for proportional const [-] float");
  Serial.println("D for derivative const [-] float");
  Serial.println("C for controller mode");
  Serial.println("  0 = PI controller, 1 = Valve set value");
  Serial.println("  2 = Sine, 3 = Step, 4 = Triangle, 5 = Sweep");
  Serial.println("O for Offset in [%], float 0.0-100.0");
  Serial.println("A for Amplitude in [%], float 0.0-100.0");
  Serial.println("S for period time in [seconds], float");
  Serial.println("W for sweep: W<start_freq>,<stop_freq>,<sweep_time>[,log|lin]");
  Serial.println("  Example: W0.1,10,20,log (0.1-10Hz in 20s, logarithmic)");
  Serial.println("V for manual setting of valve output [%], float 0.0-100.0");
  Serial.println("M for MUX channel selection (0-5)");
  Serial.println("  0=Direct, 1=AirValve, 2=O2Valve, 3=ExpValve, 4=NC, 5=Blower");
  Serial.println("! lists current settings");
}

void CommandParser::printSettings(const SystemConfig& config, int controller_mode,
                                  float offset, float amplitude, float period_seconds,
                                  float valve_signal, uint8_t mux_channel,
                                  float sweep_start, float sweep_stop,
                                  float sweep_time, bool sweep_log) {
  Serial.printf("T= %lu us (GUI/serial output)\n", config.delta_t);
  Serial.printf("X= %lu us (control execution)\n", config.control_interval);
  Serial.printf("F= %.2f L/min\n", config.digital_flow_reference);
  Serial.printf("Q= %d\n", config.quiet_mode);
  Serial.printf("M= %d (%s)\n", mux_channel, getMuxChannelName(mux_channel));
  Serial.printf("C= %d\n", controller_mode);
  Serial.printf("O= %.2f %%\n", offset);
  Serial.printf("A= %.2f %%\n", amplitude);
  Serial.printf("S= %.3f s\n", period_seconds);
  Serial.printf("W= %.3f -> %.3f Hz, %.3f s, %s\n", sweep_start, sweep_stop, sweep_time, sweep_log ? "log" : "lin");
  Serial.printf("V= %.2f %%\n", valve_signal);
  Serial.println();
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
      
    case 'T': case 't':  // GUI/Serial output interval
      if (params.length() > 0) {
        config.delta_t = params.toInt();
        if (config.delta_t < 0) config.delta_t = 0;
        if (config.delta_t > 1000000) config.delta_t = 1000000;  // Max 1 second
        Serial.print("T= ");
        Serial.print(config.delta_t);
        Serial.print(" us");
        sendOK();
      } else {
        Serial.print("T= ");
        Serial.print(config.delta_t);
        Serial.println(" us");
      }
      break;

    case 'X': case 'x':  // Control system execution interval
      if (params.length() > 0) {
        config.control_interval = params.toInt();
        if (config.control_interval < 500) config.control_interval = 500;  // Min 1ms
        if (config.control_interval > 1000000) config.control_interval = 1000000;  // Max 1 second
        Serial.print("X= ");
        Serial.print(config.control_interval);
        Serial.print(" us");
        sendOK();
      } else {
        Serial.print("X= ");
        Serial.print(config.control_interval);
        Serial.println(" us");
      }
      break;

    case 'Q': case 'q':  // Quiet mode
      if (params.length() > 0) {
        config.quiet_mode = params.toInt();
        if (config.quiet_mode > 7) config.quiet_mode = 1;  // Now supports 0-7
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
        if (mode >= 0 && mode <= 5) {
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
      
    case 'O': case 'o':  // Offset (as percentage 0-100%)
      {
        SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
        if (params.length() > 0) {
          float offset_percent = params.toFloat();
          // Clamp to 0-100%
          if (offset_percent < 0.0) offset_percent = 0.0;
          if (offset_percent > 100.0) offset_percent = 100.0;
          // Store as percentage - ActuatorControl handles hardware conversion
          sigConfig.offset = offset_percent;
          actuator.setSignalGeneratorConfig(sigConfig);
          Serial.print("O= ");
          Serial.print(offset_percent, 2);
          Serial.print(" %");
          sendOK();
        } else {
          Serial.print("O= ");
          Serial.print(sigConfig.offset, 2);  // Already in percentage
          Serial.println(" %");
        }
      }
      break;
      
    case 'A': case 'a':  // Amplitude (as percentage 0-100%)
      {
        SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
        if (params.length() > 0) {
          float amplitude_percent = params.toFloat();
          // Clamp to 0-100%
          if (amplitude_percent < 0.0) amplitude_percent = 0.0;
          if (amplitude_percent > 100.0) amplitude_percent = 100.0;
          // Store as percentage - ActuatorControl handles hardware conversion
          sigConfig.amplitude = amplitude_percent;
          actuator.setSignalGeneratorConfig(sigConfig);
          Serial.print("A= ");
          Serial.print(amplitude_percent, 2);
          Serial.print(" %");
          sendOK();
        } else {
          Serial.print("A= ");
          Serial.print(sigConfig.amplitude, 2);  // Already in percentage
          Serial.println(" %");
        }
      }
      break;
      
    case 'V': case 'v':  // Valve control signal (as percentage 0-100%)
      if (params.length() > 0) {
        float valve_percent = params.toFloat();
        // Clamp to 0-100%
        if (valve_percent < 0.0) valve_percent = 0.0;
        if (valve_percent > 100.0) valve_percent = 100.0;
        // Pass percentage directly - ActuatorControl handles hardware conversion
        actuator.setValveControlSignal(valve_percent);
        Serial.print("V= ");
        Serial.print(valve_percent, 2);
        Serial.print(" %");
        sendOK();
      } else {
        Serial.print("V= ");
        Serial.print(actuator.getValveControlSignal(), 2);  // Returns percentage
        Serial.println(" %");
      }
      break;
      
    case 'S': case 's':  // Period time in seconds
      {
        SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
        if (params.length() > 0) {
          float period_sec = params.toFloat();
          if (period_sec < 0.001) period_sec = 0.001;  // Minimum 1ms
          if (period_sec > 3600.0) period_sec = 3600.0;  // Maximum 1 hour
          sigConfig.period_seconds = period_sec;
          actuator.setSignalGeneratorConfig(sigConfig);
          Serial.print("S= ");
          Serial.print(sigConfig.period_seconds, 3);
          Serial.print(" s");
          sendOK();
        } else {
          Serial.print("S= ");
          Serial.print(sigConfig.period_seconds, 3);
          Serial.println(" s");
        }
      }
      break;

    case 'M': case 'm':  // MUX channel selection (0-5)
      if (params.length() > 0) {
        int channel = params.toInt();
        if (channel >= 0 && channel <= 5) {
          config.mux_channel = (uint8_t)channel;
          Serial.printf("M= %d (%s)", channel, getMuxChannelName(channel));
          sendOK();
        } else {
          Serial.println("Error: MUX channel must be 0-5");
          sendError();
        }
      } else {
        Serial.printf("M= %d (%s)\n", config.mux_channel, getMuxChannelName(config.mux_channel));
      }
      break;

    case 'W': case 'w':  // Sweep configuration: W<start_freq>,<stop_freq>,<sweep_time>[,log|lin]
      {
        SweepConfig sweepConfig = actuator.getSweepConfig();
        if (params.length() > 0) {
          // Parse comma-separated parameters
          int comma1 = params.indexOf(',');
          int comma2 = params.indexOf(',', comma1 + 1);
          int comma3 = params.indexOf(',', comma2 + 1);

          if (comma1 > 0 && comma2 > comma1) {
            // Parse start_freq, stop_freq, sweep_time
            sweepConfig.start_freq = params.substring(0, comma1).toFloat();
            sweepConfig.stop_freq = params.substring(comma1 + 1, comma2).toFloat();

            if (comma3 > comma2) {
              // Has optional log/lin parameter
              sweepConfig.sweep_time = params.substring(comma2 + 1, comma3).toFloat();
              String sweepType = params.substring(comma3 + 1);
              sweepType.trim();
              sweepType.toLowerCase();
              sweepConfig.logarithmic = (sweepType == "log");
            } else {
              // No log/lin parameter, just sweep_time
              sweepConfig.sweep_time = params.substring(comma2 + 1).toFloat();
            }

            // Validate parameters
            if (sweepConfig.start_freq < 0.001) sweepConfig.start_freq = 0.001;  // Min 1mHz
            if (sweepConfig.stop_freq < 0.001) sweepConfig.stop_freq = 0.001;
            if (sweepConfig.sweep_time < 0.1) sweepConfig.sweep_time = 0.1;      // Min 100ms
            if (sweepConfig.sweep_time > 3600.0) sweepConfig.sweep_time = 3600.0; // Max 1 hour

            actuator.setSweepConfig(sweepConfig);
            Serial.printf("W= %.3f, %.3f, %.3f, %s",
                          sweepConfig.start_freq,
                          sweepConfig.stop_freq,
                          sweepConfig.sweep_time,
                          sweepConfig.logarithmic ? "log" : "lin");
            sendOK();
          } else {
            sendError();
          }
        } else {
          Serial.printf("W= %.3f Hz -> %.3f Hz, %.3f s, %s\n",
                        sweepConfig.start_freq,
                        sweepConfig.stop_freq,
                        sweepConfig.sweep_time,
                        sweepConfig.logarithmic ? "log" : "lin");
        }
      }
      break;

    case '?':  // Help
      printHelp();
      break;
      
    case '!':  // Print settings
      {
        SignalGeneratorConfig sigConfig = actuator.getSignalGeneratorConfig();
        SweepConfig sweepConfig = actuator.getSweepConfig();
        printSettings(config, actuator.getControllerMode(),
                     sigConfig.offset, sigConfig.amplitude,
                     sigConfig.period_seconds,
                     actuator.getValveControlSignal(),
                     config.mux_channel,
                     sweepConfig.start_freq, sweepConfig.stop_freq,
                     sweepConfig.sweep_time, sweepConfig.logarithmic);
      }
      break;
      
    default:
      sendError();
      break;
  }
}