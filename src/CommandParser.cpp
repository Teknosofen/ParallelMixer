#include "CommandParser.hpp"
#include "ActuatorControl.hpp"
#include "VentilatorController.hpp"

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
  Serial.println("Q = 0 verbose, Q = 1 quiet, Q = 2 debug");
  Serial.println("Q = 3 special (mode+actuator), Q = 4 abbreviated");
  Serial.println("Q = 5 Flow/SupplyP/Air");
  Serial.println("Q = 6 dual bus + all (time,P,T,Bus0,Bus1,V,I,O2)");
  Serial.println("Q = 7 high-speed TSV (time,P,T,Air,Valve,I,O2)");
  Serial.println("Q = 8 FDO2 O2 data, Q = 9 FDO2 extended/raw");
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
        if (config.quiet_mode > 8) config.quiet_mode = 1;  // Supports 0-8
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

// ============================================================================
// Ventilator Command Processing (two-character commands)
// ============================================================================
bool CommandParser::processVentilatorCommands(VentilatorController& ventilator) {
  if (!_command_complete) {
    return false;
  }

  String cmdStr = _command_str;
  cmdStr.trim();

  if (cmdStr.length() < 2) {
    return false;  // Not a two-character command
  }

  // Extract two-character command and value
  String cmd = cmdStr.substring(0, 2);
  cmd.toUpperCase();
  String params = cmdStr.substring(2);
  params.trim();

  VentilatorConfig cfg = ventilator.getConfig();
  bool handled = true;
  bool hasValue = (params.length() > 0);

  // Control commands
  if (cmd == "VO") {
    if (hasValue) {
      if (params.toInt() != 0) {
        ventilator.start();
        Serial.println("Ventilator ON");
      } else {
        ventilator.stop();
        Serial.println("Ventilator OFF");
      }
    } else {
      Serial.printf("VO=%d\n", ventilator.isRunning() ? 1 : 0);
    }
  }
  else if (cmd == "VS") {
    // Status query
    VentilatorStatus st = ventilator.getStatus();
    Serial.printf("Vent: %s  State: %s\n",
                  ventilator.isRunning() ? "ON" : "OFF",
                  ventilator.getStateString());
    Serial.printf("RR=%.1f VT=%.0f IE=%.2f FI=%.0f%%\n",
                  cfg.respRate, cfg.tidalVolume_mL, cfg.ieRatio, cfg.targetFiO2 * 100);
    Serial.printf("PI=%.1f PE=%.1f MF=%.1f\n",
                  cfg.maxPressure_mbar, cfg.peep_mbar, cfg.maxInspFlow_slm);
    if (ventilator.isRunning()) {
      Serial.printf("#=%lu PkP=%.1f Vt=%.0fmL\n",
                    st.breathCount, st.peakPressure_mbar, st.measuredVt_mL);
    }
  }
  // Timing commands
  else if (cmd == "RR") {
    if (hasValue) {
      ventilator.setRespRate(params.toFloat());
      sendOK();
    } else {
      Serial.printf("RR=%.1f\n", cfg.respRate);
    }
  }
  else if (cmd == "IE") {
    if (hasValue) {
      ventilator.setIERatio(params.toFloat());
      sendOK();
    } else {
      Serial.printf("IE=%.2f\n", cfg.ieRatio);
    }
  }
  else if (cmd == "IP") {
    if (hasValue) {
      ventilator.setInspPauseFraction(params.toFloat());
      sendOK();
    } else {
      Serial.printf("IP=%.2f\n", cfg.inspPauseFraction);
    }
  }
  else if (cmd == "I1") {
    if (hasValue) {
      ventilator.setInsp1Fraction(params.toFloat());
      sendOK();
    } else {
      Serial.printf("I1=%.2f\n", cfg.insp1Fraction);
    }
  }
  else if (cmd == "I2") {
    if (hasValue) {
      ventilator.setInsp2Fraction(params.toFloat());
      sendOK();
    } else {
      Serial.printf("I2=%.2f\n", cfg.insp2Fraction);
    }
  }
  else if (cmd == "EN") {
    if (hasValue) {
      ventilator.setExpNonTrigFraction(params.toFloat());
      sendOK();
    } else {
      Serial.printf("EN=%.2f\n", cfg.expNonTrigFraction);
    }
  }
  else if (cmd == "ES") {
    if (hasValue) {
      ventilator.setExpSyncFraction(params.toFloat());
      sendOK();
    } else {
      Serial.printf("ES=%.2f\n", cfg.expSyncFraction);
    }
  }
  // Volume/Flow commands
  else if (cmd == "VT") {
    if (hasValue) {
      ventilator.setTidalVolume(params.toFloat());
      sendOK();
    } else {
      Serial.printf("VT=%.0f\n", cfg.tidalVolume_mL);
    }
  }
  else if (cmd == "MF") {
    if (hasValue) {
      ventilator.setMaxFlow(params.toFloat());
      sendOK();
    } else {
      Serial.printf("MF=%.1f\n", cfg.maxInspFlow_slm);
    }
  }
  else if (cmd == "TF") {
    if (hasValue) {
      ventilator.setTotalFlow(params.toFloat());
      sendOK();
    } else {
      Serial.printf("TF=%.1f\n", cfg.totalFlow_slm);
    }
  }
  else if (cmd == "VC") {
    if (hasValue) {
      ventilator.setUseVolumeControl(params.toInt() != 0);
      sendOK();
    } else {
      Serial.printf("VC=%d\n", cfg.useVolumeControl ? 1 : 0);
    }
  }
  // Pressure commands
  else if (cmd == "PI") {
    if (hasValue) {
      ventilator.setMaxPressure(params.toFloat());
      sendOK();
    } else {
      Serial.printf("PI=%.1f\n", cfg.maxPressure_mbar);
    }
  }
  else if (cmd == "PE") {
    if (hasValue) {
      ventilator.setPEEP(params.toFloat());
      sendOK();
    } else {
      Serial.printf("PE=%.1f\n", cfg.peep_mbar);
    }
  }
  else if (cmd == "PR") {
    if (hasValue) {
      ventilator.setPressureRampTime(params.toFloat());
      sendOK();
    } else {
      Serial.printf("PR=%.0f\n", cfg.pressureRampTime_ms);
    }
  }
  // Gas command
  else if (cmd == "FI") {
    if (hasValue) {
      // Input is percentage, convert to fraction
      ventilator.setFiO2(params.toFloat() / 100.0f);
      sendOK();
    } else {
      Serial.printf("FI=%.0f%%\n", cfg.targetFiO2 * 100);
    }
  }
  // Trigger commands
  else if (cmd == "TE") {
    if (hasValue) {
      ventilator.setTriggerEnabled(params.toInt() != 0);
      sendOK();
    } else {
      Serial.printf("TE=%d\n", cfg.triggerEnabled ? 1 : 0);
    }
  }
  else if (cmd == "BF") {
    if (hasValue) {
      ventilator.setBiasFlow(params.toFloat());
      sendOK();
    } else {
      Serial.printf("BF=%.1f\n", cfg.biasFlow_slm);
    }
  }
  else if (cmd == "FT") {
    if (hasValue) {
      ventilator.setFlowTrigger(params.toFloat());
      sendOK();
    } else {
      Serial.printf("FT=%.1f\n", cfg.flowTrigger_slm);
    }
  }
  else if (cmd == "PT") {
    if (hasValue) {
      ventilator.setPressureTrigger(params.toFloat());
      sendOK();
    } else {
      Serial.printf("PT=%.1f\n", cfg.pressureTrigger_mbar);
    }
  }
  // Alarm commands
  else if (cmd == "AH") {
    if (hasValue) {
      cfg.highPressureAlarm_mbar = params.toFloat();
      ventilator.setConfig(cfg);
      sendOK();
    } else {
      Serial.printf("AH=%.1f\n", cfg.highPressureAlarm_mbar);
    }
  }
  else if (cmd == "AL") {
    if (hasValue) {
      cfg.lowPressureAlarm_mbar = params.toFloat();
      ventilator.setConfig(cfg);
      sendOK();
    } else {
      Serial.printf("AL=%.1f\n", cfg.lowPressureAlarm_mbar);
    }
  }
  else if (cmd == "AA") {
    if (hasValue) {
      cfg.apneaTime_s = params.toFloat();
      ventilator.setConfig(cfg);
      sendOK();
    } else {
      Serial.printf("AA=%.1f\n", cfg.apneaTime_s);
    }
  }
  else {
    handled = false;  // Not a ventilator command
  }

  if (handled) {
    // Clear command buffer since we handled it
    _command_str = "";
    _command_complete = false;
  }

  return handled;
}