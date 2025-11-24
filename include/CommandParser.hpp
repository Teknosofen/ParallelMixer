#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>

// Forward declarations
class ActuatorControl;

struct SystemConfig {
  uint32_t delta_t;                    // Sampling time in microseconds
  int16_t quiet_mode;                  // 0=verbose, 1=quiet, 2-6=special modes
  bool flow_setting_is_analog;         // true=analog, false=digital
  float digital_flow_reference;        // Digital flow reference in L/min
};

class CommandParser {
public:
  CommandParser();
  
  void begin(unsigned long baud_rate);
  void update();
  bool isCommandReady() const;
  
  // Main command processing - pass in system objects
  void processCommands(SystemConfig& config, ActuatorControl& actuator);
  
  // Command handlers - these return true if command was handled
  typedef bool (*CommandCallback)(char cmd, const String& params, void* context);
  
  void setContext(void* context);
  
  // System configuration commands
  bool handleCommand(String& command);
  
  // Helper methods for common responses
  static void sendOK();
  static void sendOK(const String& value);
  static void sendError();
  static void printHelp();
  static void printSettings(const SystemConfig& config, int controller_mode, 
                           int offset, int amplitude, int samples_per_period, 
                           uint16_t valve_signal);
  
private:
  String _command_str;
  bool _command_complete;
  void* _context;
  
  void serialEvent();
};

#endif