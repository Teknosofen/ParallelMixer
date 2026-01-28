#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>

// Forward declarations
class ActuatorControl;

// MUX channel definitions
enum MuxChannel : uint8_t {
  MUX_DIRECT    = 0,  // No MUX prefix (direct communication)
  MUX_AIR_VALVE = 1,  // Air valve
  MUX_O2_VALVE  = 2,  // O2 valve
  MUX_EXP_VALVE = 3,  // Expiratory valve
  MUX_NC        = 4,  // Not Connected / Reserved
  MUX_BLOWER    = 5   // Blower
};

// Get human-readable name for MUX channel
inline const char* getMuxChannelName(uint8_t channel) {
  switch (channel) {
    case MUX_DIRECT:    return "Direct";
    case MUX_AIR_VALVE: return "AirValve";
    case MUX_O2_VALVE:  return "O2Valve";
    case MUX_EXP_VALVE: return "ExpValve";
    case MUX_NC:        return "NC";
    case MUX_BLOWER:    return "Blower";
    default:            return "Unknown";
  }
}

struct SystemConfig {
  uint32_t delta_t;                    // GUI/Serial output interval in microseconds (T command)
  uint32_t control_interval;           // Control system execution interval in microseconds (X command)
  uint32_t PressSamplTime;             // ABP2 pressure sampling time in microseconds (default: 10000 = 100Hz)
  int16_t quiet_mode;                  // 0=verbose, 1=quiet, 3-7=special modes
  float digital_flow_reference;        // Digital flow reference in L/min
  uint8_t mux_channel;                 // Current MUX channel (0-5), see MuxChannel enum
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
                           float offset, float amplitude, float period_seconds,
                           float valve_signal, uint8_t mux_channel,
                           float sweep_start = 0, float sweep_stop = 0,
                           float sweep_time = 0, bool sweep_log = false);
  
private:
  String _command_str;
  bool _command_complete;
  void* _context;
  
  void serialEvent();
};

#endif