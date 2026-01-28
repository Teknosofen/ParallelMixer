#ifndef SERIAL_MUX_ROUTER_HPP
#define SERIAL_MUX_ROUTER_HPP

#include <Arduino.h>

// Maximum length of a serial message (including address, command char and value)
#define MUX_SERIAL_MAX_MSG_LEN 32

/**
 * @brief Serial multiplexer router for multi-device communication
 *
 * This class handles serial communication with multiple downstream devices
 * through a hardware multiplexer. Each device is addressed by a single digit ('0'-'9').
 *
 * Protocol:
 *   - Address '0': No prefix (direct communication, backward compatible)
 *   - Address '1'-'9': Message prefixed with address digit
 *
 * Message format:
 *   - Without MUX: <CMD>VALUE\n        (e.g., "I1.25\n")
 *   - With MUX:    <ADDR><CMD>VALUE\n  (e.g., "3I1.25\n" for address 3)
 *
 * Receive commands (case-insensitive):
 *   'I' - Current (Amperes)
 *   'R' - Blower RPM
 *   'F' - Actual Flow
 *   'P' - Actual Pressure
 *
 * Send commands (uppercase for MUX compatibility):
 *   'I' - Set Current
 *   'U' - Set Voltage
 *   'F' - Set Flow
 *   'P' - Set Pressure
 *   'R' - Set Blower RPM
 *
 * Usage:
 *   SerialMuxRouter router(&Serial1);
 *   router.begin();
 *
 *   // In loop:
 *   router.update();  // Process incoming data
 *
 *   // Send command to device at address 3:
 *   router.sendSetFlow(3, 50.0);
 *
 *   // Read measurement from device at address 3:
 *   float current = router.getCurrent(3);
 */
class SerialMuxRouter {
public:
  static const uint8_t MAX_ADDRESSES = 6;   // Addresses 0-5
  static const float INVALID_VALUE;         // -9.9 indicates no data received

  // Receive command characters (measurements)
  static const char CMD_RX_CURRENT = 'I';
  static const char CMD_RX_BLOWER_RPM = 'R';
  static const char CMD_RX_FLOW = 'F';
  static const char CMD_RX_PRESSURE = 'P';

  // Send command characters (setpoints) - uppercase for MUX compatibility
  static const char CMD_TX_SET_CURRENT = 'I';
  static const char CMD_TX_SET_VOLTAGE = 'U';
  static const char CMD_TX_SET_FLOW = 'F';
  static const char CMD_TX_SET_PRESSURE = 'P';
  static const char CMD_TX_SET_BLOWER_RPM = 'R';

  /**
   * @brief Constructor
   * @param serial Pointer to HardwareSerial (e.g., &Serial1)
   */
  SerialMuxRouter(HardwareSerial* serial);

  /**
   * @brief Initialize the router
   */
  void begin();

  /**
   * @brief Update - call this frequently in main loop
   * Processes incoming characters and routes to correct address slot
   */
  void update();

  /**
   * @brief Clear the receive buffer and pending serial data
   */
  void clearBuffer();

  // ==========================================================================
  // Received Data Getters (per address)
  // ==========================================================================

  /**
   * @brief Get current measurement for address
   * @param address Device address (0-9)
   * @return Current in Amperes, or INVALID_VALUE if no data
   */
  float getCurrent(uint8_t address) const;

  /**
   * @brief Get blower RPM measurement for address
   * @param address Device address (0-9)
   * @return RPM value, or INVALID_VALUE if no data
   */
  float getBlowerRPM(uint8_t address) const;

  /**
   * @brief Get actual flow measurement for address
   * @param address Device address (0-9)
   * @return Flow value, or INVALID_VALUE if no data
   */
  float getActualFlow(uint8_t address) const;

  /**
   * @brief Get actual pressure measurement for address
   * @param address Device address (0-9)
   * @return Pressure value, or INVALID_VALUE if no data
   */
  float getActualPressure(uint8_t address) const;

  // ==========================================================================
  // Timestamps (per address)
  // ==========================================================================

  uint32_t getCurrentTimestamp(uint8_t address) const;
  uint32_t getBlowerRPMTimestamp(uint8_t address) const;
  uint32_t getActualFlowTimestamp(uint8_t address) const;
  uint32_t getActualPressureTimestamp(uint8_t address) const;

  // ==========================================================================
  // Stale Data Detection (per address)
  // ==========================================================================

  bool isCurrentStale(uint8_t address, uint32_t max_age_ms = 100) const;
  bool isBlowerRPMStale(uint8_t address, uint32_t max_age_ms = 100) const;
  bool isActualFlowStale(uint8_t address, uint32_t max_age_ms = 100) const;
  bool isActualPressureStale(uint8_t address, uint32_t max_age_ms = 100) const;

  // ==========================================================================
  // Send Commands (with address prefix)
  // ==========================================================================

  /**
   * @brief Send set current command
   * @param address Device address (0-9), 0 = no prefix
   * @param value Current setpoint
   * @return true if sent successfully
   */
  bool sendSetCurrent(uint8_t address, float value);

  /**
   * @brief Send set voltage command
   * @param address Device address (0-9), 0 = no prefix
   * @param value Voltage setpoint
   * @return true if sent successfully
   */
  bool sendSetVoltage(uint8_t address, float value);

  /**
   * @brief Send set flow command
   * @param address Device address (0-9), 0 = no prefix
   * @param value Flow setpoint
   * @return true if sent successfully
   */
  bool sendSetFlow(uint8_t address, float value);

  /**
   * @brief Send set pressure command
   * @param address Device address (0-9), 0 = no prefix
   * @param value Pressure setpoint
   * @return true if sent successfully
   */
  bool sendSetPressure(uint8_t address, float value);

  /**
   * @brief Send set blower RPM command
   * @param address Device address (0-9), 0 = no prefix
   * @param value RPM setpoint
   * @return true if sent successfully
   */
  bool sendSetBlowerRPM(uint8_t address, float value);

  /**
   * @brief Send generic command (for extensibility)
   * @param address Device address (0-9), 0 = no prefix
   * @param command Command character
   * @param value Float value to send
   * @return true if sent successfully
   */
  bool sendCommand(uint8_t address, char command, float value);

private:
  HardwareSerial* _serial;
  String _receiveBuffer;

  // Per-address measurement storage
  float _current[MAX_ADDRESSES];
  float _blowerRPM[MAX_ADDRESSES];
  float _actualFlow[MAX_ADDRESSES];
  float _actualPressure[MAX_ADDRESSES];

  // Per-address timestamps (millis)
  uint32_t _currentTimestamp[MAX_ADDRESSES];
  uint32_t _blowerRPMTimestamp[MAX_ADDRESSES];
  uint32_t _actualFlowTimestamp[MAX_ADDRESSES];
  uint32_t _actualPressureTimestamp[MAX_ADDRESSES];

  /**
   * @brief Process a complete message
   * @param message Complete message without \n terminator
   *
   * Parses address prefix (if present) and routes to correct storage slot
   */
  void processMessage(const String& message);

  /**
   * @brief Check if address is valid
   * @param address Address to check
   * @return true if address is 0-9
   */
  bool isValidAddress(uint8_t address) const;

  /**
   * @brief Check if timestamp is stale
   * @param timestamp Timestamp to check
   * @param max_age_ms Maximum age in milliseconds
   * @return true if stale or never received (timestamp == 0)
   */
  bool isStale(uint32_t timestamp, uint32_t max_age_ms) const;
};

#endif // SERIAL_MUX_ROUTER_HPP
