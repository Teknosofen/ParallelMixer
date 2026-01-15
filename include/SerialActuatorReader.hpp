#ifndef SERIAL_ACTUATOR_READER_HPP
#define SERIAL_ACTUATOR_READER_HPP

#include <Arduino.h>

// Maximum length of a serial message (including command char and value)
#define SERIAL_ACTUATOR_MAX_MSG_LEN 32

/**
 * @brief Asynchronous serial reader for actuator data
 *
 * This class provides non-blocking serial reception from an external actuator.
 * It builds messages character-by-character and parses them when complete.
 *
 * Protocol: <CHR>FLOAT\n
 * Example: "I1.25\n" - Current reading of 1.25 A
 * Example: "V50.5\n" - Valve position 50.5%
 *
 * Usage:
 *   SerialActuatorReader reader(&Serial1);
 *   reader.begin();
 *
 *   // In loop:
 *   reader.update();  // Call frequently to process incoming data
 *
 *   // Access data:
 *   float current = reader.getValveActuatorCurrent();
 *   float misc = reader.getValveActuatorMisc();
 */
class SerialActuatorReader {
public:
  /**
   * @brief Constructor
   * @param serial Pointer to HardwareSerial (e.g., &Serial1)
   */
  SerialActuatorReader(HardwareSerial* serial);

  /**
   * @brief Initialize the reader
   */
  void begin();

  /**
   * @brief Update - call this frequently in main loop
   * Processes incoming characters and builds messages
   */
  void update();

  /**
   * @brief Get valve actuator current (command 'I')
   * @return Current in Amperes, or -9.9 if no data received yet
   */
  float getValveActuatorCurrent() const;

  /**
   * @brief Get valve actuator misc data (any command except 'I')
   * @return Last received misc value, or -9.9 if no data received yet
   */
  float getValveActuatorMisc() const;

  /**
   * @brief Get the command character of the last misc data
   * @return Command character (e.g., 'V', 'P'), or '\0' if no data
   */
  char getValveActuatorMiscCommand() const;

  /**
   * @brief Get timestamp of last current update
   * @return Milliseconds since boot when last 'I' command received
   */
  uint32_t getCurrentTimestamp() const;

  /**
   * @brief Get timestamp of last misc update
   * @return Milliseconds since boot when last misc command received
   */
  uint32_t getMiscTimestamp() const;

  /**
   * @brief Check if current data is stale
   * @param max_age_ms Maximum age in milliseconds (default: 1000ms)
   * @return true if data is older than max_age_ms
   */
  bool isCurrentStale(uint32_t max_age_ms = 100) const;

  /**
   * @brief Check if misc data is stale
   * @param max_age_ms Maximum age in milliseconds (default: 1000ms)
   * @return true if data is older than max_age_ms
   */
  bool isMiscStale(uint32_t max_age_ms = 100) const;

  /**
   * @brief Clear the receive buffer
   * Useful for recovering from communication errors
   */
  void clearBuffer();

  // ============================================================================
  // Serial Transmission Methods
  // ============================================================================

  /**
   * @brief Send a command to the external actuator
   * @param command Single character command identifier
   * @param value Float value to send
   * @return true if command was sent, false on error
   *
   * Protocol: <CHR>FLOAT\n with 2 decimal places
   * Example: sendSerialCommand('V', 50.5) sends "V50.50\n"
   *
   * Note: Caller is responsible for value validation/limiting
   */
  bool sendSerialCommand(char command, float value);

  /**
   * @brief Read a response from the external actuator (blocking)
   * @param command [OUT] Receives the command character
   * @param value [OUT] Receives the parsed float value
   * @param timeout_ms Timeout in milliseconds (default: 100ms)
   * @return true if valid response received, false on timeout or error
   *
   * Note: This is a blocking call. For non-blocking operation, use update()
   * and the getter methods instead.
   */
  bool readSerialResponse(char& command, float& value, uint32_t timeout_ms = 100);

  /**
   * @brief Read a measurement from the external actuator (blocking)
   * @param command [OUT] Receives the command character
   * @param value [OUT] Receives the parsed float value
   * @param timeout_ms Timeout in milliseconds (default: 100ms)
   * @return true if valid response received, false on timeout or error
   *
   * Note: Alias for readSerialResponse(). Use for semantic clarity when
   * reading measurements vs command confirmations.
   */
  bool readSerialMeasurement(char& command, float& value, uint32_t timeout_ms = 100);

private:
  HardwareSerial* _serial;         // Pointer to serial port
  String _receiveBuffer;           // Buffer for building messages

  // Stored data
  float _valveActuatorCurrent;     // Current in Amperes (command 'I')
  bool _valveActuatorCurrentValid = false; // Flag indicating if current data is valid
  float _valveActuatorMisc;        // Misc data (any other command)
  bool _valveActuatorMiscValid    = false;   // Flag indicating if misc data is valid
  char _valveActuatorMiscCmd;      // Command char for misc data

  // Timestamps
  uint32_t _currentTimestamp;      // Time of last current update
  uint32_t _miscTimestamp;         // Time of last misc update

  /**
   * @brief Process a complete message
   * @param message Complete message without \n terminator
   */
  void processMessage(const String& message);
};

#endif // SERIAL_ACTUATOR_READER_HPP
