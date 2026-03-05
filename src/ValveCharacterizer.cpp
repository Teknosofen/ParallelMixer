#include "ValveCharacterizer.hpp"
#include "SerialMuxRouter.hpp"
#include "SensorReader.hpp"

// ============================================================================
// Constructor
// ============================================================================
ValveCharacterizer::ValveCharacterizer()
    : _muxRouter(nullptr),
      _state(CHAR_IDLE),
      _currentVoltage(0),
      _currentStep(0),
      _totalSteps(0),
      _stateEntryTime_ms(0),
      _sumFlow(0),
      _sumPressure(0),
      _sumPaw(0),
      _sampleCount(0),
      _pointCount(0) {
    memset(&_config, 0, sizeof(_config));
    memset(_points, 0, sizeof(_points));
}

void ValveCharacterizer::begin(SerialMuxRouter* muxRouter) {
    _muxRouter = muxRouter;
}

// ============================================================================
// Start / Abort
// ============================================================================
bool ValveCharacterizer::start(const CharacterizationConfig& config) {
    if (_state != CHAR_IDLE) {
        Serial.println("[CHAR] Already running — send CX to abort first");
        return false;
    }
    if (config.muxChannel != 1 && config.muxChannel != 2) {
        Serial.printf("[CHAR] Invalid channel %d — only 1 (air) and 2 (O2) supported\n",
                      config.muxChannel);
        return false;
    }
    if (!_muxRouter) {
        Serial.println("[CHAR] No MUX router configured");
        return false;
    }

    _config = config;

    // Sensible defaults if zero
    if (_config.maxVoltage <= 0.0f) _config.maxVoltage = 12.0f;
    if (_config.stepVoltage <= 0.0f) _config.stepVoltage = 0.1f;
    if (_config.settleTime_ms == 0) _config.settleTime_ms = 200;
    if (_config.samplesPerStep == 0) _config.samplesPerStep = 10;

    _totalSteps = (uint16_t)(_config.maxVoltage / _config.stepVoltage) + 1;
    _currentStep = 0;
    _currentVoltage = 0.0f;
    _pointCount = 0;

    printHeader();

    // Set initial voltage and start settling
    setVoltage(0.0f);
    _state = CHAR_RAMP_SETTLE;
    _stateEntryTime_ms = millis();

    return true;
}

void ValveCharacterizer::abort() {
    if (_state == CHAR_IDLE) return;

    setVoltage(0.0f);
    _state = CHAR_IDLE;
    Serial.println("[CHAR] Aborted — voltage set to 0");
}

// ============================================================================
// Main Update — call every loop iteration
// ============================================================================
bool ValveCharacterizer::update(const SensorData& bus0, const SensorData& bus1,
                                 float pawPressure_mbar) {
    if (_state == CHAR_IDLE) return false;

    uint32_t now = millis();

    switch (_state) {
        case CHAR_RAMP_SETTLE:
            // Wait for flow to stabilize
            if ((now - _stateEntryTime_ms) >= _config.settleTime_ms) {
                // Start sampling
                _sumFlow = 0;
                _sumPressure = 0;
                _sumPaw = 0;
                _sampleCount = 0;
                _state = CHAR_RAMP_SAMPLE;
            }
            break;

        case CHAR_RAMP_SAMPLE:
            // Accumulate sensor readings
            _sumFlow += getFlow(bus0, bus1);
            _sumPressure += getSupplyPressure(bus0, bus1);
            _sumPaw += pawPressure_mbar;
            _sampleCount++;

            if (_sampleCount >= _config.samplesPerStep) {
                // Compute averages
                float avgFlow = _sumFlow / _sampleCount;
                float avgPressure = _sumPressure / _sampleCount;
                float avgPaw = _sumPaw / _sampleCount;

                // Compute Cv = Q / sqrt(deltaP_kPa)
                // avgPressure is kPa (ABP2), avgPaw is mbar (ELVH) — convert to kPa
                float deltaP = avgPressure - avgPaw / 10.0f;
                float cv = 0.0f;
                if (deltaP > 0.1f && avgFlow > 0.001f) {
                    cv = avgFlow / sqrtf(deltaP);
                }

                // Store point
                CharacterizationPoint pt;
                pt.voltage_V = _currentVoltage;
                pt.flow_slm = avgFlow;
                pt.supplyPressure_kPa = avgPressure;
                pt.pawPressure_mbar = avgPaw;
                pt.cv = cv;

                if (_pointCount < MAX_POINTS) {
                    _points[_pointCount] = pt;
                    _pointCount++;
                }

                // Print data row
                printDataRow(pt);

                // Next step
                _currentStep++;
                _currentVoltage = _currentStep * _config.stepVoltage;

                if (_currentVoltage > _config.maxVoltage + 0.001f) {
                    // Done
                    setVoltage(0.0f);
                    _state = CHAR_DONE;
                    Serial.println("# --- Sweep complete ---");
                    printCvTable();
                    _state = CHAR_IDLE;
                } else {
                    // Move to next step
                    setVoltage(_currentVoltage);
                    _state = CHAR_RAMP_SETTLE;
                    _stateEntryTime_ms = millis();
                }
            }
            break;

        case CHAR_DONE:
            _state = CHAR_IDLE;
            break;

        case CHAR_IDLE:
        default:
            break;
    }

    return (_state != CHAR_IDLE);
}

// ============================================================================
// Sensor Helpers
// ============================================================================
float ValveCharacterizer::getFlow(const SensorData& bus0, const SensorData& bus1) const {
    if (_config.muxChannel == 1) return bus0.sfm3505_air_flow;
    if (_config.muxChannel == 2) return bus1.sfm3505_air_flow;
    return 0.0f;
}

float ValveCharacterizer::getSupplyPressure(const SensorData& bus0,
                                             const SensorData& bus1) const {
    if (_config.muxChannel == 1) return bus0.supply_pressure;
    if (_config.muxChannel == 2) return bus1.supply_pressure;
    return 0.0f;
}

void ValveCharacterizer::setVoltage(float voltage) {
    if (_muxRouter) {
        // Send sweep value directly as V percentage (0 to maxVoltage%)
        float percent = voltage;
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        _muxRouter->sendCommand(_config.muxChannel, 'V', percent);
    }
}

// ============================================================================
// Output Formatting
// ============================================================================
void ValveCharacterizer::printHeader() {
    const char* valveName = (_config.muxChannel == 1) ? "Air" : "O2";
    Serial.println();
    Serial.println("# ============================================================");
    Serial.printf( "# VALVE CHARACTERIZATION — %s valve (MUX %d)\n",
                   valveName, _config.muxChannel);
    Serial.printf( "# maxV=%.2f  stepV=%.3f  settle=%lums  samples=%d\n",
                   _config.maxVoltage, _config.stepVoltage,
                   _config.settleTime_ms, _config.samplesPerStep);
    Serial.println("# ============================================================");
    Serial.println("# V(%),\tFlow(slm),\tPsupply(kPa),\tPaw(mbar)");
}

void ValveCharacterizer::printDataRow(const CharacterizationPoint& pt) {
    Serial.printf("%.3f,\t%.3f,\t\t%.1f,\t\t%.2f\n",
                  pt.voltage_V, pt.flow_slm, pt.supplyPressure_kPa,
                  pt.pawPressure_mbar);
}

void ValveCharacterizer::printCvTable() {
    if (_pointCount == 0) return;

    const char* valveName = (_config.muxChannel == 1) ? "Air" : "O2";
    const char* varPrefix = (_config.muxChannel == 1) ? "air" : "o2";

    // Compute average supply pressure for this sweep
    float avgPsupply = 0;
    for (uint16_t i = 0; i < _pointCount; i++) {
        avgPsupply += _points[i].supplyPressure_kPa;
    }
    avgPsupply /= _pointCount;

    Serial.println();
    Serial.println("// ============================================================");
    Serial.printf( "// %s valve flow table from characterization\n", valveName);
    Serial.printf( "// Set Psupply ≈ %.0f kPa (%.2f bar) — actual varies with flow\n",
                   _points[0].supplyPressure_kPa, _points[0].supplyPressure_kPa / 100.0f);
    Serial.printf( "// Avg Psupply ≈ %.0f kPa (%.2f bar)\n",
                   avgPsupply, avgPsupply / 100.0f);
    Serial.printf( "// MUX channel %d, %d raw points\n", _config.muxChannel, _pointCount);
    Serial.println("// x = V%%, y = Flow (slm)");
    Serial.println("// NOTE: Reduce to ≤16 points before loading into PressureBandedTable");
    Serial.println("// ============================================================");
    Serial.printf( "static const LookupPoint %sFlowBand_P%.0f[] = {\n",
                   varPrefix, _points[0].supplyPressure_kPa);

    for (uint16_t i = 0; i < _pointCount; i++) {
        Serial.printf("    {%.3ff, %.3ff}%s  // Psupply=%.0f kPa\n",
                      _points[i].voltage_V,
                      _points[i].flow_slm,
                      (i < _pointCount - 1) ? "," : " ",
                      _points[i].supplyPressure_kPa);
    }

    Serial.println("};");
    Serial.printf( "// Table has %d entries.\n", _pointCount);
    Serial.printf( "// Load with: _valve.setFlowBand(<bandIndex>, %.1ff, %sFlowBand_P%.0f, %d);\n",
                   _points[0].supplyPressure_kPa, varPrefix,
                   _points[0].supplyPressure_kPa, _pointCount);
    Serial.println();
}
