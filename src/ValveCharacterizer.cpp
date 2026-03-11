#include "ValveCharacterizer.hpp"
#include "SerialMuxRouter.hpp"
#include "SensorReader.hpp"
#include "ActuatorControl.hpp"

// ============================================================================
// Constructor
// ============================================================================
ValveCharacterizer::ValveCharacterizer()
    : _muxRouter(nullptr),
      _blowerActuator(nullptr),
      _state(CHAR_IDLE),
      _charMode(CHARMODE_INSP_VALVE),
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

void ValveCharacterizer::begin(SerialMuxRouter* muxRouter, ActuatorControl* blowerActuator) {
    _muxRouter = muxRouter;
    _blowerActuator = blowerActuator;
}

// ============================================================================
// Start / Abort
// ============================================================================
bool ValveCharacterizer::start(const CharacterizationConfig& config) {
    if (_state != CHAR_IDLE) {
        Serial.println("[CHAR] Already running — send CX to abort first");
        return false;
    }

    // Determine mode from channel
    switch (config.muxChannel) {
        case 1: case 2:
            _charMode = CHARMODE_INSP_VALVE;
            break;
        case 3:
            _charMode = CHARMODE_EXP_VALVE;
            break;
        case 4:
            _charMode = CHARMODE_BLOWER;
            if (!_blowerActuator) {
                Serial.println("[CHAR] Blower actuator not configured (ch4)");
                return false;
            }
            break;
        default:
            Serial.printf("[CHAR] Invalid channel %d — use 1(air), 2(O2), 3(exp), 4(blower)\n",
                          config.muxChannel);
            return false;
    }

    if (!_muxRouter && _charMode != CHARMODE_BLOWER) {
        Serial.println("[CHAR] No MUX router configured");
        return false;
    }

    _config = config;

    // Sensible defaults if zero
    if (_config.maxVoltage <= 0.0f) {
        _config.maxVoltage = (_charMode == CHARMODE_BLOWER) ? 100.0f : 12.0f;
    }
    if (_config.stepVoltage <= 0.0f) {
        _config.stepVoltage = (_charMode == CHARMODE_BLOWER) ? 1.0f : 0.1f;
    }
    if (_config.settleTime_ms == 0) _config.settleTime_ms = 200;
    if (_config.samplesPerStep == 0) _config.samplesPerStep = 10;

    _totalSteps = (uint16_t)(_config.maxVoltage / _config.stepVoltage) + 1;
    _currentStep = 0;
    _currentVoltage = 0.0f;
    _pointCount = 0;

    printHeader();

    // Set initial output and start settling
    setOutput(0.0f);
    _state = CHAR_RAMP_SETTLE;
    _stateEntryTime_ms = millis();

    return true;
}

void ValveCharacterizer::abort() {
    if (_state == CHAR_IDLE) return;

    setOutput(0.0f);
    _state = CHAR_IDLE;
    Serial.println("[CHAR] Aborted — output set to 0");
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
                float deltaP = 0.0f;
                float cv = 0.0f;
                if (_charMode == CHARMODE_INSP_VALVE) {
                    // Insp valve: deltaP = Psupply - Paw
                    deltaP = avgPressure - avgPaw / 10.0f;
                } else if (_charMode == CHARMODE_EXP_VALVE) {
                    // Exp valve: deltaP = Paw (exhausting to atmosphere)
                    deltaP = avgPaw / 10.0f;  // mbar → kPa
                }
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
                    setOutput(0.0f);
                    _state = CHAR_DONE;
                    Serial.println("# --- Sweep complete ---");
                    printSummaryTable();
                    _state = CHAR_IDLE;
                } else {
                    // Move to next step
                    setOutput(_currentVoltage);
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
    // Insp valves: flow from same bus as MUX channel
    // Blower / Exp valve: flow always from bus0 (air flow sensor)
    if (_config.muxChannel == 2) return bus1.sfm3505_air_flow;
    return bus0.sfm3505_air_flow;  // ch 1, 3, 4 all use bus0
}

float ValveCharacterizer::getSupplyPressure(const SensorData& bus0,
                                             const SensorData& bus1) const {
    if (_config.muxChannel == 2) return bus1.supply_pressure;
    return bus0.supply_pressure;  // ch 1, 3, 4
}

void ValveCharacterizer::setOutput(float percent) {
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    if (_charMode == CHARMODE_BLOWER && _blowerActuator) {
        // Direct PWM on GPIO21 via ActuatorControl
        _blowerActuator->outputToValve(percent);
    } else if (_muxRouter) {
        // MUX V% command for inspiratory or expiratory valves
        _muxRouter->sendCommand(_config.muxChannel, 'V', percent);
    }
}

// ============================================================================
// Output Formatting
// ============================================================================
static const char* channelName(uint8_t ch) {
    switch (ch) {
        case 1: return "Air";
        case 2: return "O2";
        case 3: return "Exp";
        case 4: return "Blower";
        default: return "??";
    }
}

void ValveCharacterizer::printHeader() {
    Serial.println();
    Serial.println("# ============================================================");

    switch (_charMode) {
    case CHARMODE_INSP_VALVE:
        Serial.printf( "# INSP VALVE CHARACTERIZATION — %s valve (MUX %d)\n",
                       channelName(_config.muxChannel), _config.muxChannel);
        Serial.printf( "# maxV=%.2f  stepV=%.3f  settle=%lums  samples=%d\n",
                       _config.maxVoltage, _config.stepVoltage,
                       _config.settleTime_ms, _config.samplesPerStep);
        Serial.println("# Columns: Result, Control, Modifier, (extra), Cv");
        Serial.println("# ============================================================");
        Serial.println("# Flow(slm),\tV(%),\tPsupply(kPa),\tPaw(mbar),\tCv");
        break;

    case CHARMODE_BLOWER:
        Serial.printf( "# BLOWER CHARACTERIZATION — GPIO21 PWM (ch %d)\n",
                       _config.muxChannel);
        Serial.printf( "# maxPWM=%.1f%%  step=%.2f%%  settle=%lums  samples=%d\n",
                       _config.maxVoltage, _config.stepVoltage,
                       _config.settleTime_ms, _config.samplesPerStep);
        Serial.println("# Apply fixed counter pressure, repeat at different pressures.");
        Serial.println("# Columns: Result, Control, Modifier, (extra)");
        Serial.println("# ============================================================");
        Serial.println("# Flow(slm),\tPWM(%),\tPaw(mbar),\tPsupply(kPa)");
        break;

    case CHARMODE_EXP_VALVE:
        Serial.printf( "# EXP VALVE CHARACTERIZATION — MUX %d\n", _config.muxChannel);
        Serial.printf( "# maxV=%.2f  stepV=%.3f  settle=%lums  samples=%d\n",
                       _config.maxVoltage, _config.stepVoltage,
                       _config.settleTime_ms, _config.samplesPerStep);
        Serial.println("# Flow is held by LF but will vary as Paw builds — both are recorded.");
        Serial.println("# Columns: Result, Control, Modifier, (extra), Cv");
        Serial.println("# ============================================================");
        Serial.println("# Paw(mbar),\tV(%),\tFlow(slm),\tPsupply(kPa),\tCv");
        break;
    }
}

void ValveCharacterizer::printDataRow(const CharacterizationPoint& pt) {
    switch (_charMode) {
    case CHARMODE_INSP_VALVE:
        // Result(Flow), Control(V%), Modifier(Psupply), Paw, Cv
        Serial.printf("%.3f,\t%.3f,\t\t%.1f,\t\t%.2f,\t\t%.5f\n",
                      pt.flow_slm, pt.voltage_V, pt.supplyPressure_kPa,
                      pt.pawPressure_mbar, pt.cv);
        break;
    case CHARMODE_BLOWER:
        // Result(Flow), Control(PWM%), Modifier(Paw), Psupply
        Serial.printf("%.3f,\t%.2f,\t\t%.2f,\t\t%.1f\n",
                      pt.flow_slm, pt.voltage_V, pt.pawPressure_mbar,
                      pt.supplyPressure_kPa);
        break;
    case CHARMODE_EXP_VALVE:
        // Result(Paw), Control(V%), Modifier(Flow), Psupply, Cv
        Serial.printf("%.2f,\t%.3f,\t\t%.3f,\t\t%.1f,\t\t%.5f\n",
                      pt.pawPressure_mbar, pt.voltage_V, pt.flow_slm,
                      pt.supplyPressure_kPa, pt.cv);
        break;
    }
}

void ValveCharacterizer::printSummaryTable() {
    if (_pointCount == 0) return;

    const char* name = channelName(_config.muxChannel);

    switch (_charMode) {
    case CHARMODE_INSP_VALVE: {
        // Compute average supply pressure for this sweep
        float avgPsupply = 0;
        for (uint16_t i = 0; i < _pointCount; i++) avgPsupply += _points[i].supplyPressure_kPa;
        avgPsupply /= _pointCount;

        const char* varPrefix = (_config.muxChannel == 1) ? "air" : "o2";

        Serial.println();
        Serial.println("// ============================================================");
        Serial.printf( "// %s valve flow table from characterization\n", name);
        Serial.printf( "// Avg Psupply ~ %.0f kPa (%.2f bar)\n", avgPsupply, avgPsupply / 100.0f);
        Serial.printf( "// MUX channel %d, %d raw points\n", _config.muxChannel, _pointCount);
        Serial.println("// x = V%, y = Flow (slm)");
        Serial.println("// NOTE: Reduce to <=16 points before loading into PressureBandedTable");
        Serial.println("// ============================================================");
        Serial.printf( "static const LookupPoint %sFlowBand_P%.0f[] = {\n",
                       varPrefix, avgPsupply);
        for (uint16_t i = 0; i < _pointCount; i++) {
            Serial.printf("    {%.3ff, %.3ff}%s  // Psup=%.0f kPa, Cv=%.4f\n",
                          _points[i].voltage_V, _points[i].flow_slm,
                          (i < _pointCount - 1) ? "," : " ",
                          _points[i].supplyPressure_kPa, _points[i].cv);
        }
        Serial.println("};");
        Serial.printf("// Load with: _valve.setFlowBand(<bandIdx>, %.1ff, %sFlowBand_P%.0f, %d);\n",
                      avgPsupply, varPrefix, avgPsupply, _pointCount);
        break;
    }

    case CHARMODE_BLOWER: {
        // Compute average counter pressure for this sweep
        float avgPaw = 0;
        for (uint16_t i = 0; i < _pointCount; i++) avgPaw += _points[i].pawPressure_mbar;
        avgPaw /= _pointCount;

        Serial.println();
        Serial.println("// ============================================================");
        Serial.printf( "// Blower flow table from characterization\n");
        Serial.printf( "// Avg counter pressure ~ %.1f mbar\n", avgPaw);
        Serial.printf( "// %d raw points\n", _pointCount);
        Serial.println("// x = PWM%, y = Flow (slm)");
        Serial.println("// Combine sweeps at different counter pressures for 2D surface");
        Serial.println("// ============================================================");
        Serial.printf( "static const LookupPoint blowerFlowBand_P%.0f[] = {\n", avgPaw);
        for (uint16_t i = 0; i < _pointCount; i++) {
            Serial.printf("    {%.2ff, %.3ff}%s  // Paw=%.1f mbar\n",
                          _points[i].voltage_V, _points[i].flow_slm,
                          (i < _pointCount - 1) ? "," : " ",
                          _points[i].pawPressure_mbar);
        }
        Serial.println("};");
        Serial.printf("// Load with: blowerSurface.loadBand(<bandIdx>, %.1ff, blowerFlowBand_P%.0f, %d);\n",
                      avgPaw, avgPaw, _pointCount);
        break;
    }

    case CHARMODE_EXP_VALVE: {
        // Compute average flow for this sweep (condition variable, like Psupply for insp)
        float avgFlow = 0;
        for (uint16_t i = 0; i < _pointCount; i++) avgFlow += _points[i].flow_slm;
        avgFlow /= _pointCount;

        Serial.println();
        Serial.println("// ============================================================");
        Serial.printf( "// Exp valve pressure table from characterization\n");
        Serial.printf( "// Avg flow ~ %.1f slm (condition variable for 2D surface)\n", avgFlow);
        Serial.printf( "// MUX channel %d, %d raw points\n", _config.muxChannel, _pointCount);
        Serial.println("// x = V%, y = Paw (mbar)");
        Serial.println("// Flow varies as Paw builds — each point records actual flow.");
        Serial.println("// NOTE: Reduce to <=16 points before loading into PressureBandedTable");
        Serial.println("// ============================================================");
        Serial.printf( "static const LookupPoint expPressureBand_F%.0f[] = {\n", avgFlow);
        for (uint16_t i = 0; i < _pointCount; i++) {
            Serial.printf("    {%.3ff, %.2ff}%s  // Flow=%.1f slm, Cv=%.4f\n",
                          _points[i].voltage_V, _points[i].pawPressure_mbar,
                          (i < _pointCount - 1) ? "," : " ",
                          _points[i].flow_slm, _points[i].cv);
        }
        Serial.println("};");
        Serial.printf("// Load with: expSurface.loadBand(<bandIdx>, %.1ff, expPressureBand_F%.0f, %d);\n",
                      avgFlow, avgFlow, _pointCount);
        break;
    }
    }

    Serial.printf("// Table has %d entries.\n", _pointCount);
    Serial.println();
}
