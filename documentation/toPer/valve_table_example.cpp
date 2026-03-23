// valve_table_example.cpp
// Self-contained example showing how to configure and run InspValveController.
//
// This file does NOT compile as-is (it omits Arduino / project headers) but
// illustrates every step your integration must cover:
//
//   1.  Measured lookup tables (produced by CC sweeps + reduce_cc_sweep.py)
//   2.  Controller configuration
//   3.  Table loading at startup
//   4.  Per-cycle update call
//   5.  Reading diagnostic status
//
// See INTEGRATION_GUIDE.md for the full workflow.

#include "LocalValveController.hpp"


// ============================================================================
// 1.  Measured flow tables — one array per CC sweep
// ============================================================================
//
// These values come from running:
//   CC1,14,0.2,250   at three different supply pressures
// then reducing each raw sweep to ≤16 representative points with:
//   python tools/reduce_cc_sweep.py sweep_3bar.txt sweep_4.5bar.txt sweep_6bar.txt --normalize
//
// Format: {V%, Flow [slm]}
// x = drive percentage sent to the valve (0–100 %)
// y = measured flow in standard litres per minute

// --- Band 0: avg supply pressure ≈ 302 kPa (3.02 bar) ---
static const LookupPoint airFlowBand0[] = {
    { 0.00f,    0.0f},   // dead zone
    { 5.20f,    0.0f},   // still closed
    { 5.60f,    0.35f},  // cracking
    { 6.00f,    2.14f},
    { 6.40f,    6.89f},
    { 6.80f,   14.02f},
    { 7.20f,   23.51f},
    { 7.60f,   35.10f},
    { 8.00f,   49.22f},
    { 8.40f,   64.70f},
    { 8.80f,   82.15f},
    { 9.20f,   98.40f},
    { 9.60f,  108.20f},
    {10.00f,  110.50f},  // saturated
};
static const uint8_t airFlowBand0_count = sizeof(airFlowBand0) / sizeof(airFlowBand0[0]);

// --- Band 1: avg supply pressure ≈ 451 kPa (4.51 bar) ---
static const LookupPoint airFlowBand1[] = {
    { 0.00f,    0.0f},
    { 5.00f,    0.0f},
    { 5.40f,    0.19f},
    { 5.80f,    1.05f},
    { 6.20f,    4.32f},
    { 6.60f,   11.08f},
    { 7.00f,   21.50f},
    { 7.40f,   34.80f},
    { 7.80f,   50.22f},
    { 8.20f,   68.90f},
    { 8.60f,   89.10f},
    { 9.00f,  112.30f},
    { 9.40f,  138.50f},
    { 9.80f,  155.20f},
    {10.20f,  156.80f},  // saturated
};
static const uint8_t airFlowBand1_count = sizeof(airFlowBand1) / sizeof(airFlowBand1[0]);

// --- Band 2: avg supply pressure ≈ 604 kPa (6.04 bar) ---
static const LookupPoint airFlowBand2[] = {
    { 0.00f,    0.0f},
    { 5.00f,    0.0f},
    { 5.60f,    0.17f},
    { 6.00f,    0.91f},
    { 6.40f,    4.08f},
    { 6.80f,   12.89f},
    { 7.20f,   22.48f},
    { 7.60f,   34.40f},
    { 8.00f,   48.91f},
    { 8.40f,   68.54f},
    { 8.80f,   84.81f},
    { 9.20f,  103.18f},
    { 9.60f,  127.14f},
    {10.00f,  160.04f},
    {10.60f,  194.88f},
    {11.20f,  195.50f},  // saturated
};
static const uint8_t airFlowBand2_count = sizeof(airFlowBand2) / sizeof(airFlowBand2[0]);


// ============================================================================
// 2.  Controller instance and configuration
// ============================================================================

static InspValveController airValve;

static void configureAirValve() {

    InspValveConfig cfg = {};

    // PI gains — flow controller
    // Units: A per (slm error) for kp; A per (slm·s) for ki
    // Start conservative; feedforward handles ~90 % of the control action.
    cfg.flowPI.kp          =  0.005f;
    cfg.flowPI.ki          =  0.05f;
    cfg.flowPI.outputMin   = -0.5f;
    cfg.flowPI.outputMax   =  0.5f;
    cfg.flowPI.trackingRate =  0.3f;   // anti-windup tracking speed (~per call)

    // PI gains — pressure-limit controller
    // This PI normally sits at its upper clamp (non-limiting).
    // Only activates when Paw > pressureLimit_mbar.
    cfg.pressurePI.kp          =  0.002f;
    cfg.pressurePI.ki          =  0.02f;
    cfg.pressurePI.outputMin   =  0.0f;
    cfg.pressurePI.outputMax   =  1.5f;
    cfg.pressurePI.trackingRate =  0.3f;

    // Hardware current limits [A]
    cfg.minCurrent_A = 0.0f;
    cfg.maxCurrent_A = 1.2f;

    // Cracking current: I_crack = base + k * P_supply [bar]
    // Measure by slowly raising current until the valve first cracks open.
    cfg.crackBaseOffset_A         = 0.12f;   // A (spring only, zero supply pressure)
    cfg.crackPressCoeff_A_per_bar = 0.03f;   // A per bar of supply pressure

    cfg.useFeedforward = true;

    airValve.begin(cfg);
}


// ============================================================================
// 3.  Load tables at startup — MUST be in ascending pressure order
// ============================================================================

static void loadAirValveTables() {
    airValve.setFlowBand(0, 302.0f, airFlowBand0, airFlowBand0_count);
    airValve.setFlowBand(1, 451.0f, airFlowBand1, airFlowBand1_count);
    airValve.setFlowBand(2, 604.0f, airFlowBand2, airFlowBand2_count);
}


// ============================================================================
// 4.  Per-cycle update — call at your control rate (typically 100 Hz)
// ============================================================================
//
// Sensor units:
//   flow_slm            — standard litres per minute  (SFM3505)
//   supplyPressure_kPa  — kilopascals                 (ABP2)
//   airwayPressure_mbar — millibar                    (ELVH)

static float runAirValveCycle(
    float flowSetpoint_slm,
    float pressureLimit_mbar,
    float measuredFlow_slm,
    float supplyPressure_kPa,
    float airwayPressure_mbar,
    float dt_s)
{
    InspValveSetpoints sp;
    sp.flow_slm          = flowSetpoint_slm;
    sp.pressureLimit_mbar = pressureLimit_mbar;

    InspValveMeasurements meas;
    meas.flow_slm            = measuredFlow_slm;
    meas.supplyPressure_kPa  = supplyPressure_kPa;
    meas.airwayPressure_mbar = airwayPressure_mbar;

    float current_A = airValve.update(sp, meas, dt_s);

    // current_A → your valve driver / DAC here

    return current_A;
}


// ============================================================================
// 5.  Reading diagnostic status (optional)
// ============================================================================

static void printValveStatus() {
    InspValveStatus s = airValve.getStatus();

    // s.outputCurrent_A      — final commanded current
    // s.feedforwardCurrent_A — feedforward contribution only
    // s.flowPIOutput_A       — flow PI output (feedforward + PI correction)
    // s.pressurePIOutput_A   — pressure-limit PI output
    // s.pressureLimiting     — true = pressure PI is the active controller
    // s.active               — true = valve is being driven (flow sp > 0)

    Serial.printf("I=%.3fA  FF=%.3fA  flowPI=%.3fA  presPI=%.3fA  pLim=%d\n",
        s.outputCurrent_A,
        s.feedforwardCurrent_A,
        s.flowPIOutput_A,
        s.pressurePIOutput_A,
        (int)s.pressureLimiting);
}


// ============================================================================
// Worked runtime trace — 50 slm requested, Psupply = 500 kPa
// ============================================================================
//
// findBrackets(500 kPa):
//   band 1 (451 kPa) and band 2 (604 kPa)
//   t = (500 - 451) / (604 - 451) ≈ 0.32
//
// Band 1 inverseLookup(50 slm):
//   bracket {7.80, 50.22} → V% ≈ 7.79 %
//
// Band 2 inverseLookup(50 slm):
//   bracket {8.00, 48.91} and {8.40, 68.54}
//   t2 = (50 - 48.91) / (68.54 - 48.91) ≈ 0.056
//   V% = 8.00 + 0.056 * 0.40 ≈ 8.02 %
//
// Blend:
//   V%_ff = 7.79 + 0.32 * (8.02 - 7.79) = 7.86 %
//
// Flow PI adds a small correction for any residual error.
// Cracking current (e.g., 0.12 + 0.03 * 5.0 = 0.27 A) is added last.
