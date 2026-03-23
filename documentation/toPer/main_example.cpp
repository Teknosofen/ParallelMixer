// main_example.cpp
// Minimal example: how to set up and run InspValveController in a bare-metal
// or RTOS environment outside of the ParallelMixer project.
//
// The only files you need from this package:
//   LocalValveController.hpp
//   LocalValveController_core.cpp
//
// Replace the three stub functions at the top with your real sensor reads
// and valve driver output.  Everything else is portable C++11.

#include "LocalValveController.hpp"

// ============================================================================
// Platform stubs — replace with your hardware calls
// ============================================================================

// Return measured flow through the inspiratory valve [slm]
static float readFlow_slm()            { return 0.0f; /* your sensor */ }

// Return supply pressure from the ABP2 (or equivalent) upstream of the valve [kPa]
static float readSupplyPressure_kPa()  { return 450.0f; /* your sensor */ }

// Return patient-circuit pressure [mbar]
static float readAirwayPressure_mbar() { return 0.0f; /* your sensor */ }

// Send the computed current to the valve driver [A]
static void  writeValveCurrent_A(float current) { (void)current; /* your DAC/driver */ }


// ============================================================================
// Measured lookup tables
// ============================================================================
// These are produced by running CC1 sweeps on the ParallelMixer and then
// reducing each sweep with tools/reduce_cc_sweep.py --normalize.
// Replace with the arrays generated for YOUR valve unit.
// Format: {V%, Flow [slm]} — must be sorted ascending by V%.

static const LookupPoint airFlowBand0[] = {   // avg Psupply ≈ 302 kPa
    { 0.00f,    0.0f},
    { 5.20f,    0.0f},
    { 5.60f,    0.35f},
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
    {10.00f,  110.50f},
};

static const LookupPoint airFlowBand1[] = {   // avg Psupply ≈ 451 kPa
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
    {10.20f,  156.80f},
};

static const LookupPoint airFlowBand2[] = {   // avg Psupply ≈ 604 kPa
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
    {11.20f,  195.50f},
};


// ============================================================================
// Controller instance
// ============================================================================

static InspValveController airValve;


// ============================================================================
// One-time setup
// ============================================================================

void setup() {

    // --- Controller configuration ---
    InspValveConfig cfg = {};

    // Flow PI — corrects residual linearisation error.
    // Feedforward handles ~90 % of the control action; these gains can be small.
    cfg.flowPI.kp           =  0.005f;   // A per slm error
    cfg.flowPI.ki           =  0.05f;    // A per (slm · s)
    cfg.flowPI.outputMin    = -0.5f;     // A  (negative allows the PI to pull back)
    cfg.flowPI.outputMax    =  0.5f;     // A
    cfg.flowPI.trackingRate =  0.3f;     // anti-windup tracking speed per cycle

    // Pressure-limit PI — ceiling controller.
    // Sits at its maximum while Paw < limit; pulls output down if Paw exceeds limit.
    cfg.pressurePI.kp           =  0.002f;
    cfg.pressurePI.ki           =  0.02f;
    cfg.pressurePI.outputMin    =  0.0f;
    cfg.pressurePI.outputMax    =  1.5f;  // A  — set above your normal flow PI output
    cfg.pressurePI.trackingRate =  0.3f;

    // Hardware current limits
    cfg.minCurrent_A = 0.0f;
    cfg.maxCurrent_A = 1.2f;

    // Cracking current: I_crack = base + k * Psupply [bar]
    // Measure on your valve; typical range: base 0.05–0.20 A, k 0.01–0.05 A/bar
    cfg.crackBaseOffset_A         = 0.12f;
    cfg.crackPressCoeff_A_per_bar = 0.03f;

    cfg.useFeedforward = true;

    airValve.begin(cfg);

    // --- Load flow surface (ascending pressure order) ---
    airValve.setFlowBand(0, 302.0f, airFlowBand0,
                         sizeof(airFlowBand0) / sizeof(airFlowBand0[0]));
    airValve.setFlowBand(1, 451.0f, airFlowBand1,
                         sizeof(airFlowBand1) / sizeof(airFlowBand1[0]));
    airValve.setFlowBand(2, 604.0f, airFlowBand2,
                         sizeof(airFlowBand2) / sizeof(airFlowBand2[0]));

    // Reset integrators (also call this whenever the ventilator stops/starts)
    airValve.reset();
}


// ============================================================================
// Control loop — call at a fixed rate, typically 100 Hz (dt = 0.01 s)
// ============================================================================

void controlLoop(float flowSetpoint_slm,
                 float pressureLimit_mbar,
                 float dt_s) {

    // --- Populate setpoints ---
    InspValveSetpoints sp;
    sp.flow_slm           = flowSetpoint_slm;
    sp.pressureLimit_mbar = pressureLimit_mbar;

    // --- Read sensors ---
    InspValveMeasurements meas;
    meas.flow_slm            = readFlow_slm();
    meas.supplyPressure_kPa  = readSupplyPressure_kPa();
    meas.airwayPressure_mbar = readAirwayPressure_mbar();

    // --- Run controller ---
    float current_A = airValve.update(sp, meas, dt_s);

    // --- Drive valve ---
    writeValveCurrent_A(current_A);

    // --- Optional: read diagnostics ---
    // InspValveStatus s = airValve.getStatus();
    // s.feedforwardCurrent_A  — feedforward contribution
    // s.flowPIOutput_A        — flow PI output (FF + correction)
    // s.pressurePIOutput_A    — pressure-limit PI output
    // s.pressureLimiting      — true = pressure ceiling is active
    // s.active                — true = valve is being driven
}


// ============================================================================
// Example main / task entry point
// ============================================================================

int main() {

    setup();

    // In a real system replace this loop with your scheduler tick or RTOS task.
    const float DT     = 0.01f;   // 100 Hz
    const float FLOW_SP   = 30.0f;  // slm — e.g. from your breath controller
    const float PRESS_LIM = 40.0f;  // mbar — inspiratory pressure safety ceiling

    while (true) {
        controlLoop(FLOW_SP, PRESS_LIM, DT);

        // your delay / task sleep for DT here
    }

    return 0;
}
