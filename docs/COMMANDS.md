# ParallelMixer Command Reference

Serial command interface for controlling the ParallelMixer system. Commands are single characters followed by optional parameters. Commands are case-insensitive.

## Quick Reference

| Command | Description | Format |
|---------|-------------|--------|
| `T` | GUI/Serial output interval | `T<microseconds>` |
| `X` | Control execution interval | `X<microseconds>` |
| `Q` | Quiet mode (output verbosity) | `Q<0-7>` |
| `M` | MUX channel selection | `M<0-5>` |
| `C` | Controller mode | `C<0-5>` |
| `F` | LLC flow setpoint (air[,O2]) | `F<SLM>[,<SLM>]` |
| `V` | Valve output (manual) | `V<0-100%>` |
| `O` | Signal generator offset | `O<0-100%>` |
| `A` | Signal generator amplitude | `A<0-100%>` |
| `S` | Signal generator period | `S<seconds>` |
| `W` | Frequency sweep config | `W<start>,<stop>,<time>[,log\|lin]` |
| `P` | PID proportional gain | `P<float>` |
| `I` | PID integral gain | `I<float>` |
| `D` | PID derivative gain | `D<float>` |
| `Z` | Zero (reset) PID integrator | `Z` |
| `E` | External/Internal PWM | `E<0\|1>` |
| `LE` | Enable/disable LLC | `LE<0\|1>` |
| `LS` | LLC status (modes + setpoints + stale flags) | `LS` |
| `LM` | Flow source mode (per actuator) | `LM`, `LM[A\|O\|B]<0\|1>` |
| `LF` | LLC manual flow test (air/O2) | `LF<SLM>[,<SLM>]` |
| `LB` | LLC blower flow test | `LB<SLM>[,<Plim_mbar>]` |
| `LP` | LLC pressure safety limit | `LP<mbar>` |
| `LX` | Stop LLC manual tests | `LX` |
| `CC` | Actuator characterization | `CC<ch>[,max[,step[,settleMs]]]` |
| `CX` | Abort characterization | `CX` |
| `?` | Show help | `?` |
| `!` | Show current settings | `!` |

---

## MUX Channel Selection (`M`)

Select which MUX channel to control and monitor. Each channel runs its signal generator independently in parallel.

```
M0          Select channel 0 - Direct (no MUX prefix)
M1          Select channel 1 - AirValve
M2          Select channel 2 - O2Valve
M3          Select channel 3 - ExpValve (Expiratory valve)
M4          Select channel 4 - NC (Not Connected/Reserved)
M5          Select channel 5 - Blower
M           Query current channelgit pull
```

**Protocol:**
- Channel 0 (Direct): Commands sent without prefix (e.g., `V50.00\n`)
- Channels 1-5: Commands prefixed with channel digit (e.g., `3V50.00\n` for ExpValve)

---

## Controller Mode (`C`)

Set the control mode for the currently selected MUX channel.

```
C0          PID control mode
C1          Valve set value (manual) mode
C2          Sine wave generator
C3          Step wave generator
C4          Triangle wave generator
C5          Frequency sweep generator
C           Query current mode
```

---

## Signal Generator Configuration

### Offset (`O`)
DC offset for signal generators (percentage of full scale).
```
O25         Set offset to 25%
O50.5       Set offset to 50.5%
O           Query current offset
```

### Amplitude (`A`)
Peak-to-peak amplitude for signal generators.
```
A20         Set amplitude to 20%
A10.5       Set amplitude to 10.5%
A           Query current amplitude
```

### Period (`S`)
Period time for periodic signal generators (Sine, Step, Triangle).
```
S1          Set period to 1 second
S0.5        Set period to 500ms
S2.5        Set period to 2.5 seconds
S           Query current period
```

### Frequency Sweep (`W`)
Configure frequency sweep parameters.
```
W0.1,10,20,log      Sweep 0.1Hz to 10Hz in 20s, logarithmic
W1,100,30,lin       Sweep 1Hz to 100Hz in 30s, linear
W0.5,5,10           Sweep 0.5Hz to 5Hz in 10s (default: linear)
W                   Query current sweep config
```

---

## Manual Valve Control (`V`)

Set valve output directly (only active in mode C1).
```
V0          Set valve to 0%
V50         Set valve to 50%
V100        Set valve to 100%
V           Query current valve signal
```

---

## PID Controller

### Proportional Gain (`P`)
```
P1.5        Set P gain to 1.5
P           Query current P gain
```

### Integral Gain (`I`)
```
I0.5        Set I gain to 0.5
I           Query current I gain
```

### Derivative Gain (`D`)
```
D0.1        Set D gain to 0.1
D           Query current D gain
```

### Reset Integrator (`Z`)
```
Z           Reset PID integrator to zero
```

### Flow Reference (`F`) — top-level LLC flow setpoint

Sets the inspiratory flow setpoint(s) fed into the Local Valve Controller (LLC).
Only active when the ventilator is OFF (use `VO0` first); otherwise the command
is ignored with a warning. The LLC routes the setpoint per actuator according to
the configured flow source mode (see `LM`):

- `LOCAL_PI` (default): runs an on-ESP PI loop that emits `V<%>` to the
  motor-controller over MUX.
- `REMOTE_F`: forwards the SLM setpoint directly as `F<SLM>` over MUX to the
  motor-controller's own flow loop.

```
F10         10 SLM on air, 0 SLM on O2
F20,5       20 SLM air, 5 SLM O2
F           Use 0 SLM on both (idle test)
```

Note: the legacy per-MUX-channel `F` (digital_flow_reference inside
`SystemConfig`) is retained in storage for compatibility but is no longer
reachable from the serial port — top-level `F` always goes through the LLC.

---

## Timing Configuration

### Output Interval (`T`)
GUI/Serial output update interval in microseconds.
```
T100000     Set to 100ms (10 Hz output)
T50000      Set to 50ms (20 Hz output)
T           Query current interval
```

### Control Interval (`X`)
Control system execution interval in microseconds.
```
X10000      Set to 10ms (100 Hz control)
X5000       Set to 5ms (200 Hz control)
X           Query current interval
```

---

## Output Mode (`Q`)

Set serial output verbosity (Q0-Q9).
```
Q0          Verbose: Pressure, Temp, Air flow, Valve
Q1          Quiet: No output
Q2          Debug: Integrator, Error, Valve, Flow
Q3          Special: Mode + MUX channel + Actuator data
Q4          Abbreviated: dP, Flow, Valve
Q5          Flow, SupplyP, Air
Q6          Full TSV (Time_ms, SupplyP, LowP, Temp, Bus0_O2, Bus0_Air, Bus1_O2, Bus1_Air, Valve, Current, FDO2_hPa, FDO2_%)
Q7          High-speed TSV (Time_ms, SupplyP, LowP, Temp, Air, Valve, Current, O2_hPa, O2%)
Q8          FDO2 O2 sensor data (pO2, O2%, Temp, Status)
Q9          FDO2 extended/raw data (Phase, Signal, Ambient, Pressure, RH)
```

---

## Usage Examples

### Example 1: Configure channel 1 for sine wave
```
M1          Select MUX channel 1
C2          Set mode to Sine
O30         Set offset to 30%
A15         Set amplitude to 15%
S2          Set period to 2 seconds
!           Verify settings
```

### Example 2: Run different patterns on multiple channels in parallel
```
# Configure channel 1 for slow sine
M1
C2
O25
A20
S3

# Configure channel 2 for fast triangle
M2
C4
O50
A10
S0.5

# Configure channel 3 for frequency sweep
M3
C5
O40
A15
W0.1,5,30,log

# All three channels now run independently in parallel!
# Switch back to view channel 1:
M1
```

### Example 3: Manual valve control
```
M0          Select direct channel (no MUX)
C1          Set mode to manual valve control
V0          Start at 0%
V25         Move to 25%
V50         Move to 50%
V75         Move to 75%
V100        Move to 100%
```

### Example 4: PID flow control
```
M1          Select channel 1
C0          Set mode to PID control
P2.0        Set proportional gain
I0.5        Set integral gain
D0          Set derivative gain (usually 0)
F10         Set LLC air flow setpoint to 10 SLM (vent OFF)
Z           Reset integrator
```

---

## Local Valve Control (LLC)

The LLC arbitrates flow between two sources per actuator and persists the
selection in NVS so it survives reboot.

### Enable / Status
```
LE1         Enable LLC
LE0         Disable LLC (pure pass-through)
LS          Print full LLC status (modes, setpoints, stale flags, gains)
```

### Flow Source Mode (`LM`)

For each inspiratory actuator (air, O2, blower) choose whether flow control
runs on this ESP or on the downstream motor-controller CPU:

```
LM          Show current modes for all three actuators
LMA0        Air valve  -> ESP local PI loop (sends V over MUX)
LMA1        Air valve  -> remote motor CPU (sends F over MUX)
LMO0        O2 valve   -> ESP local PI
LMO1        O2 valve   -> remote motor CPU
LMB0        Blower     -> ESP local PI (ESP drives PWM directly)
LMB1        Blower     -> remote motor CPU
```

All three modes default to `0` (ESP local PI) on first boot and are saved
to flash (`Preferences` namespace `pmixer`, keys `fsrc_air`/`fsrc_o2`/`fsrc_blw`)
whenever they change — whether the change is made via serial or via the
web UI.

### Remote Flow Telemetry & Stale Detection

When a channel is in `REMOTE_F` mode the LLC uses the flow measurement returned
by that motor-controller over the MUX (via `getActualFlow`) instead of the
on-board SFM3505. If no fresh value arrives within `REMOTE_FLOW_STALE_MS`
(200 ms), the channel is flagged stale:

- Flow used for control falls back to `0.0` for safety.
- Serial console emits a once-per-second warning:
  `[STALE] Remote flow telemetry: Air=STALE ... (>200ms)`
- Web UI shows a red "REMOTE FLOW STALE" badge plus a per-channel "⚠ stale"
  marker next to the affected dropdown.
- `LS` output marks the offending channel.

### LLC Manual Flow Test (`LF`, `LB`)
```
LF10        10 SLM air, 0 O2 (vent must be OFF)
LF20,5      20 SLM air, 5 SLM O2
LF          Show current test status
LB50        50 SLM blower, default Plim 40 mbar
LB100,30    100 SLM blower, Plim 30 mbar
LP35        Set pressure safety limit to 35 mbar
LX          Stop all LLC manual tests
```

### Actuator Characterization (`CC`, `CX`)
```
CC1                 Air valve, defaults (max 12 %V, step 0.1, settle 200 ms)
CC2,12,0.1          O2 valve sweep at supply pressure
CC3,12,0.1          Exp valve sweep (set flow target first via LF)
CC4,100,2,300       Blower PWM 0–100 %, 2 % step, 300 ms settle
CX                  Abort sweep
```

---

## Web Endpoints (HTTP)

In addition to the serial interface, the WiFi server exposes:

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Live UI (charts + ventilator panel + flow source card) |
| `/data` | GET | Single sample JSON (incl. `fsrcAir`/`fsrcO2`/`fsrcBlw`, `flowStale`, `airStale`, `o2Stale`, `blwStale`) |
| `/dataBuffer` | GET | High-speed sample batch for charts |
| `/flowsource` | GET | Set LLC flow source modes: `?air=0\|1&o2=0\|1&blw=0\|1`. Persists to NVS and returns the current state. |

---

## Serial Protocol (MUX Router)

### Message Format
```
Without MUX (channel 0):  <CMD><VALUE>\n     Example: V50.00\n
With MUX (channels 1-5):  <N><CMD><VALUE>\n  Example: 3V50.00\n
```

### Command Characters
| Char | Send (to actuator) | Receive (from actuator) |
|------|-------------------|------------------------|
| `V` | Set Valve % | - |
| `I` | Set Current | Current measurement |
| `U` | Set Voltage | - |
| `F` | Set Flow | Flow measurement |
| `P` | Set Pressure | Pressure measurement |
| `R` | Set Blower RPM | RPM measurement |

### Example Protocol Messages
```
V50.00\n        -> Set valve to 50% (channel 0, direct)
3V50.00\n       -> Set valve to 50% (channel 3, via MUX)
I1.25\n         <- Current reading 1.25A (channel 0)
3I1.25\n        <- Current reading 1.25A from channel 3
```
