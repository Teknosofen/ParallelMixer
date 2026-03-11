#!/usr/bin/env python3
"""
reduce_cc_sweep.py — Reduce CC sweep data to ≤16 points for PressureBandedTable.

Parses the raw serial output from CC1/CC2/CC3/CC4 commands, automatically
detects the actuator type, reduces to ≤16 representative points using
the region-based strategy (dead zone, cracking, active, saturation),
and outputs ready-to-paste C++ code.

Usage:
  python reduce_cc_sweep.py                     # File dialog, or paste interactively
  python reduce_cc_sweep.py sweep1.txt ...      # From saved serial output files
  python reduce_cc_sweep.py --max-points 12 ... # Limit points (default: 16)
  python reduce_cc_sweep.py --normalize ...     # Correct for supply pressure droop

Multiple CC sweeps can be in the same file — the script splits on each
new "# ====" header block automatically.

Options:
  --max-points N, -m N   Maximum points per band (default: 16)
  --normalize, -n        Normalize insp valve flows for supply pressure droop.
                         Adjusts each flow to what it would be at the band's
                         reference pressure: Q_norm = Q_meas × sqrt(P_ref/P_actual).
                         Uses per-point Psupply from the CSV data. Has no effect
                         on blower or exp valve data.
  --help, -h             Show this help text.

Supports: Insp valves (ch1/2), Exp valve (ch3), Blower (ch4).
"""

import sys
import re
import math

MAX_POINTS_DEFAULT = 16


# ─────────────────────────────────────────────────────────────
#  Splitting & Parsing
# ─────────────────────────────────────────────────────────────

def split_sweeps(text_lines):
    """Split a file containing multiple CC outputs into separate sweep blocks.

    Each sweep starts with a '# ====...' header line. Returns a list of
    line-lists, one per sweep found. If no header is found the entire
    input is returned as a single block.
    """
    blocks = []
    current = []
    for line in text_lines:
        stripped = line.strip()
        # A new sweep header starts with '# ====' (at least 6 '=')
        if stripped.startswith('# ===='):
            # If we already accumulated CSV data, save the block
            if current:
                blocks.append(current)
                current = []
        current.append(line)
    if current:
        blocks.append(current)
    return blocks if blocks else [text_lines]


def parse_sweep(text_lines):
    """Parse CC serial output into structured sweep data.

    Returns dict with keys: type, control[], result[], condition,
    raw_rows[], n_raw, channel, name.
    Returns None if parsing fails.
    """
    actuator_type = None
    channel = 0
    name = ""
    avg_condition = None
    data_rows = []
    in_code_table = False  # Skip the code-ready table at the end

    for line in text_lines:
        line = line.rstrip('\n\r')
        stripped = line.strip()
        if not stripped:
            continue

        # ── Detect actuator type from the CSV header ──
        if stripped.startswith('#') and 'V(%)' in stripped:
            if stripped.startswith('# Flow') and 'Psupply' in stripped and 'PWM' not in stripped:
                actuator_type = 'insp'
            elif stripped.startswith('# Flow') and 'PWM(%)' in stripped:
                actuator_type = 'blower'
            elif stripped.startswith('# Paw'):
                actuator_type = 'exp'
            continue

        # ── Channel / name from header ──
        m = re.search(r'MUX (\d)', stripped)
        if m:
            channel = int(m.group(1))
        if 'Air valve' in stripped or 'air valve' in stripped:
            name = 'Air'
        elif 'O2 valve' in stripped or 'o2 valve' in stripped:
            name = 'O2'
        elif 'EXP VALVE' in stripped or 'Exp valve' in stripped:
            name = 'Exp'
        elif 'BLOWER' in stripped or 'Blower' in stripped:
            name = 'Blower'

        # ── Average condition from code-ready summary ──
        m2 = re.search(r'Avg Psupply\s*~\s*([\d.]+)\s*kPa', stripped)
        if m2:
            avg_condition = float(m2.group(1))
        m3 = re.search(r'Avg counter pressure\s*~\s*([\d.]+)\s*mbar', stripped)
        if m3:
            avg_condition = float(m3.group(1))
        m4 = re.search(r'Avg flow\s*~\s*([\d.]+)\s*slm', stripped, re.IGNORECASE)
        if m4:
            avg_condition = float(m4.group(1))

        # ── Detect start of code-ready table (stop parsing CSV) ──
        if stripped.startswith('static const') or stripped.startswith('// ===='):
            in_code_table = True
        if stripped == '# --- Sweep complete ---':
            in_code_table = True
            continue

        # ── Skip comments and code lines ──
        if stripped.startswith('#') or stripped.startswith('//') or in_code_table:
            continue

        # ── Parse CSV data row ──
        # Format: "0.000,\t0.001,\t\t603.7,\t\t-0.09,\t\t0.00000"
        parts = re.split(r'[,\t]+', stripped)
        parts = [p.strip().rstrip('f') for p in parts if p.strip()]
        if len(parts) >= 3:
            try:
                values = [float(p) for p in parts]
                data_rows.append(values)
            except ValueError:
                continue

    if not data_rows or not actuator_type:
        return None

    # All formats: col 0 = Result, col 1 = Control, col 2 = Modifier
    control = [row[1] for row in data_rows]
    result  = [row[0] for row in data_rows]

    # Compute average condition from raw data if not found in header
    if avg_condition is None and len(data_rows[0]) >= 3:
        avg_condition = sum(row[2] for row in data_rows) / len(data_rows)

    return {
        'type':      actuator_type,
        'control':   control,
        'result':    result,
        'condition': avg_condition or 0.0,
        'raw_rows':  data_rows,
        'n_raw':     len(data_rows),
        'channel':   channel,
        'name':      name,
    }


# ─────────────────────────────────────────────────────────────
#  Pressure Droop Normalization
# ─────────────────────────────────────────────────────────────

def normalize_flows(sweep):
    """Normalize inspiratory valve flows to compensate for supply pressure droop.

    During a CC sweep the supply pressure drops as flow increases (regulator
    capacity limit).  Each raw CSV row records the *actual* Psupply at that
    operating point.  This function adjusts every flow measurement to what it
    would be at the band's reference pressure (the average Psupply) using the
    orifice equation:

        Q_norm = Q_meas × sqrt(P_ref / P_actual)

    Region-by-region impact:
        Dead zone  (flow ≈ 0):  0 × anything = 0 — unchanged.
        Cracking   (tiny flow): Psupply still near nominal — factor ≈ 1.0.
        Active     (bulk):      Largest corrections — exactly where droop is
                                greatest and feedforward accuracy matters most.
        Saturation (valve open): Also corrected — the flow limit at the drooped
                                pressure scales by sqrt(P) just like an orifice.

    Only applies to inspiratory valves (type == 'insp') whose modifier column
    (column 2) contains per-point Psupply (kPa).

    Returns (n_corrected, max_correction_slm) or None if not applicable.
    """
    if sweep['type'] != 'insp':
        return None  # Only insp valves have Psupply droop

    p_ref = sweep['condition']  # Average Psupply = band reference pressure
    if p_ref <= 0:
        return None

    raw_rows = sweep['raw_rows']
    result   = sweep['result']

    max_correction = 0.0
    n_corrected = 0

    for i in range(len(result)):
        if len(raw_rows[i]) < 3:
            continue  # No pressure column

        p_actual = raw_rows[i][2]  # Per-point Psupply (kPa)

        if p_actual <= 0 or abs(result[i]) < 1e-6:
            continue  # Skip zero-flow or invalid pressure

        ratio = math.sqrt(p_ref / p_actual)
        old_flow = result[i]
        result[i] = old_flow * ratio

        correction = abs(result[i] - old_flow)
        if correction > max_correction:
            max_correction = correction
        n_corrected += 1

    return n_corrected, max_correction


# ─────────────────────────────────────────────────────────────
#  Point Reduction
# ─────────────────────────────────────────────────────────────

def reduce_points(control, result, max_points=16):
    """Reduce sweep data to ≤max_points representative indices.

    Strategy (from VALVE_CALIBRATION.md Section 5):
      - Dead zone:  2 points  (first + last zero-result)
      - Cracking:   2-3 points (transition from zero)
      - Active:     equal result spacing
      - Saturation: 1-2 points (where result flattens)
    """
    n = len(control)
    if n <= max_points:
        return list(range(n))

    result_abs_max = max(abs(r) for r in result)
    if result_abs_max == 0:
        return [0, n - 1]

    # ── 1. Dead zone: result ≈ 0 ──
    dead_threshold = result_abs_max * 0.001  # 0.1% of full scale
    dead_end = 0
    for i in range(n):
        if abs(result[i]) <= dead_threshold:
            dead_end = i
        else:
            break

    # ── 2. Saturation: result stops increasing ──
    # Walk backwards from the end; find the last point where the slope
    # is still > 2% of the average active slope.
    sat_start = n - 1
    active_result_range = result_abs_max - abs(result[min(dead_end + 1, n - 1)])
    if active_result_range > 0 and n > 5:
        for i in range(n - 1, dead_end + 3, -1):
            dx = abs(control[i] - control[i - 1])
            if dx < 1e-9:
                continue
            local_slope = abs(result[i] - result[i - 1]) / dx
            # Compare with overall average slope in active region
            overall_dx = abs(control[sat_start] - control[dead_end + 1])
            if overall_dx > 0:
                avg_slope = active_result_range / overall_dx
                if local_slope > avg_slope * 0.10:  # Still >10% of avg active slope
                    sat_start = i
                    break

    # ── Allocate indices ──
    selected = set()

    # Dead zone: keep first and last zero
    selected.add(0)
    if dead_end > 0:
        selected.add(dead_end)

    # Cracking: 2–3 points right after dead zone
    crack_start = dead_end + 1
    crack_pts = min(3, max(0, sat_start - crack_start))
    for i in range(crack_pts):
        idx = crack_start + i
        if idx < n:
            selected.add(idx)

    # Saturation: last active + end
    if sat_start < n - 1:
        selected.add(sat_start)
        selected.add(n - 1)
    else:
        selected.add(n - 1)

    # Active region: fill remaining budget at equal result spacing
    remaining = max_points - len(selected)
    active_lo = crack_start + crack_pts
    active_hi = sat_start

    if remaining > 0 and active_hi > active_lo:
        r_lo = result[active_lo]
        r_hi = result[active_hi]

        if abs(r_hi - r_lo) > 1e-9:
            # Generate evenly spaced target result values
            targets = [
                r_lo + (j + 1) / (remaining + 1) * (r_hi - r_lo)
                for j in range(remaining)
            ]
            for target in targets:
                best_idx = active_lo
                best_dist = float('inf')
                for i in range(active_lo, active_hi + 1):
                    d = abs(result[i] - target)
                    if d < best_dist:
                        best_dist = d
                        best_idx = i
                selected.add(best_idx)

    indices = sorted(selected)

    # Safety: trim to max in case rounding gave one extra
    return indices[:max_points]


# ─────────────────────────────────────────────────────────────
#  Interpolation Error Check
# ─────────────────────────────────────────────────────────────

def compute_max_error(control, result, indices):
    """Compute the maximum interpolation error vs. all raw points."""
    sel_x = [control[i] for i in indices]
    sel_y = [result[i]  for i in indices]
    max_err = 0.0
    max_err_idx = 0

    for i in range(len(control)):
        x = control[i]
        y_actual = result[i]
        # Piecewise linear prediction from reduced table
        y_pred = sel_y[0]
        if x <= sel_x[0]:
            y_pred = sel_y[0]
        elif x >= sel_x[-1]:
            y_pred = sel_y[-1]
        else:
            for j in range(len(sel_x) - 1):
                if sel_x[j] <= x <= sel_x[j + 1]:
                    dx = sel_x[j + 1] - sel_x[j]
                    if dx > 0:
                        t = (x - sel_x[j]) / dx
                        y_pred = sel_y[j] + t * (sel_y[j + 1] - sel_y[j])
                    break

        err = abs(y_actual - y_pred)
        if err > max_err:
            max_err = err
            max_err_idx = i

    return max_err, max_err_idx


# ─────────────────────────────────────────────────────────────
#  C++ Code Generation
# ─────────────────────────────────────────────────────────────

def generate_cpp(sweep, indices, band_index=0):
    """Generate C++ code for one reduced band."""
    t = sweep['type']
    cond = sweep['condition']
    n_pts = len(indices)

    if t == 'insp':
        ch = sweep['channel']
        prefix = 'air' if ch == 1 else 'o2'
        array_name = f'{prefix}FlowBand{band_index}'
        cond_str = f'Psupply ~ {cond:.0f} kPa ({cond / 100:.2f} bar)'
        x_label, y_label = 'V%', 'Flow (slm)'
        load_fn = (f'_{prefix}Valve.setFlowBand({band_index}, {cond:.1f}f, '
                   f'{array_name}, {n_pts});')
    elif t == 'exp':
        array_name = f'expPressureBand{band_index}'
        cond_str = f'avg flow ~ {cond:.1f} slm'
        x_label, y_label = 'V%', 'Paw (mbar)'
        load_fn = (f'_expValve.setPressureBand({band_index}, {cond:.1f}f, '
                   f'{array_name}, {n_pts});')
    else:  # blower
        array_name = f'blowerFlowBand{band_index}'
        cond_str = f'avg Paw ~ {cond:.0f} mbar'
        x_label, y_label = 'PWM%', 'Flow (slm)'
        load_fn = (f'_blower.setFlowBand({band_index}, {cond:.1f}f, '
                   f'{array_name}, {n_pts});')

    lines = []
    lines.append(f'// Band {band_index}: {cond_str}'
                 f'  ({n_pts} pts from {sweep["n_raw"]} raw)')
    lines.append(f'// x = {x_label}, y = {y_label}')
    lines.append(f'static const LookupPoint {array_name}[] = {{')

    for k, idx in enumerate(indices):
        x = sweep['control'][idx]
        y = sweep['result'][idx]
        comma = ',' if k < n_pts - 1 else ' '

        # Region annotation
        result_max = max(abs(r) for r in sweep['result'])
        if abs(y) <= result_max * 0.001:
            note = '  // Dead zone'
        elif k <= 4 and any(abs(sweep['result'][indices[j]]) <= result_max * 0.001
                            for j in range(k)):
            note = '  // Cracking'
        elif idx >= len(sweep['result']) - 3 and k >= n_pts - 2:
            note = '  // Saturated'
        else:
            note = ''

        lines.append(f'    {{{x:7.2f}f, {y:8.2f}f}}{comma}{note}')

    lines.append('};')
    lines.append(f'// {load_fn}')
    lines.append('')

    return '\n'.join(lines), load_fn


# ─────────────────────────────────────────────────────────────
#  Interactive Input
# ─────────────────────────────────────────────────────────────

def read_sweep_interactive(sweep_num):
    """Read one sweep by pasting. Two consecutive blank lines = done."""
    print(f'\n--- Sweep {sweep_num} ---')
    print('Paste CC serial output (header + CSV + summary).')
    print('Press Enter twice on empty lines when done:\n')
    lines = []
    blank_count = 0
    while True:
        try:
            raw = input()
        except EOFError:
            break
        if raw.strip() == '':
            blank_count += 1
            if blank_count >= 2:
                break
        else:
            blank_count = 0
            lines.append(raw)
    return lines


# ─────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────

def main():
    # Parse arguments
    max_points = MAX_POINTS_DEFAULT
    do_normalize = False
    files = []
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] in ('--max-points', '-m') and i + 1 < len(args):
            max_points = int(args[i + 1])
            i += 2
        elif args[i] in ('--normalize', '-n'):
            do_normalize = True
            i += 1
        elif args[i] in ('--help', '-h'):
            print(__doc__)
            return
        else:
            files.append(args[i])
            i += 1

    # ── No files on CLI → try file dialog, fall back to interactive paste ──
    if not files:
        try:
            import tkinter as tk
            from tkinter import filedialog
            root = tk.Tk()
            root.withdraw()
            selected = filedialog.askopenfilenames(
                title='Select CC sweep file(s)',
                filetypes=[('Text files', '*.txt *.csv *.log'), ('All files', '*.*')],
            )
            root.destroy()
            if selected:
                files = list(selected)
        except Exception:
            pass  # No GUI available — fall through to interactive mode

    print('=' * 62)
    print('  CC Sweep  -->  PressureBandedTable  (max %d pts/band)' % max_points)
    print('=' * 62)

    sweeps = []

    if files:
        # ── File mode (each file may contain multiple sweeps) ──
        for f in files:
            try:
                with open(f, 'r') as fh:
                    text = fh.readlines()
                blocks = split_sweeps(text)
                found = 0
                for block in blocks:
                    sw = parse_sweep(block)
                    if sw:
                        sweeps.append(sw)
                        found += 1
                        print(f'  Loaded {f}: {sw["name"] or sw["type"]},'
                              f' {sw["n_raw"]} pts, condition={sw["condition"]:.1f}')
                if found == 0:
                    print(f'  SKIP  {f}: could not parse any sweeps')
                elif found > 1:
                    print(f'  ({found} sweeps found in {f})')
            except FileNotFoundError:
                print(f'  ERROR {f}: file not found')
    else:
        # ── Interactive mode ──
        print('\nYou will paste one CC sweep at a time (max 4).')
        sweep_num = 1
        while sweep_num <= 4:
            text = read_sweep_interactive(sweep_num)
            if not text:
                break
            sw = parse_sweep(text)
            if sw:
                sweeps.append(sw)
                print(f'  OK: {sw["name"] or sw["type"]},'
                      f' {sw["n_raw"]} raw points,'
                      f' condition = {sw["condition"]:.1f}')
                sweep_num += 1
            else:
                print('  Could not parse — check the format and try again.')
                continue

            if sweep_num <= 4:
                ans = input('\nAdd another sweep? (y/N): ').strip().lower()
                if ans != 'y':
                    break

    if not sweeps:
        print('\nNo valid sweeps. Exiting.')
        return

    # Sort by condition (ascending) — required by PressureBandedTable
    sweeps.sort(key=lambda s: s['condition'])

    # ── Pressure droop normalization (insp valves only) ──
    if do_normalize:
        print('\n  Pressure droop normalization enabled (--normalize)')
        for sw in sweeps:
            info = normalize_flows(sw)
            if info is not None:
                n_corr, max_corr = info
                print(f'    {sw["name"] or sw["type"]} (cond={sw["condition"]:.0f}):'
                      f'  {n_corr} points corrected,'
                      f'  max correction = {max_corr:.2f} slm')
            else:
                print(f'    {sw["name"] or sw["type"]} (cond={sw["condition"]:.0f}):'
                      f'  skipped (not insp valve)')

    # ── Reduce & report ──
    print('\n' + '-' * 62)
    print('  Reduction Summary')
    print('-' * 62)

    band_results = []  # (sweep, indices, code, load_fn)

    for band_idx, sw in enumerate(sweeps):
        indices = reduce_points(sw['control'], sw['result'], max_points)
        max_err, err_idx = compute_max_error(sw['control'], sw['result'], indices)
        code, load_fn = generate_cpp(sw, indices, band_idx)
        band_results.append((sw, indices, code, load_fn))

        result_range = max(abs(r) for r in sw['result'])
        err_pct = (max_err / result_range * 100) if result_range > 0 else 0

        print(f'\n  Band {band_idx}:  {sw["n_raw"]} raw  -->  {len(indices)} pts'
              f'   (condition = {sw["condition"]:.1f})')
        print(f'    Max interpolation error: {max_err:.2f}'
              f' ({err_pct:.1f}% of range)'
              f'  at control={sw["control"][err_idx]:.2f}')

        # Print selected points
        print(f'    {"Control":>8}  {"Result":>10}   Region')
        print(f'    {"--------":>8}  {"----------":>10}   ------')
        for k, idx in enumerate(indices):
            x = sw['control'][idx]
            y = sw['result'][idx]
            region = ''
            rmax = max(abs(r) for r in sw['result'])
            if abs(y) <= rmax * 0.001:
                region = 'dead'
            elif k <= 4 and any(abs(sw['result'][indices[j]]) <= rmax * 0.001
                                for j in range(k)):
                region = 'crack'
            elif idx >= len(sw['result']) - 3 and k >= len(indices) - 2:
                region = 'sat'
            else:
                region = 'active'
            print(f'    {x:8.2f}  {y:10.2f}   {region}')

    # ── Output C++ code ──
    print('\n' + '=' * 62)
    print('  C++ CODE — paste into LocalValveController::setDefaults()')
    print('=' * 62)
    print()

    for sw, indices, code, load_fn in band_results:
        print(code)

    print('// ── Load bands (ascending condition order) ──')
    for sw, indices, code, load_fn in band_results:
        print(load_fn)

    print()
    print('// Done. %d band(s), max %d points each.' %
          (len(band_results), max_points))
    print()


if __name__ == '__main__':
    main()
