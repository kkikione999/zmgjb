#!/usr/bin/env python3
"""
auto_tune.py -- Automated PID step-response sweep for bench testing.

The drone sits flat on a table (no propellers).  The script:
  1. Sets a PID gain combination on Roll axis
  2. Injects an RC step (target roll = +5 deg)
  3. Captures motor PWM correction over 3 seconds
  4. Analyses correction magnitude, growth rate, and oscillation
  5. Sweeps multiple gain sets and ranks them

Usage:
    python3 auto_tune.py
    python3 auto_tune.py --port /dev/cu.usbmodem114403
"""

import argparse
import glob
import math
import re
import sys
import time

try:
    import serial
except ImportError:
    print("[ERROR] pip3 install pyserial")
    sys.exit(1)

# --------------- constants -----------------------------------------------
BAUD = 460800
READ_TIMEOUT = 0.1

PWM_RE = re.compile(
    r"PWM:\s*M1=(\d+)\s*M2=(\d+)\s*M3=(\d+)\s*M4=(\d+)"
)
ATT_RE = re.compile(
    r"Euler:\s*([\d.\-]+),\s*([\d.\-]+),\s*([\d.\-]+)\s+"
    r"Gyro:\s*([\d.\-]+),\s*([\d.\-]+),\s*([\d.\-]+)"
)

# Roll axis index for pidr / pida commands
AXIS_ROLL = 0

# Test parameters
THROTTLE_X100 = 55          # 55 % base throttle
STEP_ROLL_X100 = 20         # roll target = 20/100 * 25 deg = 5 deg
STABILISE_SEC = 1.0         # wait before step
CAPTURE_SEC = 3.0           # read motors after step
SETTLE_SEC = 2.0            # read motors after returning to level
SAMPLE_INTERVAL = 0.10      # ~100 ms between samples
RC_KEEPALIVE_SEC = 1.5      # re-send rc at least this often (< 5 s timeout)

# --------------- serial helpers ------------------------------------------

def auto_detect_port():
    patterns = [
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
    ]
    for pat in patterns:
        cands = sorted(glob.glob(pat))
        for c in cands:
            if "/cu." in c:
                return c
        if cands:
            return cands[0]
    return None


def send_cmd(ser, cmd):
    """Send a line command, wait briefly, return response text."""
    ser.reset_input_buffer()
    ser.write((cmd.strip() + "\n").encode("utf-8"))
    ser.flush()
    time.sleep(0.05)
    buf = ser.read(1024).decode("utf-8", errors="replace").strip()
    return buf


def read_motors(ser):
    """Send 'motors', return (M1, M2, M3, M4) or None."""
    resp = send_cmd(ser, "motors")
    m = PWM_RE.search(resp)
    if m:
        return (int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)))
    return None


def read_att(ser):
    """Send 'att', return (roll, pitch, yaw, gx, gy, gz) or None."""
    resp = send_cmd(ser, "att")
    m = ATT_RE.search(resp)
    if m:
        return tuple(float(m.group(i)) for i in range(1, 7))
    return None


# --------------- PID parameter helpers -----------------------------------

def set_rate_pid(ser, axis, kp, ki=0.0, kd=0.0):
    """Set rate-loop PID for one axis.  Values are float, sent as x100 int."""
    cmd = f"pidr {axis} {int(kp*100)} {int(ki*100)} {int(kd*100)}"
    return send_cmd(ser, cmd)


def set_angle_pid(ser, axis, kp, ki=0.0, kd=0.0):
    """Set angle-loop PID for one axis.  Values are float, sent as x100 int."""
    cmd = f"pida {axis} {int(kp*100)} {int(ki*100)} {int(kd*100)}"
    return send_cmd(ser, cmd)


def reset_pid(ser):
    return send_cmd(ser, "pidrst")


def send_rc(ser, t, r, p, y):
    return send_cmd(ser, f"rc {t} {r} {p} {y}")


# --------------- step response test --------------------------------------

def run_step_test(ser, rate_kp, rate_ki, rate_kd,
                  angle_kp, angle_ki, angle_kd):
    """
    Run a single step response test and return structured data.

    Returns dict with keys:
        params       - (rate_kp, rate_ki, rate_kd, angle_kp, angle_ki, angle_kd)
        samples      - list of (t, M1, M2, M3, M4) tuples during step
        settle       - list of (t, M1, M2, M3, M4) tuples after returning to level
        att_samples  - list of (t, roll, pitch, yaw, gx, gy, gz) during step
    """
    # 1. Reset PID state
    reset_pid(ser)

    # 2. Set Roll axis PID
    set_rate_pid(ser, AXIS_ROLL, rate_kp, rate_ki, rate_kd)
    set_angle_pid(ser, AXIS_ROLL, angle_kp, angle_ki, angle_kd)

    # Also set Pitch to safe zero to avoid cross-axis interference
    set_rate_pid(ser, 1, 0, 0, 0)
    set_angle_pid(ser, 1, 0, 0, 0)
    set_rate_pid(ser, 2, 0, 0, 0)
    set_angle_pid(ser, 2, 0, 0, 0)

    time.sleep(0.1)

    # 3. Start with level throttle
    send_rc(ser, THROTTLE_X100, 0, 0, 0)
    time.sleep(STABILISE_SEC)

    # 4. Apply step: roll target = +5 deg (20/100 * 25 = 5 deg)
    send_rc(ser, THROTTLE_X100, STEP_ROLL_X100, 0, 0)
    step_start = time.time()

    # 5. Capture motor data for CAPTURE_SEC
    samples = []
    att_samples = []
    last_rc_time = time.time()

    while time.time() - step_start < CAPTURE_SEC:
        now = time.time()
        elapsed = now - step_start

        # RC keepalive
        if now - last_rc_time >= RC_KEEPALIVE_SEC:
            send_rc(ser, THROTTLE_X100, STEP_ROLL_X100, 0, 0)
            last_rc_time = now

        motors = read_motors(ser)
        if motors:
            samples.append((elapsed, *motors))

        att = read_att(ser)
        if att:
            att_samples.append((elapsed, *att))

        # Sleep to approximate SAMPLE_INTERVAL, minus time already spent
        spent = time.time() - now
        remaining = SAMPLE_INTERVAL - spent
        if remaining > 0:
            time.sleep(remaining)

    # 6. Return to level
    send_rc(ser, THROTTLE_X100, 0, 0, 0)
    settle_start = time.time()

    # 7. Capture settle data
    settle = []
    last_rc_time = time.time()

    while time.time() - settle_start < SETTLE_SEC:
        now = time.time()
        elapsed = now - settle_start

        if now - last_rc_time >= RC_KEEPALIVE_SEC:
            send_rc(ser, THROTTLE_X100, 0, 0, 0)
            last_rc_time = now

        motors = read_motors(ser)
        if motors:
            settle.append((elapsed, *motors))

        spent = time.time() - now
        remaining = SAMPLE_INTERVAL - spent
        if remaining > 0:
            time.sleep(remaining)

    # 8. Stop motors
    send_rc(ser, 0, 0, 0, 0)

    return {
        "params": (rate_kp, rate_ki, rate_kd, angle_kp, angle_ki, angle_kd),
        "samples": samples,
        "settle": settle,
        "att_samples": att_samples,
    }


# --------------- analysis ------------------------------------------------

def analyse_result(result):
    """
    Analyse step response data.

    Since the drone is on a table and cannot tilt:
      - Motor correction = how hard PID tries to reach the angle target
      - P-gain  -> magnitude of correction
      - I-gain  -> growth of correction over time (integral windup)
      - Oscillation -> instability in the PID output

    Key metrics:
      correction_magnitude  - mean absolute motor differential (L - R)
      correction_growth     - slope of motor differential over time
      oscillation_count     - number of zero-crossings of the differential
      steady_correction     - mean correction in last 0.5 s of step
      initial_correction    - mean correction in first 0.3 s of step
    """
    samples = result["samples"]
    if len(samples) < 3:
        return {
            "correction_magnitude": 0.0,
            "correction_growth": 0.0,
            "oscillation_count": 0,
            "steady_correction": 0.0,
            "initial_correction": 0.0,
            "composite_score": -999.0,
            "n_samples": len(samples),
        }

    # Roll correction: left motors (M1+M4) vs right motors (M2+M3)
    # Positive roll target -> left motors should spin faster
    diffs = []
    for (t, m1, m2, m3, m4) in samples:
        left = m1 + m4
        right = m2 + m3
        diffs.append((t, left - right))

    # Mean absolute correction
    abs_diffs = [abs(d) for (_, d) in diffs]
    correction_magnitude = sum(abs_diffs) / len(abs_diffs)

    # Growth rate: linear regression of diff vs time
    n = len(diffs)
    ts = [d[0] for d in diffs]
    ds = [d[1] for d in diffs]
    sum_t = sum(ts)
    sum_d = sum(ds)
    sum_td = sum(t * d for t, d in diffs)
    sum_tt = sum(t * t for t in ts)
    denom = n * sum_tt - sum_t * sum_t
    if abs(denom) > 1e-9:
        correction_growth = (n * sum_td - sum_t * sum_d) / denom
    else:
        correction_growth = 0.0

    # Oscillation: count sign changes in the differential
    oscillation_count = 0
    for i in range(1, len(ds)):
        if ds[i] * ds[i - 1] < 0:
            oscillation_count += 1

    # Initial correction (first 0.3 s)
    initial_diffs = [abs(d) for (t, d) in diffs if t <= 0.3]
    initial_correction = sum(initial_diffs) / len(initial_diffs) if initial_diffs else 0.0

    # Steady correction (last 0.5 s)
    max_t = max(ts)
    steady_diffs = [abs(d) for (t, d) in diffs if t >= max_t - 0.5]
    steady_correction = sum(steady_diffs) / len(steady_diffs) if steady_diffs else 0.0

    # Composite score (higher = better):
    #   - Want moderate correction_magnitude (not too low, not saturating)
    #   - Want controlled growth (positive but not extreme)
    #   - Want low oscillation
    #   - Penalise saturation (correction hitting PWM limits)

    # Ideal correction magnitude: enough to be meaningful but not saturating
    # With 55% throttle (~549 PWM) and max 999, the correction headroom is ~450
    # Ideal correction is maybe 100-300 PWM differential
    mag_score = min(correction_magnitude / 200.0, 1.5)  # 1.0 at 200, caps at 1.5

    # Growth: moderate positive growth is good (shows I-term works)
    # But too much means windup problems
    growth_score = 0.0
    if correction_growth > 0:
        growth_score = min(correction_growth / 50.0, 1.0)  # 1.0 at 50 PWM/s
    else:
        growth_score = 0.2  # slight penalty for no growth

    # Oscillation penalty
    osc_penalty = min(oscillation_count / 10.0, 1.0)  # 0 is best

    # Saturation check: if steady_correction > 400, we are near limits
    sat_penalty = max(0, (steady_correction - 400) / 200.0)

    composite_score = (mag_score * 0.4
                       + growth_score * 0.2
                       - osc_penalty * 0.3
                       - sat_penalty * 0.1)

    return {
        "correction_magnitude": correction_magnitude,
        "correction_growth": correction_growth,
        "oscillation_count": oscillation_count,
        "steady_correction": steady_correction,
        "initial_correction": initial_correction,
        "composite_score": composite_score,
        "n_samples": len(samples),
    }


# --------------- sweep definitions ---------------------------------------

def build_sweep_plan():
    """Build the full parameter sweep plan."""
    tests = []

    # Sweep 1: Rate Kp only (angle Kp = 0)
    for rkp in [0.10, 0.20, 0.30, 0.50, 0.80]:
        tests.append({
            "label": f"S1 rate_kp={rkp:.2f}",
            "sweep": 1,
            "rate_kp": rkp, "rate_ki": 0, "rate_kd": 0,
            "angle_kp": 0, "angle_ki": 0, "angle_kd": 0,
        })

    # Sweep 2: Angle Kp only (rate Kp = 0.20)
    for akp in [1.0, 2.0, 3.0, 4.0, 6.0]:
        tests.append({
            "label": f"S2 angle_kp={akp:.1f}",
            "sweep": 2,
            "rate_kp": 0.20, "rate_ki": 0, "rate_kd": 0,
            "angle_kp": akp, "angle_ki": 0, "angle_kd": 0,
        })

    # Sweep 3: Combined (best candidates)
    for akp in [2.0, 3.0, 4.0]:
        for rkp in [0.15, 0.25, 0.40]:
            tests.append({
                "label": f"S3 rkp={rkp:.2f} akp={akp:.1f}",
                "sweep": 3,
                "rate_kp": rkp, "rate_ki": 0, "rate_kd": 0,
                "angle_kp": akp, "angle_ki": 0, "angle_kd": 0,
            })

    return tests


# --------------- printing ------------------------------------------------

def print_test_result(idx, total, test, result, metrics):
    """Print a concise summary of one test."""
    params = result["params"]
    label = test["label"]
    rkp, rki, rkd, akp, aki, akd = params

    print(f"\n--- Test {idx}/{total}: {label} ---")
    print(f"  Rate  Kp={rkp:.2f} Ki={rki:.2f} Kd={rkd:.2f}  "
          f"Angle Kp={akp:.1f} Ki={aki:.2f} Kd={akd:.2f}")

    # Print motor samples (every 3rd sample to keep concise)
    samples = result["samples"]
    if samples:
        print(f"  Samples ({len(samples)} total, showing every 3rd):")
        for i, (t, m1, m2, m3, m4) in enumerate(samples):
            if i % 3 == 0:
                left = m1 + m4
                right = m2 + m3
                diff = left - right
                print(f"    t={t:5.2f}s  M1={m1:3d} M2={m2:3d} M3={m3:3d} M4={m4:3d}  "
                      f"L-R={diff:+4d}")

    # Print attitude samples (every 5th)
    att_samples = result["att_samples"]
    if att_samples:
        print(f"  Attitude ({len(att_samples)} total, showing every 5th):")
        for i, (t, roll, pitch, yaw, gx, gy, gz) in enumerate(att_samples):
            if i % 5 == 0:
                print(f"    t={t:5.2f}s  Roll={roll:+6.2f} Pitch={pitch:+6.2f} Yaw={yaw:+6.2f}  "
                      f"Gyro=({gx:+.1f},{gy:+.1f},{gz:+.1f})")

    # Metrics
    print(f"  Metrics:")
    print(f"    correction_magnitude = {metrics['correction_magnitude']:.1f} PWM")
    print(f"    initial_correction   = {metrics['initial_correction']:.1f} PWM")
    print(f"    steady_correction    = {metrics['steady_correction']:.1f} PWM")
    print(f"    correction_growth    = {metrics['correction_growth']:.1f} PWM/s")
    print(f"    oscillation_count    = {metrics['oscillation_count']}")
    print(f"    composite_score      = {metrics['composite_score']:.3f}")


def print_summary_table(all_results):
    """Print a final summary table ranking all tests by composite score."""
    print("\n")
    print("=" * 100)
    print("  SUMMARY TABLE (ranked by composite score)")
    print("=" * 100)

    # Sort by composite_score descending
    ranked = sorted(all_results, key=lambda x: x["metrics"]["composite_score"], reverse=True)

    header = (f"  {'#':>3}  {'Label':<28}  "
              f"{'Mag':>6}  {'Init':>6}  {'Steady':>6}  "
              f"{'Growth':>7}  {'Osc':>4}  {'Score':>7}")
    print(header)
    print("  " + "-" * 96)

    for i, entry in enumerate(ranked):
        label = entry["test"]["label"]
        m = entry["metrics"]
        rank = f"{i+1:>3}"
        print(f"  {rank}  {label:<28}  "
              f"{m['correction_magnitude']:>6.1f}  "
              f"{m['initial_correction']:>6.1f}  "
              f"{m['steady_correction']:>6.1f}  "
              f"{m['correction_growth']:>+7.1f}  "
              f"{m['oscillation_count']:>4}  "
              f"{m['composite_score']:>7.3f}")

    print("  " + "-" * 96)

    if ranked:
        best = ranked[0]
        label = best["test"]["label"]
        m = best["metrics"]
        print(f"\n  BEST: {label}")
        print(f"        Rate Kp={best['test']['rate_kp']:.2f}  "
              f"Angle Kp={best['test']['angle_kp']:.1f}")
        print(f"        correction={m['correction_magnitude']:.1f} PWM  "
              f"growth={m['correction_growth']:+.1f} PWM/s  "
              f"oscillations={m['oscillation_count']}")

        # Sweep-specific bests
        for sweep_num in [1, 2, 3]:
            sweep_results = [e for e in ranked if e["test"]["sweep"] == sweep_num]
            if sweep_results:
                sw_best = sweep_results[0]
                print(f"        Sweep {sweep_num} best: {sw_best['test']['label']}  "
                      f"score={sw_best['metrics']['composite_score']:.3f}")

    print()


# --------------- main ----------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Automated PID step-response sweep for drone bench testing")
    parser.add_argument(
        "--port", default=None,
        help="Serial port (default: auto-detect /dev/cu.usbmodem*)")
    args = parser.parse_args()

    port = args.port
    if not port:
        port = auto_detect_port()
    if not port:
        print("[ERROR] No serial port found. Use --port to specify.")
        sys.exit(1)

    print(f"Connecting to {port} @ {BAUD}")
    ser = serial.Serial(port, BAUD, timeout=READ_TIMEOUT)

    try:
        # Flush startup output
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.3)
        ser.read(4096)

        # Initial stop
        send_rc(ser, 0, 0, 0, 0)
        time.sleep(0.2)

        # Build sweep plan
        tests = build_sweep_plan()
        total = len(tests)
        print(f"\nSweep plan: {total} tests")
        print(f"  Sweep 1: Rate Kp only (5 tests)")
        print(f"  Sweep 2: Angle Kp only (5 tests)")
        print(f"  Sweep 3: Combined (9 tests)")
        print()

        all_results = []

        for idx, test in enumerate(tests, 1):
            print(f"[{idx}/{total}] Running: {test['label']} ...")

            result = run_step_test(
                ser,
                test["rate_kp"], test["rate_ki"], test["rate_kd"],
                test["angle_kp"], test["angle_ki"], test["angle_kd"],
            )

            metrics = analyse_result(result)
            result_entry = {"test": test, "result": result, "metrics": metrics}
            all_results.append(result_entry)

            print_test_result(idx, total, test, result, metrics)

            # Brief pause between tests
            time.sleep(0.5)

        # Final summary
        print_summary_table(all_results)

    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping motors ...")
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        # Safety: always stop motors
        try:
            send_rc(ser, 0, 0, 0, 0)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print("Motors stopped. Serial closed.")


if __name__ == "__main__":
    main()
