#!/usr/bin/env python3
"""
roll_auto_tune.py -- Roll 单轴 PID 自主调参脚本
全自动完成 Roll 轴 Rate 内环 (P→D→I) + Angle 外环 (P) 调参。

用法:
    python3 scripts/roll_auto_tune.py
    python3 scripts/roll_auto_tune.py --port /dev/cu.usbmodem1234
    python3 scripts/roll_auto_tune.py --phase A    # 只跑阶段 A
    python3 scripts/roll_auto_tune.py --phase ABCD  # 指定阶段
"""

import argparse
import glob
import re
import signal
import sys
import time

try:
    import serial
except ImportError:
    print("[ERROR] pip3 install pyserial")
    sys.exit(1)

# ── 常量 ──────────────────────────────────────────────────────
BAUD = 460800
RATE_THROTTLE = 30     # Rate 环测试油门 (有角度环保护，可以提高到 30)
ANGLE_THROTTLE = 30    # Angle 环测试油门 (低于悬停)
ROLL_FUSE_ANGLE = 45   # Roll 超过此角度立即熔断停机 (有角度环保护，放宽到 45)
HOLD_ANGLE_KP = 2.0    # Rate 扫描时固定使用的角度环 Kp
PWM_DANGER = 950       # 超过此值立即停机
COOLDOWN = 3           # 测试间最小冷却秒数
RC_REFRESH = 4         # RC 刷新间隔秒数
INIT_SETTLE = 0.05     # 参数初始化后等待 (超过一个外环周期 20ms)
RAMP_STEP = 5          # 油门渐升步长 (%) (有角度环保护，可以正常渐升)
RAMP_INTERVAL = 0.3    # 油门渐升间隔 (s)

KP_SCAN = [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.80]
KD_SCAN = [0.01, 0.02, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30]
KI_SCAN = [0.01, 0.03, 0.05, 0.08]
KP_ANGLE_SCAN = [2.0, 3.0, 4.0, 5.0, 6.0]

KP_FACTOR = 0.65       # 振荡阈值 × 此值 = 工作 Kp

NUM_SAMPLES_SHORT = 20 # 4s @ 0.2s
NUM_SAMPLES_LONG = 40  # 8s @ 0.2s
SAMPLE_INTERVAL = 0.2

PWM_RE = re.compile(r"PWM:\s*M1=(\d+)\s*M2=(\d+)\s*M3=(\d+)\s*M4=(\d+)")
ATT_RE = re.compile(
    r"Euler:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s+"
    r"Gyro:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)"
)

GREEN  = "\033[92m"; RED    = "\033[91m"; YELLOW = "\033[93m"
CYAN   = "\033[96m"; BOLD   = "\033[1m"; RESET  = "\033[0m"


# ── 串口通信 ──────────────────────────────────────────────────
class DroneSerial:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self._current_throttle = 0

    def send(self, cmd):
        self.ser.reset_input_buffer()
        self.ser.write((cmd.strip() + "\n").encode())
        self.ser.flush()
        time.sleep(0.05)
        return self.ser.read(1024).decode("utf-8", errors="replace").strip()

    def motors(self):
        resp = self.send("motors")
        m = PWM_RE.search(resp)
        if m:
            return tuple(int(m.group(i)) for i in range(1, 5))
        return None

    def att(self):
        resp = self.send("att")
        m = ATT_RE.search(resp)
        if m:
            return {
                "roll": float(m.group(1)), "pitch": float(m.group(2)),
                "yaw": float(m.group(3)), "gx": float(m.group(4)),
                "gy": float(m.group(5)), "gz": float(m.group(6)),
            }
        return None

    def rc(self, t, r, p, y):
        self._current_throttle = t
        self.send(f"rc {t} {r} {p} {y}")

    def ramp_throttle(self, target, roll=0):
        """油门从当前值渐升到 target。返回 True=正常, False=熔断"""
        current = self._current_throttle
        if current < target:
            steps = list(range(current + RAMP_STEP, target + 1, RAMP_STEP))
            if not steps or steps[-1] != target:
                steps.append(target)
        elif current > target:
            steps = list(range(current - RAMP_STEP, target - 1, -RAMP_STEP))
            if not steps or steps[-1] != target:
                steps.append(target)
        else:
            return True
        for t in steps:
            self.rc(t, roll, 0, 0)
            time.sleep(RAMP_INTERVAL)
            # 每步检查角度
            att_check = self.att()
            if att_check and abs(att_check["roll"]) > ROLL_FUSE_ANGLE:
                print(f"  {RED}[FUSE] Ramp中 Roll={att_check['roll']:+.1f}° 超限!{RESET}")
                self.stop()
                return False  # 熔断
        return True  # 正常完成

    def init_roll_only(self, kp_rate=0, ki_rate=0, kd_rate=0, kp_angle=0, hold_angle=False):
        """初始化为 Roll-only PID 模式。
        hold_angle=True 时忽略 kp_angle，使用 HOLD_ANGLE_KP 保持水平。
        """
        self.send("pidrst")
        for axis in [0, 1, 2]:
            self.send(f"pida {axis} 0 0 0")
        self.send("pidr 1 0 0 0")
        self.send("pidr 2 0 0 0")
        kp_x100 = int(kp_rate * 100)
        ki_x100 = int(ki_rate * 100)
        kd_x100 = int(kd_rate * 100)
        self.send(f"pidr 0 {kp_x100} {ki_x100} {kd_x100}")
        effective_kpa = HOLD_ANGLE_KP if hold_angle else kp_angle
        if effective_kpa > 0:
            kpa_x100 = int(effective_kpa * 100)
            self.send(f"pida 0 {kpa_x100} 0 0")
        time.sleep(INIT_SETTLE)

    def stop(self):
        self.rc(0, 0, 0, 0)

    def close(self):
        try:
            self.stop()
            self.ser.close()
        except Exception:
            pass

    def check_pwm_danger(self, vals):
        """检查 PWM 是否超限，超限返回 True"""
        if vals and any(v > PWM_DANGER for v in vals):
            print(f"\n{RED}[DANGER] PWM > {PWM_DANGER}! 紧急停机!{RESET}")
            self.stop()
            return True
        return False


def auto_detect_port():
    candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not candidates:
        return None
    for port in candidates:
        try:
            ser = serial.Serial(port, BAUD, timeout=0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            ser.write(b"motors\n")
            ser.flush()
            time.sleep(0.05)
            resp = ser.read(1024).decode("utf-8", errors="replace")
            ser.close()
            if "PWM:" in resp:
                print(f"{GREEN}[OK]{RESET} 串口: {port}")
                return port
        except Exception:
            pass
    return None


# ── 采样辅助 ──────────────────────────────────────────────────
def sample_cycle(drone, num_samples, throttle, roll_target=0):
    """
    执行一轮采样: ramp 油门 → 稳定 → 采样 → 停机。
    返回 (samples 列表, fused_bool)。
    fused=True 表示角度熔断停机。
    """
    # 渐升油门
    fused = not drone.ramp_throttle(throttle, roll_target)
    if fused:
        return [], True  # 渐升阶段熔断

    # 渐升完成后读一次 att 检查
    att_check = drone.att()
    if att_check and abs(att_check["roll"]) > ROLL_FUSE_ANGLE:
        print(f"\n  {RED}[FUSE] Roll={att_check['roll']:+.1f}° 超限! 立即停机!{RESET}")
        drone.stop()
        return [], True

    # 稳定等待
    time.sleep(1.5)

    # 稳定期结束后再检查一次角度
    att_check = drone.att()
    if att_check and abs(att_check["roll"]) > ROLL_FUSE_ANGLE:
        print(f"\n  {RED}[FUSE] 稳定期 Roll={att_check['roll']:+.1f}° 超限! 立即停机!{RESET}")
        drone.stop()
        return [], True

    samples = []
    last_rc_refresh = time.time()

    for i in range(num_samples):
        # RC 刷新防超时
        now = time.time()
        if now - last_rc_refresh > RC_REFRESH:
            drone.rc(throttle, roll_target, 0, 0)
            last_rc_refresh = now

        vals = drone.motors()
        att_data = drone.att()

        # PWM 安全检查
        if drone.check_pwm_danger(vals):
            return samples, True  # danger

        # 角度熔断检查
        if att_data and abs(att_data["roll"]) > ROLL_FUSE_ANGLE:
            print(f"\n  {RED}[FUSE] Roll={att_data['roll']:+.1f}° 超限! 立即停机!{RESET}")
            drone.stop()
            return samples, True  # fused

        if vals and att_data:
            m1, m2, m3, m4 = vals
            diff = (m1 + m4) - (m2 + m3)
            samples.append({
                "roll": att_data["roll"],
                "diff": diff,
                "gx": att_data["gx"],
                "m1": m1, "m2": m2, "m3": m3, "m4": m4,
            })
            print(f"  #{i+1:2d}  Roll={att_data['roll']:+7.2f}°  diff={diff:+5d}  "
                  f"M1={m1} M2={m2} M3={m3} M4={m4}")
        else:
            print(f"  #{i+1:2d}  [读取失败]")

        time.sleep(SAMPLE_INTERVAL)

    drone.stop()
    return samples, False


def count_zero_crossings(rolls):
    """计算 Roll 角度过零点次数"""
    crossings = 0
    for i in range(1, len(rolls)):
        if rolls[i-1] * rolls[i] < 0:
            crossings += 1
    return crossings


def analyze_samples(samples):
    """分析采样数据，返回结果字典"""
    if not samples:
        return None

    rolls = [s["roll"] for s in samples]
    diffs = [s["diff"] for s in samples]
    abs_diffs = [abs(d) for d in diffs]

    return {
        "roll_avg": sum(rolls) / len(rolls),
        "roll_min": min(rolls),
        "roll_max": max(rolls),
        "roll_range": max(rolls) - min(rolls),
        "roll_zero_crossings": count_zero_crossings(rolls),
        "diff_avg": sum(diffs) / len(diffs),
        "diff_max_abs": max(abs_diffs),
        "num_samples": len(samples),
    }


def print_result(label, result):
    """打印单次扫描结果"""
    if not result:
        print(f"  {RED}无有效数据{RESET}")
        return
    print(f"  {CYAN}{label}{RESET}")
    print(f"    Roll: avg={result['roll_avg']:+.2f}°  "
          f"range=[{result['roll_min']:+.1f}°, {result['roll_max']:+.1f}°]  "
          f"过零={result['roll_zero_crossings']}")
    print(f"    Diff: avg={result['diff_avg']:+.0f}  max|diff|={result['diff_max_abs']}")


def print_header(title):
    print()
    print(f"{BOLD}{CYAN}{'=' * 60}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'=' * 60}{RESET}")


def wait_reset():
    """等待无人机自动复位"""
    time.sleep(2)


# ── 阶段 A: Rate Kp 精细扫描 ──────────────────────────────────
def phase_a(drone):
    print_header("阶段 A: Rate Kp 精细扫描")
    print(f"  扫描值: {KP_SCAN}")
    print(f"  油门: {RATE_THROTTLE}%")
    print(f"  目标: 定位振荡阈值, 工作 Kp = 阈值 × {KP_FACTOR}")

    results = []
    oscillation_kp = None
    for idx, kp in enumerate(KP_SCAN):
        print(f"\n{BOLD}--- [{idx+1}/{len(KP_SCAN)}] Kp_rate = {kp:.2f} ---{RESET}")

        # 初始化 PID (启用角度环保持水平)
        drone.init_roll_only(kp_rate=kp, hold_angle=True)

        # 采样
        samples, danger = sample_cycle(drone, NUM_SAMPLES_SHORT, RATE_THROTTLE)

        if danger:
            if not samples:
                print(f"  {RED}熔断停机! Kp={kp:.2f} 已导致翻转，跳过后续更高值{RESET}")
                oscillation_kp = kp
            else:
                print(f"  {RED}PWM 超限! 测试终止{RESET}")
            break

        result = analyze_samples(samples)
        print_result(f"Kp={kp:.2f}", result)
        results.append({"kp": kp, **result} if result else {"kp": kp})

        # 振荡检测
        if result and result["roll_zero_crossings"] > 4:
            oscillation_kp = kp
            print(f"  {RED}{BOLD}>>> 振荡检测! Roll 过零 {result['roll_zero_crossings']} 次{RESET}")
            break

        if result and result["diff_max_abs"] > 100 and result["roll_range"] > 20:
            print(f"  {YELLOW}>>> 大幅差速 + 角度范围大, 接近振荡{RESET}")

        wait_reset()

    # 汇总
    print_header("阶段 A 汇总")
    print(f"\n  {'Kp':>6s}  {'avg_diff':>8s}  {'max|d|':>7s}  "
          f"{'Roll范围':>10s}  {'过零':>4s}  {'评级':>6s}")
    print(f"  {'-'*6}  {'-'*8}  {'-'*7}  {'-'*10}  {'-'*4}  {'-'*6}")

    for r in results:
        if "diff_avg" not in r:
            print(f"  {r['kp']:6.2f}  {'无数据':>8s}")
            continue
        rating = f"{RED}振荡{RESET}" if r["roll_zero_crossings"] > 4 else (
            f"{GREEN}正常{RESET}" if r["diff_max_abs"] > 30 else f"{YELLOW}弱{RESET}")
        print(f"  {r['kp']:6.2f}  {r['diff_avg']:+8.0f}  {r['diff_max_abs']:7.0f}  "
              f"[{r['roll_min']:+.1f},{r['roll_max']:+.1f}]  {r['roll_zero_crossings']:4d}  {rating}")

    # 确定 Kp 工作值
    kp_work = None
    if oscillation_kp:
        kp_work = round(oscillation_kp * KP_FACTOR, 2)
        print(f"\n  {GREEN}{BOLD}振荡阈值 Kp = {oscillation_kp:.2f}{RESET}")
        print(f"  {GREEN}{BOLD}工作 Kp = {kp_work:.2f} ({oscillation_kp:.2f} × {KP_FACTOR}){RESET}")
    elif results:
        # 没检测到振荡，取最大的稳定 Kp
        max_kp = results[-1]["kp"]
        kp_work = max_kp
        print(f"\n  {GREEN}全部 Kp 均稳定，未检测到振荡{RESET}")
        print(f"  {GREEN}{BOLD}工作 Kp = {kp_work:.2f} (最大稳定值){RESET}")
    else:
        # 全部熔断，连最小的 Kp 都翻转
        print(f"\n  {RED}{BOLD}警告: 全部 Kp 值均导致熔断翻转!{RESET}")
        print(f"  {RED}可能原因: 油门过高、测试架问题、或 PID 方向错误{RESET}")
        print(f"  {RED}建议: 降低 RATE_THROTTLE 或检查硬件{RESET}")
        kp_work = KP_SCAN[0]  # 最低值作为兜底

    return kp_work, oscillation_kp


# ── 阶段 B: Rate Kd 扫描 ─────────────────────────────────────
def phase_b(drone, kp_work):
    print_header("阶段 B: Rate Kd 扫描")
    print(f"  固定 Kp_rate = {kp_work:.2f}")
    print(f"  扫描值: {KD_SCAN}")
    print(f"  油门: {RATE_THROTTLE}%")

    results = []

    for idx, kd in enumerate(KD_SCAN):
        print(f"\n{BOLD}--- [{idx+1}/{len(KD_SCAN)}] Kd_rate = {kd:.2f} ---{RESET}")

        drone.init_roll_only(kp_rate=kp_work, kd_rate=kd, hold_angle=True)
        samples, danger = sample_cycle(drone, NUM_SAMPLES_SHORT, RATE_THROTTLE)

        if danger:
            break

        result = analyze_samples(samples)
        print_result(f"Kd={kd:.2f}", result)
        results.append({"kd": kd, **result} if result else {"kd": kd})

        wait_reset()

    # 汇总
    print_header("阶段 B 汇总")
    print(f"\n  {'Kd':>6s}  {'过零':>4s}  {'max|d|':>7s}  {'Roll范围':>10s}  {'阻尼评级':>8s}")
    print(f"  {'-'*6}  {'-'*4}  {'-'*7}  {'-'*10}  {'-'*8}")

    best_kd = None
    best_score = -999

    for r in results:
        if "roll_zero_crossings" not in r:
            print(f"  {r['kd']:6.2f}  {'无数据':>4s}")
            continue

        zc = r["roll_zero_crossings"]
        if zc <= 2:
            rating = f"{GREEN}优秀{RESET}"
            score = 10 - zc
        elif zc <= 4:
            rating = f"{YELLOW}一般{RESET}"
            score = 5 - zc
        else:
            rating = f"{RED}振荡{RESET}"
            score = -zc

        print(f"  {r['kd']:6.2f}  {zc:4d}  {r['diff_max_abs']:7.0f}  "
              f"[{r['roll_min']:+.1f},{r['roll_max']:+.1f}]  {rating}")

        if score > best_score:
            best_score = score
            best_kd = r["kd"]

    if best_kd is not None:
        print(f"\n  {GREEN}{BOLD}最佳 Kd = {best_kd:.2f} (阻尼最优){RESET}")

    # 微调 Kp: 尝试 Kp+0.02, Kp+0.05
    print(f"\n{BOLD}--- 微调 Kp (Kd 固定 = {best_kd:.2f}) ---{RESET}")
    kp_candidates = [kp_work, round(kp_work + 0.02, 2), round(kp_work + 0.05, 2)]
    kp_candidates = [k for k in kp_candidates if k > 0]
    kp_final = kp_work
    best_zc = 999

    for kp_try in kp_candidates:
        print(f"\n  尝试 Kp={kp_try:.2f} Kd={best_kd:.2f}")
        drone.init_roll_only(kp_rate=kp_try, kd_rate=best_kd, hold_angle=True)
        samples, danger = sample_cycle(drone, NUM_SAMPLES_SHORT, RATE_THROTTLE)
        if danger:
            break
        result = analyze_samples(samples)
        print_result(f"Kp={kp_try:.2f}", result)
        if result and result["roll_zero_crossings"] < best_zc and result["roll_zero_crossings"] <= 3:
            best_zc = result["roll_zero_crossings"]
            kp_final = kp_try
            print(f"    {GREEN}→ 新最佳 Kp{RESET}")
        wait_reset()

    print(f"\n  {GREEN}{BOLD}最终 Rate 参数: Kp={kp_final:.2f} Kd={best_kd:.2f}{RESET}")
    return kp_final, best_kd


# ── 阶段 C: Rate Ki 扫描 ─────────────────────────────────────
def phase_c(drone, kp_rate, kd_rate):
    print_header("阶段 C: Rate Ki 扫描")
    print(f"  固定 Kp_rate={kp_rate:.2f} Kd_rate={kd_rate:.2f}")
    print(f"  扫描值: {KI_SCAN}")
    print(f"  油门: {RATE_THROTTLE}%")
    print(f"  目标: 消除 CG 偏置 (+5.8°), 观察收敛")

    results = []

    for idx, ki in enumerate(KI_SCAN):
        print(f"\n{BOLD}--- [{idx+1}/{len(KI_SCAN)}] Ki_rate = {ki:.2f} ---{RESET}")

        drone.init_roll_only(kp_rate=kp_rate, ki_rate=ki, kd_rate=kd_rate, hold_angle=True)
        samples, danger = sample_cycle(drone, NUM_SAMPLES_LONG, RATE_THROTTLE)

        if danger:
            break

        result = analyze_samples(samples)

        if result:
            # 计算收敛: 前 1/4 vs 后 1/4 的 Roll 平均值
            n = len(samples)
            q1_rolls = [s["roll"] for s in samples[:n//4]]
            q4_rolls = [s["roll"] for s in samples[3*n//4:]]
            q1_avg = sum(q1_rolls) / len(q1_rolls) if q1_rolls else 0
            q4_avg = sum(q4_rolls) / len(q4_rolls) if q4_rolls else 0
            convergence = abs(q1_avg) - abs(q4_avg)  # 正值=在收敛
            result["q1_avg"] = q1_avg
            result["q4_avg"] = q4_avg
            result["convergence"] = convergence

        print_result(f"Ki={ki:.2f}", result)
        if result:
            print(f"    前1/4 avg={result['q1_avg']:+.2f}°  "
                  f"后1/4 avg={result['q4_avg']:+.2f}°  "
                  f"收敛={result['convergence']:+.2f}°")
        results.append({"ki": ki, **result} if result else {"ki": ki})

        wait_reset()

    # 汇总
    print_header("阶段 C 汇总")
    print(f"\n  {'Ki':>6s}  {'前1/4':>7s}  {'后1/4':>7s}  {'收敛':>6s}  {'过零':>4s}  {'评级':>6s}")
    print(f"  {'-'*6}  {'-'*7}  {'-'*7}  {'-'*6}  {'-'*4}  {'-'*6}")

    best_ki = 0
    best_convergence = -999

    for r in results:
        if "convergence" not in r:
            print(f"  {r['ki']:6.2f}  {'无数据':>7s}")
            continue

        conv = r["convergence"]
        zc = r["roll_zero_crossings"]
        final_err = abs(r["q4_avg"])

        if final_err < 2 and conv > 0:
            rating = f"{GREEN}优秀{RESET}"
        elif conv > 0:
            rating = f"{YELLOW}收敛中{RESET}"
        else:
            rating = f"{RED}发散{RESET}"

        print(f"  {r['ki']:6.2f}  {r['q1_avg']:+7.2f}  {r['q4_avg']:+7.2f}  "
              f"{conv:+6.2f}  {zc:4d}  {rating}")

        # 选收敛好且不过冲的
        if conv > best_convergence and final_err < 5 and zc <= 3:
            best_convergence = conv
            best_ki = r["ki"]

    # Ki 上限检查
    ki_max = kp_rate / 5
    if best_ki > ki_max:
        print(f"\n  {YELLOW}Ki={best_ki:.2f} 超过 Kp/5={ki_max:.2f}, 限制为 {ki_max:.2f}{RESET}")
        best_ki = round(ki_max, 2)

    print(f"\n  {GREEN}{BOLD}最佳 Ki = {best_ki:.2f}{RESET}")
    return best_ki


# ── 阶段 D: Angle Kp 扫描 ────────────────────────────────────
def phase_d(drone, kp_rate, ki_rate, kd_rate):
    print_header("阶段 D: Angle Kp 扫描")
    print(f"  内环: Kp={kp_rate:.2f} Ki={ki_rate:.2f} Kd={kd_rate:.2f}")
    print(f"  扫描值: {KP_ANGLE_SCAN}")
    print(f"  油门: {ANGLE_THROTTLE}% (匹配悬停)")

    results = []

    for idx, kpa in enumerate(KP_ANGLE_SCAN):
        print(f"\n{BOLD}--- [{idx+1}/{len(KP_ANGLE_SCAN)}] Kp_angle = {kpa:.2f} ---{RESET}")

        # 初始化: 内环 + 角度环
        drone.init_roll_only(kp_rate=kp_rate, ki_rate=ki_rate, kd_rate=kd_rate, kp_angle=kpa)

        # 测试 1: 回中测试
        print(f"  {CYAN}测试 1: 回中 (rc {ANGLE_THROTTLE} 0 0 0){RESET}")
        samples, danger = sample_cycle(drone, NUM_SAMPLES_LONG, ANGLE_THROTTLE)
        if danger:
            break

        result_hold = analyze_samples(samples)
        print_result("回中", result_hold)

        if result_hold and result_hold["roll_zero_crossings"] > 4:
            print(f"  {RED}>>> 内外环耦合振荡! Kp_angle 过高{RESET}")
            results.append({"kpa": kpa, "hold": result_hold, "oscillating": True})
            wait_reset()
            # 振荡了, 不再尝试更高的 Kp
            remaining = KP_ANGLE_SCAN[idx+1:]
            if remaining:
                print(f"  {YELLOW}跳过后续更高值: {remaining}{RESET}")
            break

        # 测试 2: RC 阶跃
        print(f"\n  {CYAN}测试 2: 阶跃 (rc {ANGLE_THROTTLE} 20 0 0 → 5° 目标){RESET}")
        drone.init_roll_only(kp_rate=kp_rate, ki_rate=ki_rate, kd_rate=kd_rate, kp_angle=kpa)
        ramp_ok = drone.ramp_throttle(ANGLE_THROTTLE, roll_target=20)
        if not ramp_ok:
            print(f"  {RED}阶跃渐升阶段熔断，跳过此 Kp_angle{RESET}")
            results.append({"kpa": kpa, "hold": result_hold, "step": None, "oscillating": False})
            wait_reset()
            break
        time.sleep(1.0)

        step_fused = False
        step_samples = []
        for _ in range(25):
            vals = drone.motors()
            att_data = drone.att()
            if drone.check_pwm_danger(vals):
                break
            # 角度熔断检查
            if att_data and abs(att_data["roll"]) > ROLL_FUSE_ANGLE:
                print(f"\n  {RED}[FUSE] 阶跃 Roll={att_data['roll']:+.1f}° 超限! 立即停机!{RESET}")
                drone.stop()
                step_fused = True
                break
            if vals and att_data:
                step_samples.append({"roll": att_data["roll"], "diff": (vals[0]+vals[3])-(vals[1]+vals[2])})
            time.sleep(SAMPLE_INTERVAL)

        if not step_fused:
            drone.stop()
        else:
            results.append({"kpa": kpa, "hold": result_hold, "step": None, "oscillating": False})
            wait_reset()
            break
        result_step = analyze_samples(step_samples)
        if result_step:
            print(f"    阶跃: avg_roll={result_step['roll_avg']:+.2f}°  "
                  f"range=[{result_step['roll_min']:+.1f}, {result_step['roll_max']:+.1f}]  "
                  f"过零={result_step['roll_zero_crossings']}")

        results.append({"kpa": kpa, "hold": result_hold, "step": result_step, "oscillating": False})
        wait_reset()

    # 汇总
    print_header("阶段 D 汇总")
    print(f"\n  {'Kp_angle':>9s}  {'保持过零':>8s}  {'保持误差':>8s}  {'阶跃avg':>7s}  {'评级':>6s}")
    print(f"  {'-'*9}  {'-'*8}  {'-'*8}  {'-'*7}  {'-'*6}")

    best_kpa = 3.0  # 默认
    best_hold_err = 999

    for r in results:
        kpa = r["kpa"]
        if r.get("oscillating"):
            print(f"  {kpa:9.2f}  {'振荡!':>8s}  {'---':>8s}  {'---':>7s}  {RED}振荡{RESET}")
            continue
        hold = r.get("hold", {})
        step = r.get("step", {})
        hold_zc = hold.get("roll_zero_crossings", "?")
        hold_err = abs(hold.get("roll_avg", 99))
        step_avg = step.get("roll_avg", 0)

        if hold_err < 2 and hold_zc <= 2:
            rating = f"{GREEN}优秀{RESET}"
        elif hold_err < 4:
            rating = f"{YELLOW}可用{RESET}"
        else:
            rating = f"{RED}差{RESET}"

        print(f"  {kpa:9.2f}  {str(hold_zc):>8s}  {hold_err:8.2f}  {step_avg:+7.2f}  {rating}")

        if hold_err < best_hold_err and hold_zc <= 3:
            best_hold_err = hold_err
            best_kpa = kpa

    print(f"\n  {GREEN}{BOLD}最佳 Kp_angle = {best_kpa:.2f}{RESET}")
    return best_kpa


# ── 主流程 ────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Roll 单轴 PID 自主调参")
    parser.add_argument("--port", help="串口设备路径")
    parser.add_argument("--phase", default="ABCD", help="执行阶段 (A/B/C/D/ABCD)")
    args = parser.parse_args()

    # 连接串口
    port = args.port or auto_detect_port()
    if not port:
        print(f"{RED}[ERROR] 未检测到无人机串口{RESET}")
        sys.exit(1)

    print(f"[*] 连接: {port} @ {BAUD}")
    drone = DroneSerial(port)

    # 安全退出
    def safe_exit(_sig=None, _frame=None):
        print(f"\n{YELLOW}[安全停机]{RESET}")
        drone.stop()
        drone.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, safe_exit)

    # 等待 AHRS 稳定
    print("[*] 等待 AHRS 稳定 (5s)...")
    time.sleep(5)
    drone.ser.read(4096)

    # 连通性测试
    resp = drone.send("att")
    if "Euler:" in resp:
        print(f"{GREEN}[OK] 串口通信正常{RESET}")
        print(f"  {resp}")
    else:
        print(f"{YELLOW}[WARN] att 响应异常: {resp[:100]}{RESET}")

    drone.stop()

    # 调参状态
    kp_rate = 0.10
    kd_rate = 0.02
    ki_rate = 0.00
    kp_angle = 4.0
    phase = args.phase.upper()

    # ── 阶段 A ──
    if "A" in phase:
        kp_rate, osc_kp = phase_a(drone)
        if osc_kp:
            print(f"\n{BOLD}阶段 A 结论: Kp_rate = {kp_rate:.2f} (振荡阈值 {osc_kp:.2f} × {KP_FACTOR}){RESET}")
        else:
            print(f"\n{BOLD}阶段 A 结论: Kp_rate = {kp_rate:.2f} (保守值){RESET}")
        print(f"  进入阶段 B 前请确认无人机已复位")
        wait_reset()

    # ── 阶段 B ──
    if "B" in phase:
        kp_rate, kd_rate = phase_b(drone, kp_rate)
        print(f"\n{BOLD}阶段 B 结论: Kp_rate={kp_rate:.2f} Kd_rate={kd_rate:.2f}{RESET}")
        wait_reset()

    # ── 阶段 C ──
    if "C" in phase:
        ki_rate = phase_c(drone, kp_rate, kd_rate)
        print(f"\n{BOLD}阶段 C 结论: Ki_rate={ki_rate:.2f}{RESET}")
        wait_reset()

    # ── 阶段 D ──
    if "D" in phase:
        kp_angle = phase_d(drone, kp_rate, ki_rate, kd_rate)
        print(f"\n{BOLD}阶段 D 结论: Kp_angle={kp_angle:.2f}{RESET}")

    # ── 最终汇总 ──
    print_header("最终调参结果")
    print(f"""
  Roll 轴 PID 参数:
  ┌─────────────────────────────────────────┐
  │  角速度环 (Rate, 200Hz):                │
  │    Kp_rate  = {kp_rate:6.2f}                    │
  │    Ki_rate  = {ki_rate:6.2f}                    │
  │    Kd_rate  = {kd_rate:6.2f}                    │
  │                                         │
  │  角度环 (Angle, 50Hz):                  │
  │    Kp_angle = {kp_angle:6.2f}                    │
  │    Ki_angle =   0.00                    │
  │    Kd_angle =   0.00                    │
  └─────────────────────────────────────────┘

  {YELLOW}注意: pidr/pida 命令只更新运行时参数, 重启后丢失。
  最终参数需手动保存到 Flash。{RESET}
""")

    drone.close()
    print("[*] 调参完成")


if __name__ == "__main__":
    main()
