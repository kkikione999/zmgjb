#!/usr/bin/env python3
"""
roll_kp_scan.py -- Roll Rate Kp 自动扫描
无人机固定在 Roll 轴，加桨，方向已验证正确。
自动遍历多个 Kp 值，采集差速和 Roll 角度数据，输出汇总表和建议值。

用法:
    python3 scripts/roll_kp_scan.py
    python3 scripts/roll_kp_scan.py --port /dev/cu.usbmodem1234
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
    print("[ERROR] 需要安装 pyserial: pip3 install pyserial")
    sys.exit(1)

# ── 常量 ──────────────────────────────────────────────────────
BAUD = 460800
ENCODING = "utf-8"

KP_SCAN_VALUES = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.80]
THROTTLE_PCT = 50
SETTLE_TIME = 2.0       # Kp 切换后稳定等待
NUM_SAMPLES = 25         # 每个 Kp 采样次数
SAMPLE_INTERVAL = 0.2    # 采样间隔 (秒)
RC_REFRESH_INTERVAL = 5  # 每 5 次采样重发 rc

PWM_RE = re.compile(r"PWM:\s*M1=(\d+)\s*M2=(\d+)\s*M3=(\d+)\s*M4=(\d+)")
ATT_RE = re.compile(
    r"Euler:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s+"
    r"Gyro:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)"
)

# ANSI 颜色
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


# ── 串口自动检测 ──────────────────────────────────────────────
def auto_detect_port():
    """遍历 /dev/cu.usbmodem*，发送 'motors' 命令，匹配 'PWM:' 的端口。"""
    candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not candidates:
        return None

    for port in candidates:
        try:
            ser = serial.Serial(port, BAUD, timeout=0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            resp = send_cmd(ser, "motors")
            ser.close()
            if "PWM:" in resp:
                print(f"{GREEN}[OK]{RESET} 检测到无人机串口: {port}")
                return port
            else:
                print(f"[--] {port} 有响应但无 PWM 数据, 跳过")
        except Exception as e:
            print(f"[--] {port} 无法打开: {e}")
    return None


# ── 串口通信 ──────────────────────────────────────────────────
def send_cmd(ser, cmd):
    """发送命令并读取响应。"""
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode(ENCODING))
    ser.flush()
    time.sleep(0.05)
    return ser.read(1024).decode(ENCODING, errors="replace").strip()


def parse_motors(resp):
    """解析 motors 命令响应, 返回 (M1, M2, M3, M4) 或 None。"""
    m = PWM_RE.search(resp)
    if m:
        return (int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)))
    return None


def parse_att(resp):
    """解析 att 命令响应, 返回 (roll, pitch, yaw, gx, gy, gz) 或 None。"""
    m = ATT_RE.search(resp)
    if m:
        return {
            "roll": float(m.group(1)), "pitch": float(m.group(2)),
            "yaw": float(m.group(3)), "gx": float(m.group(4)),
            "gy": float(m.group(5)), "gz": float(m.group(6)),
        }
    return None


# ── 输出辅助 ──────────────────────────────────────────────────
def print_header(title):
    print()
    print(f"{BOLD}{CYAN}{'=' * 60}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'=' * 60}{RESET}")


def rating_label(max_diff):
    """根据最大差速返回评级标签。"""
    if max_diff > 50:
        return f"{GREEN}{BOLD}强{RESET}"
    elif max_diff >= 20:
        return f"{YELLOW}中等{RESET}"
    else:
        return f"{RED}弱{RESET}"


# ── 单次 Kp 扫描 ─────────────────────────────────────────────
def scan_one_kp(ser, kp):
    """对一个 Kp 值执行完整的扫描流程。返回结果字典。"""
    kp_x100 = int(kp * 100)

    # a. 发送初始化命令
    init_cmds = [
        "pidrst",
        "pida 0 0 0 0",   # 关闭 Roll 角度环
        "pida 1 0 0 0",   # 关闭 Pitch 角度环
        "pida 2 0 0 0",   # 关闭 Yaw 角度环
        "pidr 1 0 0 0",   # 关闭 Pitch rate
        "pidr 2 0 0 0",   # 关闭 Yaw rate
        f"pidr 0 {kp_x100} 0 0",  # 设 Roll rate Kp
    ]
    for cmd in init_cmds:
        send_cmd(ser, cmd)

    # b. 发 rc 50 0 0 0 (50% 油门)
    send_cmd(ser, f"rc {THROTTLE_PCT} 0 0 0")

    # c. 等 2 秒稳定
    time.sleep(SETTLE_TIME)

    # d. 提示用户
    print()
    print(f"{BOLD}{YELLOW}>>> Kp = {kp:.2f} 请你执行: 左右倾斜无人机 5 秒{RESET}")

    # e. 持续读 motors+att 25 次
    samples = []
    for i in range(NUM_SAMPLES):
        # 每 5 次采样后重发一次 rc 命令防超时
        if i > 0 and i % RC_REFRESH_INTERVAL == 0:
            send_cmd(ser, f"rc {THROTTLE_PCT} 0 0 0")

        motors_resp = send_cmd(ser, "motors")
        att_resp = send_cmd(ser, "att")

        motors = parse_motors(motors_resp)
        att = parse_att(att_resp)

        if motors:
            m1, m2, m3, m4 = motors
            left = m1 + m4
            right = m2 + m3
            diff = left - right
        else:
            m1 = m2 = m3 = m4 = None
            diff = None

        roll = att["roll"] if att else None

        # 打印每次采样
        if roll is not None and diff is not None:
            print(f"  #{i+1:2d}  Roll={roll:+7.2f}  diff={diff:+5d}"
                  f"  M1={m1} M2={m2} M3={m3} M4={m4}")
        else:
            tag_parts = []
            if roll is None:
                tag_parts.append("att=ERR")
            if diff is None:
                tag_parts.append("motors=ERR")
            print(f"  #{i+1:2d}  [{', '.join(tag_parts)}]")

        samples.append({"roll": roll, "diff": diff})

        time.sleep(SAMPLE_INTERVAL)

    # f. 停机 rc 0 0 0 0
    send_cmd(ser, "rc 0 0 0 0")

    # g. 计算结果
    valid = [s for s in samples if s["roll"] is not None and s["diff"] is not None]
    if not valid:
        return {"kp": kp, "max_diff": 0, "roll_min": 0, "roll_max": 0, "rating": "无数据"}

    abs_diffs = [abs(s["diff"]) for s in valid]
    rolls = [s["roll"] for s in valid]
    max_diff = max(abs_diffs)
    roll_min = min(rolls)
    roll_max = max(rolls)

    result = {
        "kp": kp,
        "max_diff": max_diff,
        "roll_min": roll_min,
        "roll_max": roll_max,
        "roll_range": roll_max - roll_min,
        "num_valid": len(valid),
        "rating": rating_label(max_diff),
    }

    print()
    print(f"  {CYAN}结果:{RESET} 最大差速={max_diff}  "
          f"Roll 范围=[{roll_min:+.1f}, {roll_max:+.1f}]  "
          f"评级={rating_label(max_diff)}")

    return result


# ── 主流程 ────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Roll Rate Kp 自动扫描")
    parser.add_argument("--port", help="指定串口设备路径")
    args = parser.parse_args()

    # 1. 自动检测串口，连接
    port = args.port
    if not port:
        print(f"{BOLD}Roll Rate Kp 自动扫描{RESET}")
        print("[*] 自动检测串口...")
        port = auto_detect_port()
        if not port:
            print(f"{RED}[ERROR] 未检测到无人机串口{RESET}")
            print("  请用 --port 参数指定, 例如:")
            print("  python3 scripts/roll_kp_scan.py --port /dev/cu.usbmodem1234")
            sys.exit(1)

    print(f"[*] 连接串口: {port} @ {BAUD}")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"{RED}[ERROR] 无法打开 {port}: {e}{RESET}")
        sys.exit(1)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # 安全退出: Ctrl-C 时停机
    def safe_exit(_signum=None, _frame=None):
        print(f"\n{YELLOW}[安全停机]{RESET}")
        try:
            ser.write(b"rc 0 0 0 0\n")
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, safe_exit)

    # 2. 等 2 秒排空缓冲区
    print("[*] 等待 2 秒排空缓冲区...")
    time.sleep(2.0)
    ser.reset_input_buffer()

    # 连通性测试
    test_resp = send_cmd(ser, "motors")
    if PWM_RE.search(test_resp):
        print(f"{GREEN}[OK] 串口通信正常{RESET}")
    else:
        print(f"{YELLOW}[WARN] motors 命令未得到预期响应{RESET}")
        print(f"  收到: {repr(test_resp[:200])}")
        print("  尝试继续...")

    # 确保初始停机
    send_cmd(ser, "rc 0 0 0 0")

    print_header(f"开始 Kp 扫描: {KP_SCAN_VALUES}")
    print(f"  每个 Kp 测试 {NUM_SAMPLES} 次采样 ({NUM_SAMPLES * SAMPLE_INTERVAL:.1f}s)")
    print(f"  油门: {THROTTLE_PCT}%")
    print(f"  请在每个 Kp 测试期间左右倾斜无人机")
    print()

    # 3. Kp 扫描
    all_results = []
    for idx, kp in enumerate(KP_SCAN_VALUES):
        print_header(f"[{idx+1}/{len(KP_SCAN_VALUES)}] Kp = {kp:.2f}")
        result = scan_one_kp(ser, kp)
        all_results.append(result)

        # Kp 之间短暂停顿
        time.sleep(0.5)

    # 确保停机
    send_cmd(ser, "rc 0 0 0 0")

    # 5. 汇总表
    print_header("Kp 扫描汇总")
    print()
    print(f"  {'Kp':>6s}  {'最大差速':>8s}  {'Roll范围':>10s}  {'评级':>6s}")
    print(f"  {'-'*6}  {'-'*8}  {'-'*10}  {'-'*6}")

    for r in all_results:
        kp_str = f"{r['kp']:.2f}"
        diff_str = f"{r['max_diff']}"
        roll_str = f"[{r['roll_min']:+.1f}, {r['roll_max']:+.1f}]"
        print(f"  {kp_str:>6s}  {diff_str:>8s}  {roll_str:>10s}  {r['rating']}")

    # 6. 建议起始 Kp
    print()
    # 找最大差速 > 80 的最小 Kp
    strong_kps = [r for r in all_results if r["max_diff"] > 80]
    if strong_kps:
        # 已按扫描顺序排列, 第一个 >80 的就是最小的
        rec_kp = strong_kps[0]["kp"] * 0.65
        print(f"  {GREEN}{BOLD}建议起始 Kp = {rec_kp:.2f}{RESET}  "
              f"(差速>80 的最小 Kp={strong_kps[0]['kp']:.2f} x 0.65)")
    else:
        # 找差速最大的 Kp
        best = max(all_results, key=lambda r: r["max_diff"])
        rec_kp = best["kp"] * 0.65
        print(f"  {YELLOW}未找到差速>80 的 Kp 值{RESET}")
        print(f"  {YELLOW}最大差速={best['max_diff']} (Kp={best['kp']:.2f}){RESET}")
        print(f"  {YELLOW}建议先增大 Kp 扫描上限再重试{RESET}")
        print(f"  {YELLOW}或使用当前最强响应: Kp={rec_kp:.2f} ({best['kp']:.2f} x 0.65){RESET}")

    print()
    print(f"[*] 扫描完成。安全停机 rc 0 0 0 0")
    ser.close()
    print("[*] 串口已关闭")


if __name__ == "__main__":
    main()
