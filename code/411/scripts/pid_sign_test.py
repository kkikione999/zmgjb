#!/usr/bin/env python3
"""
pid_sign_test.py -- Phase 0: PID 符号方向自动验证
验证 Roll / Pitch / Yaw 三轴 PID 闭环修正方向是否正确。

用法:
    python3 pid_sign_test.py                       # 自动检测串口
    python3 pid_sign_test.py --port /dev/tty.xxx   # 指定串口
"""

import argparse
import glob
import re
import signal
import sys
import time
import threading

try:
    import serial
except ImportError:
    print("[ERROR] 需要安装 pyserial: pip3 install pyserial")
    sys.exit(1)

# ── 常量 ──────────────────────────────────────────────────────
BAUD = 460800
ENCODING = "utf-8"
PWM_RE = re.compile(r"PWM:\s*M1=(\d+)\s*M2=(\d+)\s*M3=(\d+)\s*M4=(\d+)")

# ANSI 颜色
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


# ── 串口自动检测 ──────────────────────────────────────────────
def auto_detect_port():
    patterns = [
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
    ]
    candidates = []
    for pat in patterns:
        candidates.extend(sorted(glob.glob(pat)))
    if not candidates:
        return None
    for c in candidates:
        if "/cu." in c:
            return c
    return candidates[0]


def open_serial(port):
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] 无法打开 {port}: {e}")
        return None


# ── 串口通信 ──────────────────────────────────────────────────
def drain(ser, timeout=0.3):
    """排空输入缓冲区，返回所有读到的文本。"""
    buf = ""
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = ser.read(4096)
        if data:
            buf += data.decode(ENCODING, errors="replace")
        else:
            break
    return buf


def send_line(ser, cmd):
    """发送一行命令（追加 \\n）。"""
    ser.write((cmd.strip() + "\n").encode(ENCODING))
    ser.flush()


def send_and_drain(ser, cmd, delay=0.1):
    """发送命令并排空响应。"""
    send_line(ser, cmd)
    time.sleep(delay)
    return drain(ser)


def read_motors(ser):
    """发送 motors 命令并解析 PWM 值。返回 (m1, m2, m3, m4) 或 None。"""
    # 先排空缓冲区
    drain(ser, 0.2)
    # 发送 motors
    send_line(ser, "motors")
    # 读响应，最多等 500ms
    resp = drain(ser, 0.5)
    match = PWM_RE.search(resp)
    if match:
        return tuple(int(match.group(i)) for i in range(1, 5))
    # 调试：打印原始响应
    print(f"  {YELLOW}[DEBUG] motors 原始响应: {repr(resp[:200])}{RESET}")
    return None


def read_motors_with_rc(ser, throttle_pct=55, max_attempts=8):
    """
    循环发送 rc 命令维持油门，同时尝试读取 motors。
    解决 rc 命令 200ms 超时问题。
    """
    for attempt in range(max_attempts):
        # 每次先发 rc 维持油门
        send_line(ser, f"rc {throttle_pct} 0 0 0")
        time.sleep(0.02)  # 等 20ms 让 maneuver_task 处理
        # 排空缓冲区
        drain(ser, 0.05)
        # 发送 motors
        send_line(ser, "motors")
        time.sleep(0.05)
        resp = drain(ser, 0.3)
        match = PWM_RE.search(resp)
        if match:
            return tuple(int(match.group(i)) for i in range(1, 5))
        if attempt == 2:
            print(f"  {YELLOW}[DEBUG] 尝试 #{attempt+1} 原始: {repr(resp[:200])}{RESET}")
    print(f"  {RED}[DEBUG] 全部 {max_attempts} 次尝试均未匹配 PWM{RESET}")
    return None


# ── 输出辅助 ──────────────────────────────────────────────────
def print_header(title):
    print()
    print(f"{BOLD}{CYAN}{'=' * 55}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'=' * 55}{RESET}")


def print_result(label, correct, detail):
    if correct:
        tag = f"{GREEN}{BOLD}PASS -- 方向正确{RESET}"
    else:
        tag = f"{RED}{BOLD}FAIL -- 方向反了！{RESET}"
    print(f"  {label}: {tag}")
    print(f"    {detail}")


# ── 测试用例 ──────────────────────────────────────────────────
def test_roll(ser):
    """Test 1: Roll 方向 -- 右倾斜时左电机应加大。"""
    print_header("Test 1: Roll 方向验证")

    # 发送 PID 参数（不发油门，避免超时问题）
    setup_cmds = [
        "pidrst",
        "pida 0 100 0 0",
        "pida 1 100 0 0",
        "pidr 0 10 0 0",
        "pidr 1 10 0 0",
    ]
    print("  发送 PID 参数...")
    for cmd in setup_cmds:
        send_and_drain(ser, cmd, 0.1)

    print()
    print(f"  {YELLOW}请将无人机向右倾斜约15度，保持住，然后按回车...{RESET}")
    print(f"  {YELLOW}(脚本会自动发送油门并读取电机){RESET}")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        print("  已跳过")
        return None

    print("  读取电机 PWM（保持倾斜！）...")
    vals = read_motors_with_rc(ser, throttle_pct=55)
    if not vals:
        print(f"  {RED}[ERROR] 无法读取电机 PWM{RESET}")
        return None

    m1, m2, m3, m4 = vals
    print(f"  PWM: M1={m1}  M2={m2}  M3={m3}  M4={m4}")

    left  = m1 + m4
    right = m2 + m3
    print(f"  左侧(M1+M4)={left}  右侧(M2+M3)={right}")

    correct = left > right
    detail = (
        f"右倾 → 左侧应加速对抗倾斜。"
        f" 左={left} 右={right} 差={left - right:+d}"
    )
    print_result("Roll", correct, detail)

    # 停机
    send_and_drain(ser, "rc 0 0 0 0", 0.1)
    return correct


def test_pitch(ser):
    """Test 2: Pitch 方向 -- 机头上仰时前电机应减速、后电机应加速。"""
    print_header("Test 2: Pitch 方向验证")

    setup_cmds = [
        "pidrst",
        "pida 0 100 0 0",
        "pida 1 100 0 0",
        "pidr 0 10 0 0",
        "pidr 1 10 0 0",
    ]
    print("  发送 PID 参数...")
    for cmd in setup_cmds:
        send_and_drain(ser, cmd, 0.1)

    print()
    print(f"  {YELLOW}请将无人机机头上仰约15度，保持住，然后按回车...{RESET}")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        print("  已跳过")
        return None

    print("  读取电机 PWM（保持倾斜！）...")
    vals = read_motors_with_rc(ser, throttle_pct=55)
    if not vals:
        print(f"  {RED}[ERROR] 无法读取电机 PWM{RESET}")
        return None

    m1, m2, m3, m4 = vals
    print(f"  PWM: M1={m1}  M2={m2}  M3={m3}  M4={m4}")

    front = m1 + m2
    rear  = m3 + m4
    print(f"  前侧(M1+M2)={front}  后侧(M3+M4)={rear}")

    correct = front < rear
    detail = (
        f"上仰 → 前侧应减速、后侧应加速对抗倾斜。"
        f" 前={front} 后={rear} 差={rear - front:+d}"
    )
    print_result("Pitch", correct, detail)

    send_and_drain(ser, "rc 0 0 0 0", 0.1)
    return correct


def test_yaw(ser):
    """Test 3: Yaw 方向 -- 正偏航指令时 CCW 电机应加速。"""
    print_header("Test 3: Yaw 方向验证")

    setup_cmds = [
        "pidrst",
        "pidr 2 10 0 0",
    ]
    print("  发送 PID 参数...")
    for cmd in setup_cmds:
        send_and_drain(ser, cmd, 0.1)

    print()
    print(f"  {YELLOW}保持水平，按回车发送偏航指令并读取电机...{RESET}")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        print("  已跳过")
        return None

    print("  发送偏航指令并读取电机 PWM...")
    # 发送带 yaw 的 rc 命令并读取 motors
    for attempt in range(8):
        send_line(ser, "rc 55 0 0 20")
        time.sleep(0.02)
        drain(ser, 0.03)
        send_line(ser, "motors")
        time.sleep(0.05)
        resp = drain(ser, 0.3)
        match = PWM_RE.search(resp)
        if match:
            m1, m2, m3, m4 = tuple(int(match.group(i)) for i in range(1, 5))
            break
    else:
        print(f"  {RED}[ERROR] 无法读取电机 PWM{RESET}")
        send_and_drain(ser, "rc 0 0 0 0", 0.1)
        return None

    print(f"  PWM: M1={m1}  M2={m2}  M3={m3}  M4={m4}")

    cw  = m1 + m3
    ccw = m2 + m4
    print(f"  CW(M1+M3)={cw}  CCW(M2+M4)={ccw}")

    correct = ccw > cw
    detail = (
        f"正偏航(机头右转) → CCW电机应加速产生反扭矩。"
        f" CW={cw} CCW={ccw} 差={ccw - cw:+d}"
    )
    print_result("Yaw", correct, detail)

    send_and_drain(ser, "rc 0 0 0 0", 0.1)
    return correct


# ── 主流程 ────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Phase 0: PID 符号方向验证")
    parser.add_argument("--port", help="指定串口设备路径")
    args = parser.parse_args()

    port = args.port
    if not port:
        port = auto_detect_port()
        if not port:
            print("[ERROR] 未检测到 USB 串口设备")
            print("  请用 --port 参数指定")
            sys.exit(1)

    print(f"{BOLD}PID Sign Test -- Phase 0{RESET}")
    print(f"连接串口: {port} @ {BAUD}")

    ser = open_serial(port)
    if not ser:
        sys.exit(1)

    def safe_exit(_signum=None, _frame=None):
        print(f"\n{YELLOW}[安全停机]{RESET}")
        try:
            send_line(ser, "rc 0 0 0 0")
        except Exception:
            pass
        if ser.is_open:
            ser.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, safe_exit)

    # 清空启动输出
    print("清空串口缓冲区...")
    drain(ser, 1.0)
    send_and_drain(ser, "rc 0 0 0 0", 0.1)

    # 简单连通性测试
    print("测试串口连通性...")
    drain(ser, 0.3)
    send_line(ser, "motors")
    time.sleep(0.1)
    test_resp = drain(ser, 0.5)
    if PWM_RE.search(test_resp):
        print(f"  {GREEN}串口通信正常，motors 命令可用{RESET}")
    else:
        print(f"  {YELLOW}[WARN] motors 命令未得到预期响应{RESET}")
        print(f"  [DEBUG] 收到: {repr(test_resp[:300])}")
        print(f"  尝试继续...")

    print()
    print(f"{BOLD}本工具将引导你完成 Roll / Pitch / Yaw 三轴 PID 方向验证。{RESET}")
    print(f"  {YELLOW}注意: 请确保螺旋桨已拆除！{RESET}")
    print()
    try:
        input("按回车开始测试...")
    except (EOFError, KeyboardInterrupt):
        safe_exit()

    # 执行三项测试
    results = {}
    results["Roll"]  = test_roll(ser)
    results["Pitch"] = test_pitch(ser)
    results["Yaw"]   = test_yaw(ser)

    # 安全停机
    send_and_drain(ser, "rc 0 0 0 0", 0.1)

    # 汇总
    print_header("测试汇总")
    all_pass = True
    for axis, result in results.items():
        if result is None:
            tag = f"{YELLOW}SKIP{RESET}"
        elif result:
            tag = f"{GREEN}{BOLD}PASS -- 方向正确{RESET}"
        else:
            tag = f"{RED}{BOLD}FAIL -- 方向反了！{RESET}"
            all_pass = False
        print(f"  {axis:>5s}: {tag}")

    print()
    if all_pass:
        print(f"  {GREEN}{BOLD}全部通过！可以进入调参阶段。{RESET}")
    else:
        print(f"  {RED}{BOLD}存在方向错误！混控符号需要修复！{RESET}")
        print(f"  请检查 freertos.c 混控公式 (第 717-720 行)")

    print()
    ser.close()
    print("串口已关闭。")


if __name__ == "__main__":
    main()
