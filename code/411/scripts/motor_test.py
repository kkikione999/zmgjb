#!/usr/bin/env python3
"""
motor_test.py — 电机测试工具
通过 USB 串口连接 STM32，交互式测试电机 PWM、混控和启动阈值。

用法:
    python3 motor_test.py                       # 自动检测串口
    python3 motor_test.py --port /dev/tty.xxx   # 指定串口
"""

import argparse
import glob
import os
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
READ_TIMEOUT = 0.1          # 读取响应超时 100ms
SWEEP_STEP = 2              # 扫频步进 PWM
SWEEP_MAX = 80              # 最大 PWM
SWEEP_HOLD = 1.0            # 每步保持秒数
ENCODING = "utf-8"


# ── 串口自动检测 ──────────────────────────────────────────────
def auto_detect_port():
    """在 macOS 上自动查找 USB 串口设备。"""
    patterns = [
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
    ]
    candidates = []
    for pat in patterns:
        candidates.extend(sorted(glob.glob(pat)))

    if not candidates:
        return None

    # 优先 cu 设备（macOS 上可同时读写）
    for c in candidates:
        if "/cu." in c:
            return c
    return candidates[0]


def open_serial(port):
    """打开串口并清空缓冲区。"""
    try:
        ser = serial.Serial(port, BAUD, timeout=READ_TIMEOUT)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] 无法打开 {port}: {e}")
        return None


# ── 串口通信 ──────────────────────────────────────────────────
def send_command(ser, cmd):
    """发送命令并读取 STM32 响应。"""
    tx = (cmd.strip() + "\n").encode(ENCODING, errors="replace")
    ser.write(tx)
    ser.flush()

    # 读取响应（等待直到无新数据或超时）
    time.sleep(0.05)
    response_lines = []
    deadline = time.time() + READ_TIMEOUT
    while time.time() < deadline:
        data = ser.read(256)
        if data:
            text = data.decode(ENCODING, errors="replace")
            response_lines.append(text)
            deadline = time.time() + READ_TIMEOUT   # 收到数据则续期
        else:
            break

    return "".join(response_lines).strip()


def send_stop(ser):
    """发送紧急停机命令。"""
    if ser and ser.is_open:
        try:
            send_command(ser, "stop")
        except Exception:
            pass


# ── 命令处理 ──────────────────────────────────────────────────
def print_banner():
    print()
    print("Motor Test Tool — STM32 USART1 (460800)")
    print("=" * 42)
    print("m<N> <pwm>  — 设置电机 N (1-4) PWM (0-80)")
    print("all <pwm>   — 所有电机设为相同 PWM")
    print("stop        — 紧急停机")
    print("mix T R P Y — 测试混控 (浮点值)")
    print("sweep <N>   — 扫频找电机最低启动 PWM")
    print("quit        — 退出 (先发 stop)")
    print("=" * 42)


def handle_sweep(ser, motor_num):
    """
    扫频测试：逐步增加 PWM，找到电机最低启动值。
    在 Python 端实现，不依赖 STM32 特殊命令。
    """
    if motor_num < 1 or motor_num > 4:
        print(f"[ERROR] 电机编号必须是 1-4，收到 {motor_num}")
        return

    print(f"\n--- 电机 {motor_num} 扫频测试 ---")
    print(f"从 PWM=0 开始，步进 {SWEEP_STEP}，最大 {SWEEP_MAX}")
    print("观察电机是否转动，输入 y=转动 n=未转 q=退出")
    print()

    min_start = None

    for pwm in range(0, SWEEP_MAX + 1, SWEEP_STEP):
        cmd = f"m{motor_num} {pwm}"
        resp = send_command(ser, cmd)
        print(f"  PWM={pwm:>3d}  {resp}" if resp else f"  PWM={pwm:>3d}")

        if pwm == 0:
            # PWM=0 只是确认电机停转，跳过询问
            time.sleep(SWEEP_HOLD)
            continue

        try:
            answer = input("    电机转动了吗？(y/n/q): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            answer = "q"

        if answer == "q":
            print("  扫频已中止")
            break
        elif answer == "y":
            min_start = pwm
            print(f"  >>> 电机 {motor_num} 最低启动 PWM = {min_start}")
            break

        time.sleep(SWEEP_HOLD)

    # 扫频结束，关闭该电机
    send_command(ser, f"m{motor_num} 0")

    if min_start is not None:
        print(f"\n[结果] 电机 {motor_num} 最低启动 PWM = {min_start}")
    else:
        print(f"\n[结果] 电机 {motor_num} 在 PWM 0-{SWEEP_MAX} 范围内未启动")
        print("  可能原因：接线问题、电机损坏、或 PWM 范围不够")


def process_command(ser, user_input):
    """解析并执行用户命令。"""
    parts = user_input.strip().split()
    if not parts:
        return True

    cmd = parts[0].lower()

    # sweep 命令在本地处理
    if cmd == "sweep":
        if len(parts) < 2:
            print("[ERROR] 用法: sweep <N>  (N=1-4)")
            return True
        try:
            motor_num = int(parts[1])
        except ValueError:
            print(f"[ERROR] 电机编号必须是整数: {parts[1]}")
            return True
        handle_sweep(ser, motor_num)
        return True

    if cmd == "quit" or cmd == "exit":
        return False

    # 所有其他命令直接转发给 STM32
    resp = send_command(ser, user_input.strip())
    if resp:
        print(resp)
    return True


# ── 主流程 ────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="电机测试工具 — STM32 串口交互")
    parser.add_argument("--port", help="指定串口设备路径 (如 /dev/tty.usbmodem1234)")
    args = parser.parse_args()

    # 查找串口
    port = args.port
    if not port:
        port = auto_detect_port()
        if not port:
            print("[ERROR] 未检测到 USB 串口设备")
            print("  请用 --port 参数指定，例如:")
            print("  python3 motor_test.py --port /dev/tty.usbmodem1234")
            sys.exit(1)

    print(f"连接串口: {port} @ {BAUD}")

    # 打开串口
    ser = open_serial(port)
    if not ser:
        sys.exit(1)

    # 注册退出处理：Ctrl+C 和正常退出都先发 stop
    def safe_exit(signum=None, frame=None):
        print("\n[安全停机] 发送 stop ...")
        send_stop(ser)
        if ser.is_open:
            ser.close()
        print("已退出")
        sys.exit(0)

    signal.signal(signal.SIGINT, safe_exit)

    # 启动时确保电机停止
    print("发送初始 stop ...")
    send_stop(ser)
    time.sleep(0.2)

    # 打印菜单
    print_banner()

    # 交互循环
    try:
        while True:
            try:
                user_input = input("> ").strip()
            except EOFError:
                break

            if not user_input:
                continue

            if not process_command(ser, user_input):
                break
    finally:
        print("\n[安全停机] 发送 stop ...")
        send_stop(ser)
        ser.close()
        print("已退出")


if __name__ == "__main__":
    main()
