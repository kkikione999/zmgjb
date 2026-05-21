#!/usr/bin/env python3
"""
roll_dir_test.py — Roll 方向验证测试
自动化测试无人机 Roll 轴混控方向是否正确。

测试逻辑:
  1. 连接 STM32 串口
  2. 初始化 PID 参数 (仅开 Roll rate Kp)
  3. 设置 50% 油门
  4. 提示用户向右倾斜无人机
  5. 连续读取 motors+att 30 次
  6. 判断右倾时左侧电机是否更快 (diff > 0)

用法:
    python3 scripts/roll_dir_test.py
    python3 scripts/roll_dir_test.py --port /dev/cu.usbmodem1234
"""

import argparse
import glob
import sys
import time

try:
    import serial
except ImportError:
    print("[ERROR] 需要安装 pyserial: pip3 install pyserial")
    sys.exit(1)

# ── 常量 ──────────────────────────────────────────────────────
BAUD = 460800
READ_TIMEOUT = 0.1
ENCODING = "utf-8"

NUM_SAMPLES = 30          # 采样次数
SAMPLE_INTERVAL = 0.2     # 每次采样间隔 (秒)
SETTLE_TIME = 5.0         # AHRS 稳定等待时间 (秒)
RC_REFRESH = 1.0          # rc 命令重发间隔 (秒)
ROLL_THRESHOLD = 5.0      # 判定有效倾斜的角度阈值


# ── 串口自动检测 ──────────────────────────────────────────────
def auto_detect_port():
    """遍历 /dev/cu.usbmodem* 找到能响应 'motors' 命令的端口。"""
    candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not candidates:
        return None

    for port in candidates:
        try:
            ser = serial.Serial(port, BAUD, timeout=READ_TIMEOUT)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            resp = send_command(ser, "motors")
            ser.close()
            if "PWM" in resp:
                print(f"[OK] 检测到无人机串口: {port}")
                return port
            else:
                print(f"[--] {port} 有响应但无 PWM 数据, 跳过")
        except Exception as e:
            print(f"[--] {port} 无法打开: {e}")
    return None


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
    ser.reset_input_buffer()
    tx = (cmd.strip() + "\n").encode(ENCODING, errors="replace")
    ser.write(tx)
    ser.flush()
    time.sleep(0.05)

    response_lines = []
    deadline = time.time() + READ_TIMEOUT
    while time.time() < deadline:
        data = ser.read(256)
        if data:
            text = data.decode(ENCODING, errors="replace")
            response_lines.append(text)
            deadline = time.time() + READ_TIMEOUT
        else:
            break

    return "".join(response_lines).strip()


def parse_motors(resp):
    """解析 motors 命令响应, 返回 (M1, M2, M3, M4) 或 None。"""
    # 格式: "PWM: M1=xxx M2=xxx M3=xxx M4=xxx"
    try:
        values = {}
        for part in resp.split():
            if "=" in part:
                k, v = part.split("=", 1)
                # 去掉可能的非数字后缀
                v = v.strip(",")
                values[k] = int(v)
        return (values.get("M1"), values.get("M2"),
                values.get("M3"), values.get("M4"))
    except Exception:
        return None


def parse_att(resp):
    """解析 att 命令响应, 返回 (roll, pitch, yaw, gx, gy, gz) 或 None。"""
    # 格式: "Euler: Roll, Pitch, Yaw  Gyro: gx, gy, gz"
    try:
        parts = resp.replace(",", " ").split()
        roll = pitch = yaw = gx = gy = gz = None
        # 找 Euler 后面的三个数
        if "Euler:" in parts:
            idx = parts.index("Euler:")
            roll = float(parts[idx + 1])
            pitch = float(parts[idx + 2])
            yaw = float(parts[idx + 3])
        # 找 Gyro 后面的三个数
        if "Gyro:" in parts:
            idx = parts.index("Gyro:")
            gx = float(parts[idx + 1])
            gy = float(parts[idx + 2])
            gz = float(parts[idx + 3])
        if roll is not None:
            return (roll, pitch, yaw, gx, gy, gz)
    except Exception:
        pass
    return None


# ── 测试主流程 ────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Roll 方向验证测试")
    parser.add_argument("--port", help="指定串口设备路径")
    args = parser.parse_args()

    # 1. 查找串口
    port = args.port
    if not port:
        print("[*] 自动检测串口...")
        port = auto_detect_port()
        if not port:
            print("[ERROR] 未检测到无人机串口")
            print("  请用 --port 参数指定, 例如:")
            print("  python3 scripts/roll_dir_test.py --port /dev/cu.usbmodem1234")
            sys.exit(1)

    print(f"[*] 连接串口: {port} @ {BAUD}")
    ser = open_serial(port)
    if not ser:
        sys.exit(1)

    try:
        # 2. 等待排空缓冲区
        print("[*] 等待 2 秒排空缓冲区...")
        time.sleep(2.0)
        ser.reset_input_buffer()

        # 3. 初始化 PID 参数
        print("[*] 初始化 PID 参数...")
        init_cmds = [
            "pidrst",
            "pida 0 0 0 0",    # 关闭 Roll 角度环
            "pida 1 0 0 0",    # 关闭 Pitch 角度环
            "pida 2 0 0 0",    # 关闭 Yaw 角度环
            "pidr 0 30 0 0",   # Roll rate Kp=0.30
            "pidr 1 0 0 0",    # 关闭 Pitch rate
            "pidr 2 0 0 0",    # 关闭 Yaw rate
        ]
        for cmd in init_cmds:
            resp = send_command(ser, cmd)
            print(f"  >> {cmd}  =>  {resp}")

        # 4. 设置油门
        print("[*] 设置油门 50% (rc 50 0 0 0)")
        send_command(ser, "rc 50 0 0 0")

        # 5. 等待 AHRS 稳定, 期间持续重发 rc 命令
        print(f"[*] 等待 {SETTLE_TIME:.0f} 秒让 AHRS 稳定...")
        deadline = time.time() + SETTLE_TIME
        while time.time() < deadline:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            sleep_time = min(RC_REFRESH, remaining)
            time.sleep(sleep_time)
            send_command(ser, "rc 50 0 0 0")

        # 6. 提示用户操作 (用倒计时代替 input, 因为非交互环境无法读取 stdin)
        print()
        print("=" * 60)
        print("  请你执行: 向右倾斜无人机约 15 度 并保持住")
        print("  (倒计时结束后自动开始采样)")
        print("=" * 60)
        for countdown in range(5, 0, -1):
            print(f"  >>> {countdown} 秒后开始采样...")
            sys.stdout.flush()
            time.sleep(1.0)
        print("  >>> 开始采样!")

        # 7. 连续采样
        print()
        print(f"[*] 开始采样 {NUM_SAMPLES} 次 (每次间隔 {SAMPLE_INTERVAL}s)...")
        print()
        header = f"{'#':>3}  {'M1':>4} {'M2':>4} {'M3':>4} {'M4':>4}  {'diff':>6}  {'Roll':>7}"
        print(header)
        print("-" * len(header))

        samples = []

        for i in range(NUM_SAMPLES):
            # 每隔几秒重发 rc 命令防超时
            if i > 0 and i % 5 == 0:
                send_command(ser, "rc 50 0 0 0")

            # 读取 motors 和 att
            motors_resp = send_command(ser, "motors")
            att_resp = send_command(ser, "att")

            motors = parse_motors(motors_resp)
            att = parse_att(att_resp)

            if motors and all(m is not None for m in motors):
                m1, m2, m3, m4 = motors
                left = m1 + m4
                right = m2 + m3
                diff = left - right
            else:
                m1 = m2 = m3 = m4 = diff = None
                left = right = None

            roll = att[0] if att else None

            # 9. 打印每次读数
            if m1 is not None and roll is not None:
                print(f"{i+1:>3}  {m1:>4} {m2:>4} {m3:>4} {m4:>4}  {diff:>6}  {roll:>7.2f}")
            else:
                print(f"{i+1:>3}  ---- ---- ---- ----   ----    ----  [解析失败]")
                if motors_resp:
                    print(f"     motors raw: {motors_resp}")
                if att_resp:
                    print(f"     att raw: {att_resp}")

            samples.append({
                "m1": m1, "m2": m2, "m3": m3, "m4": m4,
                "diff": diff, "roll": roll
            })

            time.sleep(SAMPLE_INTERVAL)

        # 10. 汇总分析
        print()
        print("=" * 60)
        print("  汇总分析")
        print("=" * 60)

        valid_samples = [s for s in samples if s["roll"] is not None and s["diff"] is not None]
        right_tilt = [s for s in valid_samples if s["roll"] > ROLL_THRESHOLD]

        if not right_tilt:
            print(f"  [WARN] 没有采集到 Roll > {ROLL_THRESHOLD} 的有效数据")
            print("  请确保无人机确实向右倾斜了")
        else:
            avg_diff = sum(s["diff"] for s in right_tilt) / len(right_tilt)
            avg_roll = sum(s["roll"] for s in right_tilt) / len(right_tilt)
            print(f"  有效采样点 (Roll > {ROLL_THRESHOLD}): {len(right_tilt)} / {len(valid_samples)}")
            print(f"  平均 Roll 角度: {avg_roll:.2f} deg")
            print(f"  平均 diff (M1+M4 - M2-M3): {avg_diff:.1f}")
            print()

            if avg_diff > 0:
                print("  >>> 结论: 方向正确 (PASS)")
                print("  右倾时左侧电机更快, PID 校正方向正确")
            else:
                print("  >>> 结论: 方向错误 (FAIL)")
                print("  右倾时右侧电机更快, 需要修改混控公式")
                print("  建议: 在混控中翻转 Roll 轴符号")

    finally:
        # 11. 停机
        print()
        print("[*] 发送停机命令 rc 0 0 0 0 ...")
        send_command(ser, "rc 0 0 0 0")
        ser.close()
        print("[*] 串口已关闭, 测试结束")


if __name__ == "__main__":
    main()
