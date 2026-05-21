#!/usr/bin/env python3
"""rate_test.py -- 角速度环调参测试（单线程，无锁）"""

import serial, time, re, glob, sys, argparse

parser = argparse.ArgumentParser()
parser.add_argument("--kp", type=float, default=0.20, help="Rate Kp (default: 0.20)")
args = parser.parse_args()
KP = args.kp

PORT = None
for pat in ['/dev/cu.usbmodem11403', '/dev/cu.usbmodem*']:
    cands = sorted(glob.glob(pat))
    if cands:
        PORT = cands[0]; break
if not PORT:
    print("无串口"); sys.exit(1)

BAUD = 460800
PWM_RE = re.compile(r"PWM:\s*M1=(\d+)\s*M2=(\d+)\s*M3=(\d+)\s*M4=(\d+)")

def send_cmd(ser, cmd):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()
    time.sleep(0.05)
    return ser.read(1024).decode("utf-8", errors="replace").strip()

ser = serial.Serial(PORT, BAUD, timeout=0.1)

try:
    # 初始化
    ser.reset_input_buffer(); ser.reset_output_buffer()
    time.sleep(0.3); ser.read(4096)

    # 设 PID 参数
    for cmd in ["pidrst", "pida 0 0 0 0", "pida 1 0 0 0",
                f"pidr 0 {KP*100:.0f} 0 0", f"pidr 1 {KP*100:.0f} 0 0"]:
        r = send_cmd(ser, cmd)
        print(f"  {r}")
    print()

    # 开始：油门清零
    send_cmd(ser, "rc 0 0 0 0")
    time.sleep(0.2)

    print(f"===== Rate Kp = {KP:.2f}  |  油门 55%  |  5s 测试 =====")
    print()

    # 运行约 5 秒 (25 次 x 200ms)
    for i in range(25):
        # 发 rc 保持超时不触发
        send_cmd(ser, "rc 55 0 0 0")
        # 读电机
        resp = send_cmd(ser, "motors")
        m = PWM_RE.search(resp)
        if m:
            m1, m2, m3, m4 = int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))
            L = m1 + m4; R = m2 + m3
            d = L - R
            bar = "<-LEFT" if d > 8 else ("RIGHT->" if d < -8 else "  ---")
            print(f"  #{i+1:2d}: M1={m1:3d} M2={m2:3d} M3={m3:3d} M4={m4:3d}  L-R={d:+4d} {bar}")
        else:
            print(f"  #{i+1:2d}: --")
        time.sleep(0.1)

except Exception as e:
    print(f"\n错误: {e}")

finally:
    # 停机
    try:
        ser.write(b"rc 0 0 0 0\n"); ser.flush()
        time.sleep(0.1); ser.read(256)
    except: pass
    ser.close()
    print("\n电机已停止，测试结束")
