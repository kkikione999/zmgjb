#!/usr/bin/env python3
"""
roll_tune.py -- Roll 单轴 PID 调参工具
无人机固定在 Roll 轴方向，逐轴调参。
"""

import argparse, glob, re, sys, time, threading, serial

BAUD = 460800
PWM_RE = re.compile(r"PWM:\s*M1=(\d+)\s*M2=(\d+)\s*M3=(\d+)\s*M4=(\d+)")
ATT_RE = re.compile(r"Euler:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s+Gyro:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)")
GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"; BOLD = "\033[1m"; CYAN = "\033[96m"; RESET = "\033[0m"


class Drone:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.rc = [0, 0, 0, 0]  # T, R, P, Y (x100)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._rc_loop, daemon=True)
        self._thread.start()

    def _rc_loop(self):
        while not self._stop.is_set():
            with self._lock:
                t, r, p, y = self.rc
            try:
                self.ser.write(f"rc {t} {r} {p} {y}\n".encode())
                self.ser.flush()
            except Exception:
                pass
            try:
                self.ser.read(256)
            except Exception:
                pass
            time.sleep(0.05)

    def send(self, cmd):
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write((cmd.strip() + "\n").encode())
            self.ser.flush()
            time.sleep(0.08)
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
                "roll": float(m.group(1)), "pitch": float(m.group(2)), "yaw": float(m.group(3)),
                "gx": float(m.group(4)), "gy": float(m.group(5)), "gz": float(m.group(6)),
            }
        return None

    def set_rc(self, t, r, p, y):
        with self._lock:
            self.rc = [t, r, p, y]

    def stop(self):
        self.set_rc(0, 0, 0, 0)
        self._stop.set()
        self._thread.join(timeout=1)
        self.ser.close()


def auto_port():
    for pat in ["/dev/cu.usbmodem*", "/dev/cu.usbserial*"]:
        cands = sorted(glob.glob(pat))
        for c in cands:
            return c
    return None


def init_roll_only(d, kp_rate=0.0, kd_rate=0.0, kp_angle=0.0):
    """初始化为 Roll-only 模式，关闭 Pitch/Yaw PID"""
    d.send("pidrst")
    # 关闭所有角度环
    d.send("pida 0 0 0 0")
    d.send("pida 1 0 0 0")
    d.send("pida 2 0 0 0")
    # 关闭 Pitch 和 Yaw 角速度环
    d.send("pidr 1 0 0 0")
    d.send("pidr 2 0 0 0")
    # 设 Roll rate
    kp_x100 = int(kp_rate * 100)
    kd_x100 = int(kd_rate * 100)
    d.send(f"pidr 0 {kp_x100} 0 {kd_x100}")
    # 设 Roll angle（如果有）
    if kp_angle > 0:
        kpa_x100 = int(kp_angle * 100)
        d.send(f"pida 0 {kpa_x100} 0 0")


def cmd_dir(d):
    """Step 1: 方向验证"""
    print(f"\n{BOLD}=== Roll 方向验证 ==={RESET}")
    print("只开 Roll rate Kp=0.30, 50% 油门")
    init_roll_only(d, kp_rate=0.30)
    d.set_rc(50, 0, 0, 0)
    time.sleep(0.5)

    print(f"\n  {YELLOW}请你执行: 向右倾斜无人机约 15° 并保持住{RESET}")
    print("  然后按 Enter 读取电机数据...")
    input()

    # 读 5 次
    print()
    for i in range(5):
        vals = d.motors()
        att_data = d.att()
        if vals:
            m1, m2, m3, m4 = vals
            left = m1 + m4
            right = m2 + m3
            diff = left - right
            roll_str = f"{att_data['roll']:+6.1f}°" if att_data else "  ---"
            bar = f"{GREEN}左侧更快 (正确){RESET}" if diff > 15 else (
                f"{RED}右侧更快 (错误!){RESET}" if diff < -15 else "  差异小")
            print(f"  #{i+1}: M1={m1:4d} M2={m2:4d} M3={m3:4d} M4={m4:4d}"
                  f"  L={left} R={right} L-R={diff:+4d}  Roll={roll_str}  {bar}")
        else:
            print(f"  #{i+1}: 读取失败")
        time.sleep(0.15)

    d.set_rc(0, 0, 0, 0)
    print()
    print("判断: 向右倾斜时 (Roll > 0)")
    print(f"  {GREEN}正确{RESET}: M1(FL)+M4(RL) > M2(FR)+M3(RR) → 左侧加速推回")
    print(f"  {RED}错误{RESET}: 右侧更快 → 会加剧倾斜 → 需修混控公式")


def cmd_scan(d):
    """Step 2: Kp 扫描"""
    print(f"\n{BOLD}=== Roll Rate Kp 扫描 ==={RESET}")
    print("从 Kp=0.10 起逐步增大，每次 5 秒")
    print(f"请你执行: 在每次测试中左右倾斜无人机")
    print(f"当你听到电机{RED}高频震荡(嗡嗡声){RESET}时，输入 's' 停止\n")
    input("按 Enter 开始...")

    kp_values = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.80, 1.00]
    results = []

    for kp in kp_values:
        kp_x100 = int(kp * 100)
        print(f"\n{CYAN}--- Kp = {kp:.2f} (pidr 0 {kp_x100} 0 0) ---{RESET}")
        init_roll_only(d, kp_rate=kp)
        d.set_rc(50, 0, 0, 0)
        time.sleep(0.3)

        max_diff = 0
        diffs = []
        for i in range(25):  # 5 秒 @ 200ms
            vals = d.motors()
            att_data = d.att()
            if vals:
                m1, m2, m3, m4 = vals
                diff = (m1 + m4) - (m2 + m3)
                diffs.append(diff)
                max_diff = max(max_diff, abs(diff))
                roll_str = f"{att_data['roll']:+6.1f}°" if att_data else ""
            time.sleep(0.2)

        d.set_rc(0, 0, 0, 0)
        time.sleep(0.3)

        avg_diff = sum(diffs) / len(diffs) if diffs else 0
        max_d = max(diffs) if diffs else 0
        min_d = min(diffs) if diffs else 0
        results.append((kp, avg_diff, max_d, min_d))
        print(f"  差速: avg={avg_diff:+.0f}, max={max_d:+.0f}, min={min_d:+.0f}")

        ans = input(f"  继续? (Enter=下一个, s=停止): ").strip().lower()
        if ans == 's':
            break

    print(f"\n{BOLD}扫描结果汇总:{RESET}")
    print(f"  {'Kp':>6s}  {'avg':>6s}  {'max':>6s}  {'min':>6s}")
    for kp, avg, mx, mn in results:
        print(f"  {kp:6.2f}  {avg:+6.0f}  {mx:+6.0f}  {mn:+6.0f}")

    if results:
        last_kp = results[-1][0]
        rec = last_kp * 0.65
        print(f"\n  建议起始 Kp = {rec:.2f} (振荡点 × 0.65)")


def cmd_test(d, duration=10):
    """持续读 motors+att，duration 秒"""
    print(f"\n{BOLD}=== 持续监测 {duration}s ==={RESET}")
    print(f"当前油门 = {d.rc[0]}%")
    print(f" {YELLOW}请你执行: 左右倾斜无人机{RESET}\n")

    samples = []
    for i in range(int(duration / 0.2)):
        vals = d.motors()
        att_data = d.att()
        if vals and att_data:
            m1, m2, m3, m4 = vals
            diff = (m1 + m4) - (m2 + m3)
            samples.append((att_data["roll"], diff, m1, m2, m3, m4))
            roll = att_data["roll"]
            bar = "LEFT" if diff > 15 else ("RIGHT" if diff < -15 else "----")
            print(f"  Roll={roll:+6.1f}° L-R={diff:+4d} [{bar:5s}]"
                  f" M1={m1:4d} M2={m2:4d} M3={m3:4d} M4={m4:4d}")
        time.sleep(0.2)

    if samples:
        rolls = [s[0] for s in samples]
        diffs = [s[1] for s in samples]
        print(f"\n  Roll: avg={sum(rolls)/len(rolls):+.1f}°"
              f" range=[{min(rolls):+.1f}°, {max(rolls):+.1f}°]")
        print(f"  L-R: avg={sum(diffs)/len(diffs):+.0f}"
              f" range=[{min(diffs):+.0f}, {max(diffs):+.0f}]")


def cmd_set(d, args_str):
    """设参数: set kp 0.25 / set kd 0.03 / set kpa 2.0"""
    parts = args_str.split()
    if len(parts) != 2:
        print("  用法: set kp <val> | set kd <val> | set kpa <val> | set throttle <val>")
        return

    key, val = parts[0], parts[1]
    try:
        v = float(val)
    except ValueError:
        print(f"  无效值: {val}")
        return

    if key == "kp":
        vx = int(v * 100)
        d.send(f"pidr 0 {vx} 0 0")  # keep current kd=0
        print(f"  → pidr 0 {vx} 0 0  (Kp_rate={v:.2f})")
    elif key == "kd":
        vx = int(v * 100)
        print(f"  → 需要先知道当前 Kp，建议用: pidr 0 <kp_x100> 0 {vx}")
    elif key == "kpa":
        vx = int(v * 100)
        d.send(f"pida 0 {vx} 0 0")
        print(f"  → pida 0 {vx} 0 0  (Kp_angle={v:.2f})")
    elif key == "throttle":
        d.set_rc(int(v), 0, 0, 0)
        print(f"  → 油门 {int(v)}%")
    else:
        print(f"  未知参数: {key}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    args = parser.parse_args()

    port = args.port or auto_port()
    if not port:
        print("[ERROR] 无串口"); sys.exit(1)

    print(f"{BOLD}Roll 单轴调参工具{RESET}  串口: {port}")
    print("无人机固定在 Roll 轴方向，逐轴调参")
    print()

    d = Drone(port)
    time.sleep(0.5)
    d.ser.read(4096)  # 排空启动输出

    # 等 AHRS 稳定
    print("等待 AHRS 稳定 (5s)...")
    time.sleep(5)
    d.ser.read(4096)
    print("就绪。\n")

    print(f"输入命令 (h=帮助, q=退出):")
    print()

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue

        cmd = line.lower()

        if cmd in ("q", "quit", "exit"):
            break
        elif cmd == "h":
            print(f"""
  {BOLD}Roll 单轴调参命令:{RESET}
  dir         Step 1: 方向验证 (Kp=0.30, 倾斜看差速)
  scan        Step 2: Rate Kp 自动扫描
  test [N]    持续读 motors+att N 秒 (默认 10s)
  set kp <V>  设 Roll rate Kp (例: set kp 0.25)
  set kd <V>  设 Roll rate Kd
  set kpa <V> 设 Roll angle Kp (例: set kpa 2.0)
  set throttle <V>  设油门 (例: set throttle 50)
  init [kp] [kd] [kpa]  初始化 Roll-only 模式 (默认全 0)
  pid         显示当前 PID 参数
  att         读一次姿态
  motors      读一次电机
  stop        油门归零
  raw <cmd>   发送原始命令
  h           帮助
  q           退出
""")
        elif cmd == "dir":
            cmd_dir(d)
        elif cmd == "scan":
            cmd_scan(d)
        elif cmd.startswith("test"):
            parts = line.split()
            dur = int(parts[1]) if len(parts) > 1 else 10
            # 确保有油门
            if d.rc[0] == 0:
                d.set_rc(50, 0, 0, 0)
                print("  油门自动设为 50%")
            cmd_test(d, dur)
        elif cmd.startswith("set "):
            cmd_set(d, line[4:])
        elif cmd.startswith("init"):
            parts = line.split()
            kp = float(parts[1]) if len(parts) > 1 else 0
            kd = float(parts[2]) if len(parts) > 2 else 0
            kpa = float(parts[3]) if len(parts) > 3 else 0
            init_roll_only(d, kp_rate=kp, kd_rate=kd, kp_angle=kpa)
            print(f"  Roll-only: Kp_rate={kp:.2f} Kd_rate={kd:.2f} Kp_angle={kpa:.2f}")
        elif cmd == "stop":
            d.set_rc(0, 0, 0, 0)
            print("  已停机")
        elif cmd == "motors":
            vals = d.motors()
            if vals:
                m1, m2, m3, m4 = vals
                print(f"  M1={m1} M2={m2} M3={m3} M4={m4}")
                print(f"  L(M1+M4)={m1+m4} R(M2+M3)={m2+m3} diff={m1+m4-m2-m3:+d}")
            else:
                print("  读取失败")
        elif cmd == "att":
            data = d.att()
            if data:
                print(f"  Roll={data['roll']:+.2f}° Pitch={data['pitch']:+.2f}° Yaw={data['yaw']:+.2f}°")
                print(f"  Gyro: gx={data['gx']:+.1f} gy={data['gy']:+.1f} gz={data['gz']:+.1f}")
            else:
                print("  读取失败")
        elif cmd == "pid":
            resp = d.send("pid")
            for l in resp.split("\r\n"):
                if l.strip():
                    print(f"  {l.strip()}")
        elif cmd.startswith("raw "):
            resp = d.send(line[4:])
            if resp:
                print(f"  {resp}")
        else:
            resp = d.send(line)
            if resp:
                print(f"  {resp}")

    d.stop()
    print("已退出")


if __name__ == "__main__":
    main()
