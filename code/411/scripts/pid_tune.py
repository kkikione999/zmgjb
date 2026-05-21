#!/usr/bin/env python3
"""
pid_tune.py -- PID 调参交互工具
后台线程持续发送 rc 命令防止超时，前台交互调参。
"""

import argparse
import glob
import re
import sys
import time
import threading

try:
    import serial
except ImportError:
    print("[ERROR] pip3 install pyserial")
    sys.exit(1)

BAUD = 460800
PWM_RE = re.compile(r"PWM:\s*M1=(\d+)\s*M2=(\d+)\s*M3=(\d+)\s*M4=(\d+)")
GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"; BOLD = "\033[1m"; RESET = "\033[0m"

class Drone:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.rc = [0, 0, 0, 0]  # T, R, P, Y (x100)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        # 后台线程持续发 rc
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
            # 吸掉回显等垃圾
            try:
                self.ser.read(256)
            except Exception:
                pass
            time.sleep(0.05)  # 50ms 间隔，远小于 200ms 超时

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    args = parser.parse_args()

    port = args.port or auto_port()
    if not port:
        print("[ERROR] 无串口"); sys.exit(1)

    print(f"{BOLD}PID 调参工具{RESET}  串口: {port}")
    print("后台持续发 rc 防超时，电机不会断")
    print()

    d = Drone(port)
    time.sleep(0.5)
    d.ser.read(4096)  # 排空启动输出

    # 初始化
    d.send("pidrst")
    d.send("rc 0 0 0 0")
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
            print("""
  rc <T> <R> <P> <Y>    设油门/姿态 (x100, 例: rc 55 0 0 0)
  pidr <a> <p> <i> <d>  设角速度 PID (a:0=roll 1=pitch 2=yaw)
  pida <a> <p> <i> <d>  设角度 PID
  pidrst                重置 PID 状态
  pid                   显示 PID 参数
  att                   显示姿态
  motors                显示电机 PWM
  stop                  油门归零
  sweep                 连续读 motors 10 次
  h                     帮助
  q                     退出
""")
        elif cmd.startswith("rc "):
            parts = line.split()
            if len(parts) == 5:
                d.set_rc(int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
                print(f"  rc → T={parts[1]}% R={parts[2]}% P={parts[3]}% Y={parts[4]}%")
        elif cmd == "stop":
            d.set_rc(0, 0, 0, 0)
            print("  已停机")
        elif cmd == "motors":
            vals = d.motors()
            if vals:
                m1, m2, m3, m4 = vals
                print(f"  PWM: M1={m1} M2={m2} M3={m3} M4={m4}")
                print(f"  左(M1+M4)={m1+m4} 右(M2+M3)={m2+m3}  前(M1+M2)={m1+m2} 后(M3+M4)={m3+m4}")
            else:
                print("  读取失败")
        elif cmd == "att":
            resp = d.send("att")
            print(f"  {resp}")
        elif cmd == "pid":
            resp = d.send("pid")
            for l in resp.split("\\r\\n"):
                if l.strip():
                    print(f"  {l.strip()}")
        elif cmd == "pidrst":
            resp = d.send("pidrst")
            print(f"  {resp}")
        elif cmd.startswith("pidr ") or cmd.startswith("pida "):
            resp = d.send(line)
            print(f"  {resp}")
        elif cmd == "sweep":
            for i in range(10):
                vals = d.motors()
                if vals:
                    print(f"  #{i+1}: M1={vals[0]} M2={vals[1]} M3={vals[2]} M4={vals[3]}")
                time.sleep(0.1)
        else:
            resp = d.send(line)
            if resp:
                print(f"  {resp}")

    d.stop()
    print("已退出")


if __name__ == "__main__":
    main()
