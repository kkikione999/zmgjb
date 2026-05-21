#!/usr/bin/env python3
"""
采集无人机静止状态下 AHRS 姿态数据 (30s)
用法: python3 capture_attitude.py [--port PORT] [--duration SECONDS] [--output FILE]
"""
import serial
import time
import csv
import sys
import argparse
import os

def find_stm32_port():
    """自动查找 STM32 串口"""
    import glob
    candidates = glob.glob('/dev/cu.usbmodem*')
    for p in candidates:
        # 优先选 460800 对应的端口（通常更长的那组）
        print(f"  发现串口: {p}")
    # 默认返回第一个
    if candidates:
        return candidates[0]
    return None

def main():
    parser = argparse.ArgumentParser(description='采集 STM32 AHRS 姿态数据')
    parser.add_argument('--port', default=None, help='串口设备路径')
    parser.add_argument('--baud', type=int, default=460800, help='波特率')
    parser.add_argument('--duration', type=float, default=30, help='采集时长(秒)')
    parser.add_argument('--output', default=None, help='输出文件路径')
    args = parser.parse_args()

    port = args.port or find_stm32_port()
    if not port:
        print("错误: 未找到串口设备，请用 --port 指定")
        sys.exit(1)

    outdir = os.path.join(os.path.dirname(__file__), '..', '..', 'record')
    outdir = os.path.abspath(outdir)
    os.makedirs(outdir, exist_ok=True)

    outfile = args.output or os.path.join(outdir, f"attitude_static_{int(time.time())}.csv")

    print(f"串口: {port} @ {args.baud}")
    print(f"采集时长: {args.duration}s")
    print(f"输出文件: {outfile}")
    print()

    ser = serial.Serial(port, args.baud, timeout=0.5)
    ser.reset_input_buffer()

    rows = []
    start = time.time()
    sample_count = 0
    drop_count = 0

    print("开始采集... 请保持无人机静止")
    print(f"{'时间':>8s}  {'Roll':>8s}  {'Pitch':>8s}  {'Yaw':>8s}  {'Gx':>8s}  {'Gy':>8s}  {'Gz':>8s}")

    try:
        while time.time() - start < args.duration:
            line = ser.readline()
            if not line:
                continue
            try:
                text = line.decode('utf-8', errors='replace').strip()
            except:
                continue

            if not text.startswith('ATT,'):
                continue

            parts = text.split(',')
            if len(parts) != 7:
                drop_count += 1
                continue

            try:
                roll  = float(parts[1])
                pitch = float(parts[2])
                yaw   = float(parts[3])
                gx    = float(parts[4])
                gy    = float(parts[5])
                gz    = float(parts[6])
            except ValueError:
                drop_count += 1
                continue

            elapsed = time.time() - start
            rows.append([elapsed, roll, pitch, yaw, gx, gy, gz])
            sample_count += 1

            if sample_count % 100 == 0:
                print(f"{elapsed:8.2f}  {roll:8.3f}  {pitch:8.3f}  {yaw:8.3f}  {gx:8.3f}  {gy:8.3f}  {gz:8.3f}")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        ser.close()

    # 写 CSV
    with open(outfile, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'roll_deg', 'pitch_deg', 'yaw_deg', 'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps'])
        writer.writerows(rows)

    duration_actual = time.time() - start
    print(f"\n采集完成:")
    print(f"  采样数: {sample_count}")
    print(f"  实际时长: {duration_actual:.1f}s")
    print(f"  平均频率: {sample_count/duration_actual:.1f} Hz")
    if drop_count:
        print(f"  丢弃帧: {drop_count}")
    print(f"  文件: {outfile}")

    # 统计摘要
    if rows:
        import statistics
        print(f"\n静态数据摘要:")
        for i, name in enumerate(['Roll', 'Pitch', 'Yaw', 'GyroX', 'GyroY', 'GyroZ']):
            vals = [r[i+1] for r in rows]
            mean = statistics.mean(vals)
            stdev = statistics.stdev(vals) if len(vals) > 1 else 0
            print(f"  {name:>6s}: mean={mean:8.3f}  std={stdev:8.4f}  min={min(vals):8.3f}  max={max(vals):8.3f}")

if __name__ == '__main__':
    main()
