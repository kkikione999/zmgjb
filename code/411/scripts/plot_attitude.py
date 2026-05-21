#!/usr/bin/env python3
"""生成黑白两色的姿态数据图表"""
import csv
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

def load_csv(path):
    t, roll, pitch, yaw, gx, gy, gz = [], [], [], [], [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['time_s']))
            roll.append(float(row['roll_deg']))
            pitch.append(float(row['pitch_deg']))
            yaw.append(float(row['yaw_deg']))
            gx.append(float(row['gyro_x_dps']))
            gy.append(float(row['gyro_y_dps']))
            gz.append(float(row['gyro_z_dps']))
    return np.array(t), np.array(roll), np.array(pitch), np.array(yaw), np.array(gx), np.array(gy), np.array(gz)

def main():
    csv_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'record')
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csvs = sorted([f for f in os.listdir(csv_dir) if f.startswith('attitude_static') and f.endswith('.csv')])
        if not csvs:
            print("未找到 CSV 文件")
            sys.exit(1)
        csv_path = os.path.join(csv_dir, csvs[-1])
    print(f"读取: {csv_path}")

    t, roll, pitch, yaw, gx, gy, gz = load_csv(csv_path)

    plt.rcParams.update({
        'font.size': 10,
        'axes.linewidth': 0.8,
        'lines.linewidth': 0.6,
        'figure.facecolor': 'white',
        'axes.facecolor': 'white',
        'savefig.facecolor': 'white',
        'savefig.dpi': 150,
    })

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True,
                             gridspec_kw={'hspace': 0.15})
    fig.suptitle('Static Attitude Data (30s)', fontsize=12, fontweight='bold', y=0.98)

    # --- Euler angles ---
    ax = axes[0]
    ax.plot(t, roll,  'k-',  alpha=0.9, label='Roll')
    ax.plot(t, pitch, 'k--', alpha=0.9, label='Pitch')
    ax.plot(t, yaw,   'k:',  alpha=0.9, label='Yaw')
    ax.set_ylabel('Angle (deg)')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.8, edgecolor='black')
    ax.grid(True, alpha=0.3, linewidth=0.4)
    ax.set_title('Euler Angles', fontsize=10, pad=4)

    # --- Gyro rates ---
    ax = axes[1]
    ax.plot(t, gx, 'k-',  alpha=0.9, label='Gyro X')
    ax.plot(t, gy, 'k--', alpha=0.9, label='Gyro Y')
    ax.plot(t, gz, 'k:',  alpha=0.9, label='Gyro Z')
    ax.set_ylabel('Rate (deg/s)')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.8, edgecolor='black')
    ax.grid(True, alpha=0.3, linewidth=0.4)
    ax.set_title('Gyroscope Rates', fontsize=10, pad=4)

    # --- Zoomed: Euler angles after convergence (last 20s) ---
    ax = axes[2]
    mask = t >= 10.0
    ax.plot(t[mask], roll[mask],  'k-',  alpha=0.9, label='Roll')
    ax.plot(t[mask], pitch[mask], 'k--', alpha=0.9, label='Pitch')
    ax.plot(t[mask], yaw[mask],   'k:',  alpha=0.9, label='Yaw')
    ax.set_ylabel('Angle (deg)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.8, edgecolor='black')
    ax.grid(True, alpha=0.3, linewidth=0.4)
    ax.set_title('Euler Angles (After Convergence, t>=10s)', fontsize=10, pad=4)

    for ax in axes:
        ax.tick_params(direction='in', length=3)

    fig.align_ylabels(axes)

    out_path = os.path.join(csv_dir, 'attitude_static_charts.png')
    fig.savefig(out_path, bbox_inches='tight', dpi=150)
    print(f"图表已保存: {out_path}")

    # --- 第二张图: 稳定段详细分析 ---
    fig2, axes2 = plt.subplots(2, 1, figsize=(10, 5), sharex=True,
                               gridspec_kw={'hspace': 0.15})
    fig2.suptitle('Steady-State Detail (t>=10s)', fontsize=12, fontweight='bold', y=0.98)

    ax = axes2[0]
    ax.plot(t[mask], roll[mask],  'k-',  alpha=0.8, label='Roll')
    ax.plot(t[mask], pitch[mask], 'k--', alpha=0.8, label='Pitch')
    ax.set_ylabel('Angle (deg)')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.8, edgecolor='black')
    ax.grid(True, alpha=0.3, linewidth=0.4)
    ax.set_title('Roll & Pitch (Steady)', fontsize=10, pad=4)
    # 标注均值线
    ax.axhline(y=np.mean(roll[mask]),  color='gray', linewidth=0.5, linestyle='-.')
    ax.axhline(y=np.mean(pitch[mask]), color='gray', linewidth=0.5, linestyle='-.')

    ax = axes2[1]
    ax.plot(t[mask], gx[mask], 'k-',  alpha=0.8, label='Gyro X')
    ax.plot(t[mask], gy[mask], 'k--', alpha=0.8, label='Gyro Y')
    ax.plot(t[mask], gz[mask], 'k:',  alpha=0.8, label='Gyro Z')
    ax.set_ylabel('Rate (deg/s)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.8, edgecolor='black')
    ax.grid(True, alpha=0.3, linewidth=0.4)
    ax.set_title('Gyroscope Rates (Steady)', fontsize=10, pad=4)
    ax.axhline(y=np.mean(gx[mask]), color='gray', linewidth=0.5, linestyle='-.')
    ax.axhline(y=np.mean(gz[mask]), color='gray', linewidth=0.5, linestyle='-.')

    for ax in axes2:
        ax.tick_params(direction='in', length=3)

    out_path2 = os.path.join(csv_dir, 'attitude_steady_detail.png')
    fig2.savefig(out_path2, bbox_inches='tight', dpi=150)
    print(f"图表已保存: {out_path2}")

if __name__ == '__main__':
    main()
