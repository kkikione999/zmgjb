#!/bin/bash
# flash_monitor.sh — 编译 + 烧录 + 复位 + 串口输出，一步到位
#
# 用法: bash flash_monitor.sh [捕获秒数]
# 默认捕获 6 秒

set -e

DURATION=${1:-6}
SERIAL_PORT="/dev/cu.usbmodem11403"
BAUD=460800
BUILD_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "===== [1/3] 编译 ====="
cd "$BUILD_DIR"
pio run

echo ""
echo "===== [2/3] 烧录 ====="
pio run -t upload

echo ""
echo "===== [3/3] 复位 + 串口捕获 (${DURATION}s) ====="
python3 -c "
import serial, subprocess, time, threading, sys

port = '$SERIAL_PORT'
baud = $BAUD
duration = $DURATION

try:
    ser = serial.Serial(port, baud, timeout=0.5)
except Exception as e:
    print(f'[ERROR] 无法打开 {port}: {e}', file=sys.stderr)
    sys.exit(1)

ser.reset_input_buffer()

# 后台复位 MCU
def reset_mcu():
    subprocess.run(
        ['openocd', '-f', 'interface/stlink.cfg', '-f', 'target/stm32f4x.cfg',
         '-c', 'init; reset run; shutdown'],
        capture_output=True, timeout=15
    )

t = threading.Thread(target=reset_mcu)
t.start()
time.sleep(2)

start = time.time()
while time.time() - start < duration:
    data = ser.read(256)
    if data:
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()

ser.close()
t.join(timeout=10)
"

echo ""
echo "===== 完成 ====="
