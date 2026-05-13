#!/bin/bash
# auto-build.sh — PostToolUse hook: 编辑文件后自动构建
# 输入: JSON (stdin), 包含 tool_input.file_path

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

# 无文件路径则跳过
[ -z "$FILE_PATH" ] && exit 0

case "$FILE_PATH" in
  */C3/*|*/C3)
    echo "[HOOK] 检测到 ESP32 文件变更: $FILE_PATH" >&2
    echo "[HOOK] 开始构建 ESP32..." >&2
    cd /Users/ll/fly/zmgjb/code/C3 && pio run 2>&1 | tail -5 >&2
    ;;
  */411/*|*/411)
    echo "[HOOK] 检测到 STM32 文件变更: $FILE_PATH" >&2
    echo "[HOOK] 开始构建 STM32..." >&2
    cd /Users/ll/fly/zmgjb/code/411 && pio run 2>&1 | tail -5 >&2
    ;;
esac

exit 0
