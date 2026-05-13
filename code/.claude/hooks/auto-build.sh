#!/bin/bash
# auto-build.sh — PostToolUse Hook: 文件编辑后自动构建并回传结果
#
# 输入: JSON (stdin), 由 Claude Code 传入
# 输出: JSON (stdout), 通过 additionalContext 让 Claude 看到构建结果

INPUT=$(cat)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')
[ -z "$FILE_PATH" ] && exit 0

BUILD_DIR=""
TARGET=""
case "$FILE_PATH" in
  */C3/*|*/C3) BUILD_DIR="/Users/ll/fly/zmgjb/code/C3"; TARGET="ESP32" ;;
  */411/*|*/411) BUILD_DIR="/Users/ll/fly/zmgjb/code/411"; TARGET="STM32" ;;
esac

[ -z "$BUILD_DIR" ] && exit 0

# 执行构建，捕获完整输出
BUILD_OUTPUT=$(cd "$BUILD_DIR" && pio run 2>&1)
BUILD_EXIT=$?

# 提取关键信息: 最后5行，用 jq 安全转义为 JSON 字符串
TAIL=$(echo "$BUILD_OUTPUT" | tail -5)
FILENAME="${FILE_PATH##*/}"

if [ $BUILD_EXIT -eq 0 ]; then
  STATUS="SUCCESS"
else
  STATUS="FAILED"
fi

MSG="[auto-build] $TARGET $STATUS after editing $FILENAME
$TAIL"

# 用 jq 确保输出合法 JSON
jq -n --arg ctx "$MSG" '{"hookSpecificOutput":{"hookEventName":"PostToolUse","additionalContext":$ctx}}'

exit 0
