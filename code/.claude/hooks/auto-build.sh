#!/bin/bash
# =============================================================
# auto-build.sh — PostToolUse Hook: 文件编辑后自动触发 PlatformIO 构建
# =============================================================
#
# 触发条件: Claude Code 通过 Edit 或 Write 工具修改文件后
# 输入格式: JSON (stdin)，由 Claude Code 自动传入，包含:
#   {
#     "tool_input": {
#       "file_path": "被编辑文件的绝对路径"
#     },
#     "tool_name": "Edit 或 Write",
#     "hook_event_name": "PostToolUse",
#     ...
#   }
#
# 路径匹配规则:
#   - 路径包含 /C3/   → 执行 ESP32 构建 (cd C3 && pio run)
#   - 路径包含 /411/  → 执行 STM32 构建 (cd 411 && pio run)
#   - 其他路径        → 不触发构建，静默退出
#
# 配置位置: .claude/settings.json (项目级，可提交到 git)
# =============================================================

# 从 stdin 读取 Claude Code 传入的 JSON
INPUT=$(cat)

# 提取被编辑文件的路径 (.tool_input.file_path)
FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')

# 如果没有文件路径（例如非文件操作），直接跳过
[ -z "$FILE_PATH" ] && exit 0

# 根据文件路径判断目标构建
case "$FILE_PATH" in
  */C3/*|*/C3)
    # ESP32-C3 文件变更 → 触发 ESP32 构建
    echo "[HOOK] ESP32 文件变更: $FILE_PATH" >&2
    cd /Users/ll/fly/zmgjb/code/C3 && pio run 2>&1 | tail -5 >&2
    ;;
  */411/*|*/411)
    # STM32F411 文件变更 → 触发 STM32 构建
    echo "[HOOK] STM32 文件变更: $FILE_PATH" >&2
    cd /Users/ll/fly/zmgjb/code/411 && pio run 2>&1 | tail -5 >&2
    ;;
esac

exit 0
