#!/bin/bash
# 捕获 Claude Code 传入的完整 JSON 输入
INPUT=$(cat)
echo "$(date): FULL INPUT: $INPUT" >> /tmp/claude-hook-test.log
echo "$(date): FILE_PATH=$(echo "$INPUT" | jq -r '.tool_input.file_path // empty')" >> /tmp/claude-hook-test.log
