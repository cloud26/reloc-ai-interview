#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="${1:-$project_root/../reloc-ai-interview-candidate.tar.gz}"

tar -czf "$output" \
  --exclude='reloc-ai-interview/build' \
  --exclude='reloc-ai-interview/results' \
  --exclude='reloc-ai-interview/.git' \
  --exclude='reloc-ai-interview/data/private' \
  --exclude='reloc-ai-interview/src/localizers' \
  --exclude='reloc-ai-interview/third_party' \
  --exclude='reloc-ai-interview/docs/interviewer.md' \
  --exclude='*.sc.v*' \
  --exclude='*.log' \
  --exclude='.DS_Store' \
  -C "$(dirname "$project_root")" "$(basename "$project_root")"

echo "候选人包: $output"
