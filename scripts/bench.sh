#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "用法: $0 <包含地图、queries、groundtruth.csv 的数据集目录> [native|docker]" >&2
  exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dataset_dir="$(cd "$1" && pwd)"
engine="${2:-native}"
experiment_id="${RELOC_EXPERIMENT_ID:-baseline}"
config_file="${RELOC_CONFIG:-$project_root/config/localizer.yaml}"
scene="$(basename "$dataset_dir")"
if [[ ! -f "$config_file" ]]; then
  echo "配置文件不存在：$config_file" >&2
  exit 2
fi

if [[ ! "$scene" =~ ^[A-Za-z0-9_-]+$ || ! "$experiment_id" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "场景名和 RELOC_EXPERIMENT_ID 只能包含字母、数字、_、-" >&2
  exit 2
fi
if [[ ! -f "$dataset_dir/groundtruth.csv" || ! -d "$dataset_dir/queries" ]]; then
  echo "数据集缺少 groundtruth.csv 或 queries/：$dataset_dir" >&2
  exit 2
fi

map_files=("$dataset_dir"/*.pcd)
if [[ ${#map_files[@]} -ne 1 || ! -f "${map_files[0]}" ]]; then
  echo "数据集根目录必须恰好有一个地图 .pcd：$dataset_dir" >&2
  exit 2
fi
map_name="$(basename "${map_files[0]}")"
work_dir="$project_root/results/$scene/$experiment_id"
mkdir -p "$work_dir"
if [[ ! -f "$work_dir/groundtruth.csv" ]]; then
  cp "${map_files[0]}" "$dataset_dir/groundtruth.csv" "$work_dir/"
  cp -R "$dataset_dir/queries" "$work_dir/"
fi

if [[ "$engine" == native ]]; then
  "$project_root/build/reloc_benchmark" \
    --map "$work_dir/$map_name" --evalset "$work_dir" \
    --config "$config_file"
elif [[ "$engine" == docker ]]; then
  docker run --rm \
    -v "$project_root/results:/results" \
    -v "$(dirname "$config_file"):/config:ro" \
    reloc-ai-interview:latest \
    --map "/results/$scene/$experiment_id/$map_name" \
    --evalset "/results/$scene/$experiment_id" \
    --config "/config/$(basename "$config_file")"
else
  echo "未知执行环境：$engine（应为 native 或 docker）" >&2
  exit 2
fi

echo "结果: $work_dir/benchmark_result.csv"
