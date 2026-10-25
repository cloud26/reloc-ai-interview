# 3 秒单 query 优化实验

在用户提供的 5 个场景、103 例 query 上，默认配置的原始精度成功数由 46/103 提升到 94/103；同时满足 0.2 m、2° 与 3 秒限制的成功数由 31/103（30.1%）提升到 94/103（91.3%）。优化版 103 例均未超时，最大 query 耗时为 2539.87 ms。

两版使用同一份输入、原始评分阈值和相同 odom→base 转换。时间仅统计 `localize()`，不包含地图/索引加载、PCD 文件读取及坐标系转换，符合原评测器口径。算法不接收真值、文件名或上一例位姿。合肥与安徽地图完全相同，但 query 不同；全部提供数据均参与过迭代，这些结果是已知数据回归成绩，不是独立测试集泛化证据。

| 场景 | 基线原始成功 | 基线 3 秒内成功 | 优化 3 秒内成功 | 优化中位 / P95 / 最大耗时（s） |
|---|---:|---:|---:|---:|
| 合肥 | 20/38 | 20/38 | 37/38 | 0.547 / 0.836 / 0.895 |
| 伏牛 | 3/20 | 0/20 | 19/20 | 1.468 / 2.140 / 2.203 |
| 安徽 | 11/20 | 11/20 | 17/20 | 0.566 / 0.707 / 0.755 |
| 北京 | 7/12 | 0/12 | 12/12 | 1.069 / 1.768 / 2.045 |
| 牛牛 | 5/13 | 0/13 | 9/13 | 2.213 / 2.420 / 2.540 |

环境为 Ubuntu 24.04、Xeon Platinum 8573C（9 个可见 CPU）、GCC 13.3、PCL 1.14.0、Release 编译。默认最多 4 个召回线程，配准单线程，设置 `retrieval_threads: 1` 可检查单线程表现。优化版最终回归顺序执行，期间没有其他 benchmark 或编译进程。基线测量期间部分探索任务曾在其他可用 CPU 核心运行，因此基线耗时与加速倍率仅供参考；不要把这组耗时当成跨机器承诺。

地图/索引加载单独计时，包含精地图协方差与法向量预计算。基线为了适配本机 PCL/VTK 只把 CMake 的语言设置改为 `C CXX`，定位代码保持 `b06cc0b` 不变。

| 场景 | 基线地图/index 加载（s） | 优化地图/index 加载（s） |
|---|---:|---:|
| 合肥 | 19.802 | 3.278 |
| 伏牛 | 48.458 | 8.607 |
| 安徽 | 11.653 | 3.644 |
| 北京 | 39.828 | 10.027 |
| 牛牛 | 145.905 | 35.118 |

主要改动是 SC 列预归一化与矩阵相似度计算；增加独立三维似然场召回；索引用 0.3 m 点云与 1.25 m 视点网格；按位置/朝向去重；粗 ICP 与抽样评分后只完整评分前 8 名；缓存精地图 KD-tree、GICP 协方差和法向量；用预计精配准耗时预留最终精修预算。最终点到平面优化使用 Huber 权重并最多采样 5000 点。

默认精修约束横滚、俯仰为零，以匹配本次近似平地场景。坡道、明显倾斜或需要完整姿态的任务应使用 `config/se3.yaml`。六自由度备用配置也完成了同一 103 例顺序回归，3 秒内成功 89/103（86.4%），全部未超时；两配置在合肥、安徽、北京、牛牛的成功数相同，伏牛由六自由度的 14/20 提升到水平约束的 19/20。证据见 `se3-summary.csv` 与 `se3/`。单次 PCL `align()` 不支持按这里的墙钟预算中断，阶段截止时间与预留预算属于软时间控制；本机全例低于 3 秒不等于硬实时保证。

仍有 9 例失败：5 例 `wrong_place`，4 例 `imprecise`。按 3 秒成绩比较，64 例恢复、1 例退步，其余不变；退步例为安徽 q13，误差从基线阈值内变为约 0.232 m，已明确保留。

| 场景 / query | 位置误差（m） | yaw 误差（°） | 原因 |
|---|---:|---:|---|
| 合肥 / q16.pcd | 14.969 | 90.154 | wrong_place |
| 伏牛 / q08.pcd | 0.202 | 0.528 | imprecise |
| 安徽 / q08.pcd | 1.349 | 0.048 | imprecise |
| 安徽 / q11.pcd | 0.344 | 13.889 | imprecise |
| 安徽 / q13.pcd | 0.232 | 0.178 | imprecise |
| 牛牛 / q03.pcd | 47.987 | 0.289 | wrong_place |
| 牛牛 / q09.pcd | 48.005 | 0.422 | wrong_place |
| 牛牛 / q10.pcd | 42.021 | 0.060 | wrong_place |
| 牛牛 / q13.pcd | 58.331 | 179.562 | wrong_place |

伏牛 q08 仅略超 0.2 m，但仍判失败。合肥 q16 与牛牛剩余失败主要位于重复结构的错误位置；安徽 q11 的姿态变化也未可靠恢复。进一步工作的方向是倾斜条件下的候选初始化、重复结构验证和减少不同扫描间的厘米级偏差，而不是放宽评分阈值。

结果文件保存在 [experiments/3s](experiments/3s)：两版逐例 CSV、`summary.csv`、`per_query_diff.csv`、配置快照和 `metadata.json`。证据 CSV 不包含地图点云、真值坐标或诊断估计坐标。`fitness` 公式已改变，不应直接比较两版 fitness 数值。

复现默认配置：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  python3 scripts/run_benchmark_suite.py /path/to/data/public \
    --experiment-id reproduce-3s
```

复现原始算法（依赖仍需安装，五图数据使用同一份 `/path/to/data/public`）：

```bash
git worktree add /tmp/reloc-baseline b06cc0b
cd /tmp/reloc-baseline
# 当本机 PCL/VTK 依赖 MPI C 目标时，启用 C 编译语言。
sed -i.bak 's/LANGUAGES CXX/LANGUAGES C CXX/' CMakeLists.txt
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
for scene in hefei funiu anhui beijing niuniu; do
  RELOC_EXPERIMENT_ID=reproduce-baseline \
    ./scripts/bench.sh /path/to/data/public/$scene native
done
```

重新汇总已提交的证据：

```bash
python3 scripts/summarize_benchmarks.py \
  --baseline docs/experiments/3s/baseline \
  --optimized docs/experiments/3s/optimized \
  --output /tmp/reloc-comparison
```

校验覆盖 SC 新旧距离公式的一致性、空描述子及旋转位移符号。默认配置完成 103 例完整回归，评分成功标记与原评测器一致；时间预算内成功数另行统计，没有改写原始精度成绩。
