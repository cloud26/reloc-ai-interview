# LiDAR 离线重定位：AI 协作面试项目

这是一个独立于 ROS 和机器人工作区的 C++ 项目。给定地图 PCD 与 query PCD，程序估计每个 query 在地图中的位姿；真值只在评测器中计算误差。

本分支提供 **3 秒单 query 优化版**：预归一化 Scan Context + 三维似然场双路召回 → 候选去重 → 粗 ICP 与抽样评分 → 少量候选完整评分、GICP 和稳健精修。默认最多 4 个检索线程，配准单线程；地图协方差和法向量在加载时缓存。评测成功阈值仍为 0.2 m / 2°。

默认 `config/localizer.yaml` 对最终精修施加水平姿态约束，适用于本次近似平地数据。需要保留俯仰、横滚时使用 `config/se3.yaml`。实测、逐例差异、已知失败与复现方法见 [3 秒实验报告](docs/benchmark-3s.md)。原始 SC + Top-K GICP 基线仍可在提交 `b06cc0b` 复现，原说明见 [公开基线](docs/public-baseline.md)。

## 快速开始

推荐 Docker。依赖已预装在 `intermission/reloc-ai-interview-base:ubuntu22.04-v1` 中；首次需要拉取该基础镜像，之后修改题目代码只需重新编译项目，不再安装 PCL 等系统包：

```bash
git clone https://github.com/cloud26/reloc-ai-interview.git
cd reloc-ai-interview
docker pull intermission/reloc-ai-interview-base:ubuntu22.04-v1
docker build -t reloc-ai-interview:latest .
./scripts/bench.sh data/public/hefei docker
./scripts/bench.sh data/public/funiu docker
```

面试现场建议提前在候选机器执行上面的 `docker pull`。`Dockerfile.base` 记录基础镜像的依赖清单，日常做题无需运行它；候选代码由仓库的 `Dockerfile` 单独编译。基础镜像仅含工具与依赖，不含题目代码或评测数据。

也可在安装了 PCL、Eigen3、yaml-cpp 的 Linux 环境直接构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
./scripts/bench.sh data/public/hefei native
```

macOS/Homebrew 原生构建（本项目不要求 OpenMP）：

```bash
brew install cmake pcl eigen yaml-cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

AppleClang 未配置 OpenMP 时，PCL 可能打印 `OpenMP is not available`；程序会以单线程 GICP 继续运行。需要测并行性能时，请先配置 libomp，避免把构建环境差异当成算法收益。

`bench.sh` 将输入复制到 `results/<场景>/<实验ID>/`，生成 `benchmark_result.csv`。使用不同的实验 ID 保留对照：

```bash
RELOC_EXPERIMENT_ID=idea01 ./scripts/bench.sh data/public/hefei native
```

完整数据目录可顺序跑一遍，脚本会保留每个场景的原始 CSV、日志以及时间分位数：

```bash
python3 scripts/run_benchmark_suite.py /path/to/data/public --experiment-id optimized01
RELOC_CONFIG="$PWD/config/se3.yaml" RELOC_EXPERIMENT_ID=se3 ./scripts/bench.sh data/public/hefei native
ctest --test-dir build --output-on-failure
```

若要比较参数，可把修改后的 YAML 放在独立文件，并用 `RELOC_CONFIG=/path/to/experiment.yaml` 指定；不要覆盖默认配置后把不同实验的结果混在一起。

## 面试任务

请先跑通基线，读逐例结果，然后与 AI 协作提出、实现并验证一个能提高召回质量、端到端准确率或效率的改动。你可以改变数据表示、候选生成、匹配与选择方式；题目不规定技术路线。我们关注你如何发现问题、判断 AI 的建议是否可信、设计公平的 AB 实验，以及是否在另一张公开地图上回归。

不要根据地图名、query 文件名或真值坐标写运行时特判。不要通过调宽成功阈值或改写 CSV 来“提高”成绩。交付代码、实验命令、两张公开图的逐例差异及耗时，并说明仍未解决的失败。

## 数据与评分

每个数据集目录包含一个地图 `*.pcd`、`queries/*.pcd`、`groundtruth.csv`。CSV 每行有 query 文件名、`map→base` 真值和记录时的 `odom→base` 位姿，角度为度。评测器先用记录时的 odom 位姿将 query 转回 base 系，再调用定位算法；算法无法访问 map 真值。

公开集为合肥和伏牛。默认成功阈值是水平位置误差不超过 0.2 m 且 yaw 误差不超过 2°。`wrong_place`、`imprecise`、`reloc_failed` 要分别分析。`success` 保留原始精度成绩，`success_under_budget` 另外报告同时达到精度阈值和 3 秒限制的数量；可用 `--time-budget-ms` 改变时间统计上限。首次地图/index 加载时间与逐 query 时间分开报告，跨机器耗时不能直接比较。

源码入口见 [代码地图](docs/architecture.md)，与机器人生产版的边界见 [提取说明](docs/extraction.md)，本机起点见 [公开基线](docs/public-baseline.md)。真实地图可能包含现场信息，外发前须确认分发权限。
