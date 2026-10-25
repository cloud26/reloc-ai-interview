# 代码地图

1. `src/reloc_benchmark.cpp` 读取数据，仅用 odom 位姿把 query 转回 base 系；算法返回后才读取 map 真值计算误差。原始精度 `success` 与时间预算内成功数分别报告。
2. `src/scan_context.h` 构造高度与反射强度融合描述子。列归一化只做一次，矩阵乘法计算各列相似度，再求各循环位移的距离；`yawForShift()` 集中定义位移与角度的符号。
3. `src/offline_localizer.cpp` 加载地图时建立虚拟扫描索引、0.5 m 三维似然场、粗/精地图和精地图 KD-tree，并缓存 GICP 协方差与法向量。
4. 每个 query 并行做 SC 匹配和似然场的各位置、角度匹配。两路候选交错、按位置与角度去重，先粗 ICP 和抽样误差评分，再对前 8 名完整评分。少量优胜候选做 GICP，最后用 Huber 权重的点到平面优化精修。
5. `config/localizer.yaml` 是默认平地配置；`config/se3.yaml` 保留六自由度。时间预算通过阶段截止时间、精配准耗时估计和精修采样上限控制，PCL 单次 `align()` 不支持被该预算中断，因此不构成硬实时保证。
6. `scripts/run_benchmark_suite.py` 顺序运行整套数据并汇总精度、3 秒内成功数、中位数、P95、最大耗时及超时数；`scripts/summarize_benchmarks.py` 按场景和 query 名连接两份原始 CSV，生成逐例对照。

`OfflineLocalizer::localize()` 只接收地图加载后的状态和 base 系点云，不接收文件名、地图名、真值、odom 位姿，也不利用前一例定位结果。

CSV 中 `sc_rank=0` 表示几何召回，对应 `sc_distance=nan`；正数为完整 SC 排名。`coarse_trials` 与 `gicp_trials` 分别记录粗 ICP、精 GICP 次数。`fitness` 是统一的截断距离加反射强度损失，与原始 PCL fitness 的数值不可直接横向比较。额外估计位姿列只用于离线诊断。
