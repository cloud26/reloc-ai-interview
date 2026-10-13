# 代码地图

当前代码路径：

1. `src/reloc_benchmark.cpp`：读数据、把 query 从 odom 系转到 base 系、调用定位器，然后用真值评分并写 CSV。
2. `src/bev_matcher.h`：2D 俯视似然场 + 多分辨率分支定界（Cartographer FastCorrelativeScanMatcher 思路）。对全图所有平移 × 离散 yaw 做与穷举等价的搜索，按 (x, y, yaw) 分桶 + NMS 输出彼此分离的候选；可选“可行位置”掩码剪枝和时间上限。
3. `src/voxel_occupancy.h`：地图 3D 占据位图（稠密位集，可膨胀），给大量候选做廉价 3D 一致性打分。
4. `src/offline_localizer.cpp`：
   - 建库：局部地面栅格（每格低分位高度、邻域取最小）→ 高度带 `[band_min, band_max]` 内的结构点建 2D 似然场；观测到地面的格子（膨胀后）作为机器人可行位置；3D 占据位图；GICP 目标协方差预先算好。
   - 查询：RANSAC 拟合 query 地面并调平 → 高度带点 2D 体素化 → 分支定界取 2D 候选 → 每个候选在 3D 占据位图上做 (x, y, yaw, z) 爬山并按命中率重排 → 前几名做粗 GICP，按 0.2 m 内点率选择 → 胜出者只用近处点做一次精 GICP。
5. `config/localizer.yaml`：上述各阶段参数，含分支定界和 GICP 验证的时间预算。

`OfflineLocalizer::localize()` 的输入没有 groundtruth 或 query 文件名。CSV 沿用原列名：`sc_rank` 现在是选中候选在 2D 搜索中的名次，`sc_distance` 是 `1 - 2D 似然得分`，`fitness` 是选中位姿的 3D 内点率（越大越好）。诊断用真值不能进入在线定位逻辑。
