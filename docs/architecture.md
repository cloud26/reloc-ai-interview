# 代码地图

当前代码路径：

1. `src/reloc_benchmark.cpp`：读数据、把 query 从 odom 系转到 base 系、调用定位器，然后用真值评分并写 CSV。
2. `src/scan_context.h`：单个高度+强度融合 Scan Context 描述子和旋转匹配。
3. `src/offline_localizer.cpp`：从地图网格视点生成 SC 索引；取 SC Top-K；对每个候选做一次 PCL GICP；选择 fitness 最小的有效结果。
4. `config/localizer.yaml`：基线仅暴露 SC 和 GICP 的必要参数。

`OfflineLocalizer::localize()` 的输入没有 groundtruth 或 query 文件名。若要诊断失败，可用评测器输出的位姿误差、SC rank、fitness 与耗时，但诊断用真值不能进入在线定位逻辑。
