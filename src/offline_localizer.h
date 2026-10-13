#pragma once

#include "bev_matcher.h"
#include "voxel_occupancy.h"
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <memory>
#include <string>
#include <vector>

struct BaselineConfig {
    // 高度带：相对局部地面，只用墙/柱/家具等竖直结构做 2D 全局搜索
    double band_min = 0.3;
    double band_max = 2.0;
    double floor_cell = 1.0;           // 地图局部地面估计格子 [m]
    double floor_search_radius = 3.0;  // 局部地面取邻域最低 [m]
    double ground_tolerance = 0.15;    // 距局部地面此范围内的点视为地面观测；<=0 关闭可行区域限制 [m]
    double ground_dilation = 1.0;      // 可行区域膨胀 [m]
    double query_max_range = 40.0;
    double scan_voxel = 0.2;           // query 高度带点的 2D 体素 [m]
    BevMatcher::Params bev;
    // 3D 预筛
    double occupancy_voxel = 0.2;
    int occupancy_dilate = 1;
    // 3D 精配准与验证
    double gicp_voxel = 0.2;
    double gicp_correspondence = 1.0;
    int gicp_iterations = 30;
    double fine_correspondence = 0.3;
    int fine_iterations = 30;
    double fine_max_range = 0.0;       // 精配准只用该水平半径内的点 [m]，0 表示不限
    double inlier_distance = 0.2;
    int verify_candidates = 4;
    double verify_ratio = 0.9;
    double verify_time_budget_ms = 2000.0;  // 超过后不再验证新候选
};

struct Estimate {
    bool valid = false;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double fitness = 0.0;      // 选中候选的 3D 内点率
    int sc_rank = 0;           // 选中候选在 2D 搜索中的名次（1 起）
    double sc_distance = 0.0;  // 1 - 2D 似然得分
    int gicp_trials = 0;
};

class OfflineLocalizer {
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    explicit OfflineLocalizer(BaselineConfig config);
    bool loadMap(const std::string& path);
    Estimate localize(const Cloud::ConstPtr& query_base) const;
    size_t indexSize() const { return gicp_map_->size(); }

private:
    using Gicp = pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;
    float floorAt(float x, float y) const;
    double inlierRatio(const Cloud& aligned) const;

    BaselineConfig config_;
    Cloud::Ptr gicp_map_{new Cloud};
    pcl::KdTreeFLANN<pcl::PointXYZI> map_tree_;
    std::shared_ptr<Gicp> gicp_;       // 粗配准/候选验证
    std::shared_ptr<Gicp> fine_gicp_;  // 胜出候选的近处精配准
    BevMatcher bev_;
    VoxelOccupancy occupancy_;
    // 局部地面高度栅格
    Eigen::Vector2f floor_origin_ = Eigen::Vector2f::Zero();
    int floor_width_ = 0, floor_height_ = 0;
    std::vector<float> floor_;
};
