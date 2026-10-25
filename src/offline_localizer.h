#pragma once

#include "scan_context.h"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <memory>
#include <string>
#include <vector>

struct BaselineConfig {
    ScanContext sc;
    double grid_resolution = 1.0;
    double sensor_height = 0.5;
    double gicp_voxel = 0.25;
    double gicp_correspondence = 1.5;
    int gicp_iterations = 50;
    double query_budget_ms = 2800.0;
    int coarse_candidates = 64;
    int fine_candidates = 3;
    int geometric_candidates = 32;
    double intensity_score_weight = 0.1;
    bool robust_refinement = true;
    bool upright_refinement = false;
    int retrieval_threads = 4;
    double sc_index_voxel = 0.3;
};

struct Estimate {
    bool valid = false;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double fitness = 0.0;
    int sc_rank = 0;
    double sc_distance = 0.0;
    int gicp_trials = 0;
    int coarse_trials = 0;
};

class OfflineLocalizer {
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    explicit OfflineLocalizer(BaselineConfig config);
    bool loadMap(const std::string& path);
    Estimate localize(const Cloud::ConstPtr& query_base) const;
    size_t indexSize() const { return database_.size(); }

private:
    struct Entry {
        Eigen::Vector3f center;
        Eigen::MatrixXf descriptor;
    };
    BaselineConfig config_;
    Cloud::Ptr map_{new Cloud};
    Cloud::Ptr gicp_map_{new Cloud};
    std::vector<Entry> database_;
    Cloud::Ptr coarse_map_{new Cloud};
    Eigen::Vector3f field_origin_ = Eigen::Vector3f::Zero();
    Eigen::Vector3i field_size_ = Eigen::Vector3i::Zero();
    std::vector<float> field_;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr fine_tree_{new pcl::search::KdTree<pcl::PointXYZI>};
    std::shared_ptr<std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>> target_covariances_;
    float intensity_scale_ = 1.0f;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> target_normals_;
};
