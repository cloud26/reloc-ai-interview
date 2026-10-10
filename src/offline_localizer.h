#pragma once

#include "scan_context.h"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

struct BaselineConfig {
    ScanContext sc;
    double grid_resolution = 1.0;
    double sensor_height = 0.5;
    int sc_top_k = 10;
    double gicp_voxel = 0.25;
    double gicp_correspondence = 1.5;
    int gicp_iterations = 50;
};

struct Estimate {
    bool valid = false;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double fitness = 0.0;
    int sc_rank = 0;
    double sc_distance = 0.0;
    int gicp_trials = 0;
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
};
