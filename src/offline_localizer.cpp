#include "offline_localizer.h"
#include "common/visibility_utils.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
OfflineLocalizer::Cloud::Ptr downsample(const OfflineLocalizer::Cloud::ConstPtr& input,
                                         float leaf) {
    OfflineLocalizer::Cloud::Ptr output(new OfflineLocalizer::Cloud);
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(input);
    voxel.filter(*output);
    return output;
}
}

OfflineLocalizer::OfflineLocalizer(BaselineConfig config) : config_(std::move(config)) {}

bool OfflineLocalizer::loadMap(const std::string& path) {
    if (pcl::io::loadPCDFile(path, *map_) < 0 || map_->empty()) return false;
    gicp_map_ = downsample(map_, static_cast<float>(config_.gicp_voxel));
    Eigen::Vector4f min_point, max_point;
    pcl::getMinMax3D(*map_, min_point, max_point);
    const double step = config_.grid_resolution;
    if (step <= 0.0 || config_.sc.radius <= 0.0 || config_.sc.rings < 1 ||
        config_.sc.sectors < 1 || config_.sc_top_k < 1) return false;
    const double start_x = std::ceil(min_point.x() / step) * step;
    const double start_y = std::ceil(min_point.y() / step) * step;
    for (double x = start_x; x <= max_point.x(); x += step) {
        for (double y = start_y; y <= max_point.y(); y += step) {
            pcl::CropBox<pcl::PointXYZI> crop;
            crop.setInputCloud(map_);
            crop.setMin(Eigen::Vector4f(x - config_.sc.radius, y - config_.sc.radius, -100.0f, 1.0f));
            crop.setMax(Eigen::Vector4f(x + config_.sc.radius, y + config_.sc.radius, 100.0f, 1.0f));
            std::vector<int> indices;
            crop.filter(indices);
            if (indices.size() < 100) continue;
            Cloud::Ptr local(new Cloud);
            local->reserve(indices.size());
            std::vector<float> heights;
            heights.reserve(indices.size());
            for (int index : indices) {
                const auto& point = (*map_)[static_cast<size_t>(index)];
                local->push_back(point);
                if (std::isfinite(point.z)) heights.push_back(point.z);
            }
            if (heights.empty()) continue;
            const size_t p5 = static_cast<size_t>(0.05 * (heights.size() - 1));
            std::nth_element(heights.begin(), heights.begin() + p5, heights.end());
            Eigen::Vector3f center(static_cast<float>(x), static_cast<float>(y),
                                   heights[p5] + static_cast<float>(config_.sensor_height));
            // 累积地图含物体背面；虚拟视点只应看到最近表面。
            local = visibility_utils::simulateLidarScan<pcl::PointXYZI>(
                local, center, 1.0, config_.sc.radius, 0.3);
            if (local->size() < 100) continue;
            for (auto& point : *local) {
                point.x -= center.x();
                point.y -= center.y();
            }
            database_.push_back({center, config_.sc.describe(*local)});
        }
    }
    std::cout << "[INDEX] SC viewpoints=" << database_.size()
              << " GICP map points=" << gicp_map_->size() << '\n';
    return !database_.empty();
}

Estimate OfflineLocalizer::localize(const Cloud::ConstPtr& query_base) const {
    Estimate best;
    if (!query_base || query_base->empty() || database_.empty()) return best;
    const Eigen::MatrixXf descriptor = config_.sc.describe(*query_base);
    if (descriptor.isZero()) return best;
    struct Match { double distance; int index; int shift; };
    std::vector<Match> matches;
    matches.reserve(database_.size());
    for (size_t i = 0; i < database_.size(); ++i) {
        auto [distance, shift] = config_.sc.compare(descriptor, database_[i].descriptor);
        matches.push_back({distance, static_cast<int>(i), shift});
    }
    const size_t count = std::min(static_cast<size_t>(config_.sc_top_k), matches.size());
    std::partial_sort(matches.begin(), matches.begin() + count, matches.end(),
                      [](const Match& a, const Match& b) { return a.distance < b.distance; });
    Cloud::Ptr source = downsample(query_base, static_cast<float>(config_.gicp_voxel));
    if (source->size() < 20) return best;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(gicp_map_);
    gicp.setMaximumIterations(config_.gicp_iterations);
    gicp.setMaxCorrespondenceDistance(config_.gicp_correspondence);
    best.fitness = std::numeric_limits<double>::infinity();
    for (size_t rank = 0; rank < count; ++rank) {
        const auto& match = matches[rank];
        const double yaw = -match.shift * (2.0 * M_PI / config_.sc.sectors);
        Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
        initial.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
        initial.block<3, 1>(0, 3) = database_[match.index].center;
        Cloud aligned;
        gicp.align(aligned, initial);
        ++best.gicp_trials;
        if (!gicp.hasConverged() || !gicp.getFinalTransformation().allFinite()) continue;
        const double fitness = gicp.getFitnessScore(config_.gicp_correspondence);
        if (!std::isfinite(fitness) || fitness >= best.fitness) continue;
        best.valid = true;
        best.pose = gicp.getFinalTransformation();
        best.fitness = fitness;
        best.sc_rank = static_cast<int>(rank + 1);
        best.sc_distance = match.distance;
    }
    return best;
}
