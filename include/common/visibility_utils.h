#pragma once

#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace visibility_utils {
// 地图是多次扫描的累积体；从一个虚拟视点建 SC 时只保留角度格里的最近表面。
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr simulateLidarScan(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const Eigen::Vector3f& sensor,
    double angular_resolution_deg,
    double max_range,
    double min_range) {
    using Cloud = pcl::PointCloud<PointT>;
    typename Cloud::Ptr visible(new Cloud);
    if (!cloud || cloud->empty()) return visible;
    const double resolution = angular_resolution_deg * M_PI / 180.0;
    const int azimuth_bins = static_cast<int>(std::ceil(2.0 * M_PI / resolution));
    const int elevation_bins = static_cast<int>(std::ceil(M_PI / resolution));
    std::vector<float> nearest(static_cast<size_t>(azimuth_bins * elevation_bins),
                               std::numeric_limits<float>::infinity());
    std::vector<int> point_index(nearest.size(), -1);
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& point = (*cloud)[i];
        const float dx = point.x - sensor.x();
        const float dy = point.y - sensor.y();
        const float dz = point.z - sensor.z();
        const float range = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!std::isfinite(range) || range < min_range || range > max_range) continue;
        double azimuth = std::atan2(dy, dx);
        if (azimuth < 0.0) azimuth += 2.0 * M_PI;
        const double elevation = std::acos(std::clamp(dz / range, -1.0f, 1.0f));
        const int azimuth_bin = std::clamp(static_cast<int>(azimuth / resolution), 0, azimuth_bins - 1);
        const int elevation_bin = std::clamp(static_cast<int>(elevation / resolution), 0, elevation_bins - 1);
        const int bin = elevation_bin * azimuth_bins + azimuth_bin;
        if (range < nearest[bin]) {
            nearest[bin] = range;
            point_index[bin] = static_cast<int>(i);
        }
    }
    for (const int index : point_index) if (index >= 0) visible->push_back((*cloud)[index]);
    return visible;
}
}  // namespace visibility_utils
