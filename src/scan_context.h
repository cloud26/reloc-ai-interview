#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// Scan Context：每格融合相对地面高度与反射强度。
struct ScanContext {
    int rings = 40;
    int sectors = 120;
    double radius = 30.0;
    double intensity_weight = 1.0;

    double yawForShift(int shift) const { return -double(shift) * (2.0 * M_PI / sectors); }

    Eigen::MatrixXf describe(const pcl::PointCloud<pcl::PointXYZI>& cloud) const {
        Eigen::MatrixXf result = Eigen::MatrixXf::Zero(rings, sectors);
        Eigen::MatrixXf intensity_sum = Eigen::MatrixXf::Zero(rings, sectors);
        Eigen::MatrixXi intensity_count = Eigen::MatrixXi::Zero(rings, sectors);
        Eigen::MatrixXi occupied = Eigen::MatrixXi::Zero(rings, sectors);
        std::vector<float> heights;
        heights.reserve(cloud.size());
        for (const auto& p : cloud) {
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
                std::hypot(p.x, p.y) <= radius) heights.push_back(p.z);
        }
        if (heights.empty()) return result;
        const size_t p5 = static_cast<size_t>(0.05 * (heights.size() - 1));
        std::nth_element(heights.begin(), heights.begin() + p5, heights.end());
        const float floor = heights[p5];
        for (const auto& p : cloud) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const float range = std::hypot(p.x, p.y);
            if (range > radius) continue;
            const int ring = std::clamp(static_cast<int>(std::ceil(range / radius * rings)) - 1, 0, rings - 1);
            const int sector = std::clamp(static_cast<int>(std::ceil(
                (std::atan2(p.y, p.x) + M_PI) / (2.0 * M_PI) * sectors)) - 1, 0, sectors - 1);
            result(ring, sector) = std::max(result(ring, sector), p.z - floor + 0.001f);
            occupied(ring, sector) = 1;
            if (std::isfinite(p.intensity)) {
                intensity_sum(ring, sector) += std::max(0.0f, p.intensity);
                ++intensity_count(ring, sector);
            }
        }
        std::vector<float> bin_heights, bin_intensities;
        for (int r = 0; r < rings; ++r) for (int c = 0; c < sectors; ++c) {
            if (!occupied(r, c)) continue;
            bin_heights.push_back(result(r, c));
            bin_intensities.push_back(intensity_count(r, c)
                ? intensity_sum(r, c) / intensity_count(r, c) : 0.0f);
        }
        const auto scale = [](std::vector<float>& values) {
            const size_t index = static_cast<size_t>(0.95 * (values.size() - 1));
            std::nth_element(values.begin(), values.begin() + index, values.end());
            return std::max(values[index], 1e-6f);
        };
        const float height_scale = scale(bin_heights);
        const float intensity_scale = scale(bin_intensities);
        for (int r = 0; r < rings; ++r) for (int c = 0; c < sectors; ++c) {
            if (!occupied(r, c)) continue;
            const float intensity = intensity_count(r, c)
                ? intensity_sum(r, c) / intensity_count(r, c) : 0.0f;
            result(r, c) = result(r, c) / height_scale +
                intensity_weight * intensity / intensity_scale + 0.001f;
        }
        return result;
    }

    // Normalize once, then evaluate all cyclic shifts through one matrix product.
    Eigen::MatrixXf normalize(const Eigen::MatrixXf& input) const {
        Eigen::MatrixXf result = input;
        for (int c = 0; c < sectors; ++c) {
            const float norm = result.col(c).norm();
            if (norm > 0) result.col(c) /= norm;
        }
        return result;
    }

    std::pair<double, int> compareNormalized(const Eigen::MatrixXf& query,
                                            const Eigen::MatrixXf& map) const {
        const Eigen::MatrixXf similarities = query.transpose() * map;
        std::vector<bool> qo(sectors), mo(sectors);
        for (int c = 0; c < sectors; ++c) {
            qo[c] = query.col(c).squaredNorm() > 0;
            mo[c] = map.col(c).squaredNorm() > 0;
        }
        double best = 1.0;
        int best_shift = 0;
        for (int shift = 0; shift < sectors; ++shift) {
            double sum = 0;
            int occupied_union = 0;
            for (int c = 0; c < sectors; ++c) {
                const int m = (c - shift + sectors) % sectors;
                occupied_union += qo[c] || mo[m];
                sum += similarities(c, m);
            }
            const double distance = occupied_union ? 1.0 - sum / occupied_union : 1.0;
            if (distance < best) { best = distance; best_shift = shift; }
        }
        return {best, best_shift};
    }

    std::pair<double, int> compare(const Eigen::MatrixXf& query,
                                  const Eigen::MatrixXf& map) const {
        return compareNormalized(normalize(query), normalize(map));
    }
};
