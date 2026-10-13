#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

// 地图 3D 占据位图（稠密包围盒），可选膨胀。用于大量候选位姿的廉价 3D 一致性打分。
class VoxelOccupancy {
public:
    void build(const std::vector<Eigen::Vector3f>& points, float voxel, int dilate,
               const Eigen::Vector3f& lo, const Eigen::Vector3f& hi) {
        voxel_ = voxel;
        origin_ = lo;
        size_ = ((hi - lo) / voxel).array().ceil().cast<int>() + 1;
        bits_.assign((static_cast<size_t>(size_.x()) * size_.y() * size_.z() + 63) / 64, 0);
        for (const auto& p : points) {
            const Eigen::Vector3i c = cell(p);
            for (int dx = -dilate; dx <= dilate; ++dx)
                for (int dy = -dilate; dy <= dilate; ++dy)
                    for (int dz = -dilate; dz <= dilate; ++dz) set(c + Eigen::Vector3i(dx, dy, dz));
        }
    }

    bool contains(const Eigen::Vector3f& p) const {
        const Eigen::Vector3i c = cell(p);
        if ((c.array() < 0).any() || (c.array() >= size_.array()).any()) return false;
        const size_t k = index(c);
        return (bits_[k >> 6] >> (k & 63)) & 1;
    }

    bool empty() const { return bits_.empty(); }

private:
    Eigen::Vector3i cell(const Eigen::Vector3f& p) const {
        return ((p - origin_) / voxel_).array().floor().cast<int>();
    }
    size_t index(const Eigen::Vector3i& c) const {
        return (static_cast<size_t>(c.x()) * size_.y() + c.y()) * size_.z() + c.z();
    }
    void set(const Eigen::Vector3i& c) {
        if ((c.array() < 0).any() || (c.array() >= size_.array()).any()) return;
        const size_t k = index(c);
        bits_[k >> 6] |= uint64_t{1} << (k & 63);
    }

    float voxel_ = 0.2f;
    Eigen::Vector3f origin_ = Eigen::Vector3f::Zero();
    Eigen::Vector3i size_ = Eigen::Vector3i::Zero();
    std::vector<uint64_t> bits_;
};
