#include "offline_localizer.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>
#include <limits>
#include <random>
#include <unordered_map>

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

bool finite(const pcl::PointXYZI& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

int64_t voxelKey(float x, float y, float z, float voxel) {
    const auto c = [voxel](float v) { return static_cast<int64_t>(std::floor(v / voxel)) & 0x1fffffLL; };
    return (c(x) << 42) | (c(y) << 21) | c(z);
}

float percentile(std::vector<float> values, double q) {
    if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
    const size_t k = static_cast<size_t>(q * (values.size() - 1));
    std::nth_element(values.begin(), values.begin() + k, values.end());
    return values[k];
}

// 在 query 低处点上 RANSAC 拟合近水平地面，返回把地面法向转到 +z 的旋转及调平后的地面高度。
struct Leveling {
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    float floor = 0.0f;
};

Leveling levelQuery(const OfflineLocalizer::Cloud& cloud) {
    Leveling result;
    std::vector<float> heights;
    heights.reserve(cloud.size());
    for (const auto& p : cloud) if (finite(p) && std::hypot(p.x, p.y) < 20.0f) heights.push_back(p.z);
    if (heights.size() < 50) return result;
    const float low = percentile(heights, 0.05);
    result.floor = low;
    std::vector<Eigen::Vector3f> pts;
    for (const auto& p : cloud) {
        if (!finite(p)) continue;
        const float r = std::hypot(p.x, p.y);
        if (r > 0.5f && r < 20.0f && p.z < low + 0.5f) pts.emplace_back(p.x, p.y, p.z);
    }
    if (pts.size() < 30) return result;
    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> pick(0, pts.size() - 1);
    const float max_tilt = std::cos(15.0f * static_cast<float>(M_PI) / 180.0f);
    Eigen::Vector3f best_n = Eigen::Vector3f::UnitZ();
    float best_d = -low;
    size_t best_inliers = 0;
    for (int it = 0; it < 200; ++it) {
        const auto& a = pts[pick(rng)];
        const auto& b = pts[pick(rng)];
        const auto& c = pts[pick(rng)];
        Eigen::Vector3f n = (b - a).cross(c - a);
        if (n.norm() < 1e-4f) continue;
        n.normalize();
        if (n.z() < 0) n = -n;
        if (n.z() < max_tilt) continue;
        const float d = -n.dot(a);
        size_t inliers = 0;
        for (const auto& p : pts) inliers += std::abs(n.dot(p) + d) < 0.05f;
        if (inliers > best_inliers) { best_inliers = inliers; best_n = n; best_d = d; }
    }
    if (best_inliers < 30) return result;
    // 最小二乘精修
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    std::vector<Eigen::Vector3f> in;
    for (const auto& p : pts) if (std::abs(best_n.dot(p) + best_d) < 0.05f) { in.push_back(p); mean += p; }
    mean /= static_cast<float>(in.size());
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& p : in) cov += (p - mean) * (p - mean).transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    Eigen::Vector3f n = solver.eigenvectors().col(0);
    if (n.z() < 0) n = -n;
    if (n.z() < max_tilt) n = best_n;
    result.rotation = Eigen::Quaternionf::FromTwoVectors(n, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    result.floor = (result.rotation * mean).z();
    return result;
}
}  // namespace

OfflineLocalizer::OfflineLocalizer(BaselineConfig config) : config_(std::move(config)) {}



float OfflineLocalizer::floorAt(float x, float y) const {
    const int i = std::clamp(static_cast<int>((x - floor_origin_.x()) / config_.floor_cell), 0, floor_width_ - 1);
    const int j = std::clamp(static_cast<int>((y - floor_origin_.y()) / config_.floor_cell), 0, floor_height_ - 1);
    return floor_[static_cast<size_t>(i) * floor_height_ + j];
}

bool OfflineLocalizer::loadMap(const std::string& path) {
    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile(path, *raw) < 0 || raw->empty()) return false;
    Cloud::Ptr map(new Cloud);
    map->reserve(raw->size());
    for (const auto& p : *raw) if (finite(p)) map->push_back(p);
    if (map->size() < 100 || config_.floor_cell <= 0.0 || config_.bev.resolution <= 0.0) return false;

    // 局部地面：每格低分位高度，再取邻域最小（墙/天花板格子借用邻近地面）。
    Eigen::Vector2f lo(map->front().x, map->front().y), hi = lo;
    for (const auto& p : *map) { lo = lo.cwiseMin(Eigen::Vector2f(p.x, p.y)); hi = hi.cwiseMax(Eigen::Vector2f(p.x, p.y)); }
    const float cell = static_cast<float>(config_.floor_cell);
    floor_origin_ = lo;
    floor_width_ = static_cast<int>((hi.x() - lo.x()) / cell) + 1;
    floor_height_ = static_cast<int>((hi.y() - lo.y()) / cell) + 1;
    std::vector<std::vector<float>> cell_z(static_cast<size_t>(floor_width_) * floor_height_);
    std::vector<float> all_z;
    all_z.reserve(map->size());
    for (const auto& p : *map) {
        const int i = static_cast<int>((p.x - lo.x()) / cell), j = static_cast<int>((p.y - lo.y()) / cell);
        cell_z[static_cast<size_t>(i) * floor_height_ + j].push_back(p.z);
        all_z.push_back(p.z);
    }
    const float global_floor = percentile(all_z, 0.03);
    std::vector<float> cell_low(cell_z.size(), std::numeric_limits<float>::quiet_NaN());
    for (size_t k = 0; k < cell_z.size(); ++k) if (cell_z[k].size() >= 3) cell_low[k] = percentile(cell_z[k], 0.05);
    floor_.assign(cell_z.size(), global_floor);
    const int reach = static_cast<int>(std::ceil(config_.floor_search_radius / config_.floor_cell));
    for (int i = 0; i < floor_width_; ++i) for (int j = 0; j < floor_height_; ++j) {
        float m = std::numeric_limits<float>::infinity();
        for (int di = -reach; di <= reach; ++di) for (int dj = -reach; dj <= reach; ++dj) {
            if (di * di + dj * dj > reach * reach) continue;
            const int a = i + di, b = j + dj;
            if (a < 0 || b < 0 || a >= floor_width_ || b >= floor_height_) continue;
            const float v = cell_low[static_cast<size_t>(a) * floor_height_ + b];
            if (std::isfinite(v)) m = std::min(m, v);
        }
        if (std::isfinite(m)) floor_[static_cast<size_t>(i) * floor_height_ + j] = m;
    }

    std::vector<Eigen::Vector2f> structure;
    for (const auto& p : *map) {
        const float h = p.z - floorAt(p.x, p.y);
        if (h >= config_.band_min && h <= config_.band_max) structure.emplace_back(p.x, p.y);
    }
    if (structure.size() < 100) return false;
    // 地面机器人只可能出现在观测到地面的位置附近：地面格膨胀后作为搜索区域。
    std::vector<uint8_t> ground(floor_.size(), 0);
    for (const auto& p : *map) {
        const int i = static_cast<int>((p.x - lo.x()) / cell), j = static_cast<int>((p.y - lo.y()) / cell);
        const size_t k = static_cast<size_t>(i) * floor_height_ + j;
        if (std::abs(p.z - floor_[k]) < config_.ground_tolerance) ground[k] = 1;
    }
    std::vector<uint8_t> drivable(floor_.size(), 0);
    const int grow = static_cast<int>(std::ceil(config_.ground_dilation / config_.floor_cell));
    for (int i = 0; i < floor_width_; ++i) for (int j = 0; j < floor_height_; ++j) {
        if (!ground[static_cast<size_t>(i) * floor_height_ + j]) continue;
        for (int di = -grow; di <= grow; ++di) for (int dj = -grow; dj <= grow; ++dj) {
            const int a = i + di, b = j + dj;
            if (a >= 0 && b >= 0 && a < floor_width_ && b < floor_height_) drivable[static_cast<size_t>(a) * floor_height_ + b] = 1;
        }
    }
    const auto allowed = [&](float x, float y) {
        const int i = static_cast<int>(std::floor((x - floor_origin_.x()) / cell));
        const int j = static_cast<int>(std::floor((y - floor_origin_.y()) / cell));
        if (i < 0 || j < 0 || i >= floor_width_ || j >= floor_height_) return false;
        return drivable[static_cast<size_t>(i) * floor_height_ + j] != 0;
    };
    bev_.build(structure, config_.bev, config_.ground_tolerance > 0.0 ? allowed : std::function<bool(float, float)>{});

    {
        std::vector<Eigen::Vector3f> points;
        points.reserve(map->size());
        for (const auto& p : *map) points.emplace_back(p.x, p.y, p.z);
        const float z_lo = percentile(all_z, 0.005) - 1.0f, z_hi = percentile(all_z, 0.995) + 1.0f;
        occupancy_.build(points, static_cast<float>(config_.occupancy_voxel), config_.occupancy_dilate,
                         Eigen::Vector3f(lo.x() - 1.0f, lo.y() - 1.0f, z_lo),
                         Eigen::Vector3f(hi.x() + 1.0f, hi.y() + 1.0f, z_hi));
    }
    gicp_map_ = downsample(map, static_cast<float>(config_.gicp_voxel));
    map_tree_.setInputCloud(gicp_map_);
    // 预热：在建库阶段算好目标协方差和 kd 树，查询时不再重复。
    Cloud::Ptr warm(new Cloud);
    for (size_t i = 0; i < std::min<size_t>(200, gicp_map_->size()); ++i) warm->push_back((*gicp_map_)[i]);
    for (auto* gicp : {&gicp_, &fine_gicp_}) {
        *gicp = std::make_shared<Gicp>();
        (*gicp)->setInputTarget(gicp_map_);
        (*gicp)->setInputSource(warm);
        (*gicp)->setMaximumIterations(1);
        Cloud ignored;
        (*gicp)->align(ignored);
    }
    std::cout << "[INDEX] BEV structure points=" << structure.size()
              << " GICP map points=" << gicp_map_->size() << '\n';
    return true;
}

double OfflineLocalizer::inlierRatio(const Cloud& aligned) const {
    if (aligned.empty()) return 0.0;
    std::vector<int> index(1);
    std::vector<float> d2(1);
    const float limit = static_cast<float>(config_.inlier_distance * config_.inlier_distance);
    size_t inliers = 0;
    for (const auto& p : aligned)
        if (map_tree_.nearestKSearch(p, 1, index, d2) > 0 && d2[0] <= limit) ++inliers;
    return static_cast<double>(inliers) / aligned.size();
}

Estimate OfflineLocalizer::localize(const Cloud::ConstPtr& query_base) const {
    const auto start = std::chrono::steady_clock::now();
    Estimate best;
    if (!query_base || query_base->empty() || bev_.empty() || !gicp_) return best;
    Cloud::Ptr query(new Cloud);
    query->reserve(query_base->size());
    const float max_range = static_cast<float>(config_.query_max_range);
    for (const auto& p : *query_base)
        if (finite(p) && std::hypot(p.x, p.y) <= max_range) query->push_back(p);
    if (query->size() < 50) return best;

    const Leveling level = levelQuery(*query);
    // 高度带点在 2D 体素内取均值：近处密集点不再主导得分，搜索也更快。
    std::unordered_map<int64_t, std::pair<Eigen::Vector2f, int>> cells;
    const float scan_voxel = static_cast<float>(config_.scan_voxel);
    for (const auto& p : *query) {
        const Eigen::Vector3f q = level.rotation * Eigen::Vector3f(p.x, p.y, p.z);
        const float h = q.z() - level.floor;
        if (h < config_.band_min || h > config_.band_max) continue;
        auto it = cells.try_emplace(voxelKey(q.x(), q.y(), 0.0f, scan_voxel), Eigen::Vector2f::Zero(), 0).first;
        it->second.first += q.head<2>();
        ++it->second.second;
    }
    std::vector<Eigen::Vector2f> scan;
    scan.reserve(cells.size());
    for (const auto& kv : cells) scan.push_back(kv.second.first / static_cast<float>(kv.second.second));
    if (scan.size() < 20) return best;
    const auto candidates = bev_.search(scan);
    if (candidates.empty()) return best;

    const auto initialPose = [&](double x, double y, double yaw) {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix() * level.rotation;
        pose.block<3, 1>(0, 3) = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y),
                                                 floorAt(static_cast<float>(x), static_cast<float>(y)) - level.floor);
        return pose;
    };

    // 3D 预筛：2D 结构带在重复场景（长走廊、温室）里区分度不足；用地面以上全部 3D 点
    // 在占据位图中的命中率对所有 2D 候选重排，并局部爬山补偿 2D 峰值的偏差。
    std::vector<Eigen::Vector3f> samples;
    {
        std::unordered_map<int64_t, Eigen::Vector3f> voxels;
        const float v = static_cast<float>(config_.occupancy_voxel * 1.5);
        for (const auto& p : *query) {
            const Eigen::Vector3f q = level.rotation * Eigen::Vector3f(p.x, p.y, p.z);
            if (q.z() - level.floor < config_.band_min) continue;
            voxels.try_emplace(voxelKey(q.x(), q.y(), q.z(), v), p.getVector3fMap());
        }
        samples.reserve(voxels.size());
        for (const auto& kv : voxels) samples.push_back(kv.second);
    }
    struct Ranked { size_t rank; double score3d; Eigen::Matrix4f pose; };
    std::vector<Ranked> ranked(candidates.size());
    const double step = config_.bev.resolution;
    const double yaw_step = config_.bev.yaw_step_deg * M_PI / 180.0;
    const long candidate_count = static_cast<long>(candidates.size());
#pragma omp parallel for schedule(dynamic, 1)
    for (long n = 0; n < candidate_count; ++n) {
        const auto& c = candidates[static_cast<size_t>(n)];
        const auto evaluate = [&](const Eigen::Vector4d& v) {
            Eigen::Matrix4f pose = initialPose(v[0], v[1], v[2]);
            pose(2, 3) += static_cast<float>(v[3]);
            const Eigen::Matrix3f rot = pose.block<3, 3>(0, 0);
            const Eigen::Vector3f t = pose.block<3, 1>(0, 3);
            size_t hits = 0;
            for (const auto& p : samples) hits += occupancy_.contains(rot * p + t);
            return samples.empty() ? 0.0 : static_cast<double>(hits) / samples.size();
        };
        // 坐标下降爬山 (x, y, yaw, dz)，步长逐级减半，补偿 2D 峰值与真实 3D 对齐之间的偏差。
        Eigen::Vector4d v(c.x, c.y, c.yaw, 0.0);
        double score = evaluate(v);
        Eigen::Vector4d delta(2.0 * step, 2.0 * step, yaw_step, config_.occupancy_voxel);
        for (int level_iter = 0; level_iter < 3; ++level_iter, delta *= 0.5) {
            for (bool improved = true; improved;) {
                improved = false;
                for (int d = 0; d < 4; ++d) for (int sign = -1; sign <= 1; sign += 2) {
                    Eigen::Vector4d trial = v;
                    trial[d] += sign * delta[d];
                    const double s = evaluate(trial);
                    if (s > score) { score = s; v = trial; improved = true; }
                }
            }
        }
        Ranked r{static_cast<size_t>(n), score, initialPose(v[0], v[1], v[2])};
        r.pose(2, 3) += static_cast<float>(v[3]);
        ranked[static_cast<size_t>(n)] = r;
    }
    std::stable_sort(ranked.begin(), ranked.end(), [](const Ranked& a, const Ranked& b) { return a.score3d > b.score3d; });

    Cloud::Ptr source = downsample(query, static_cast<float>(config_.gicp_voxel));
    if (source->size() < 20) return best;
    gicp_->setInputSource(source);
    gicp_->setMaxCorrespondenceDistance(config_.gicp_correspondence);
    gicp_->setMaximumIterations(config_.gicp_iterations);
    // 3D 验证：3D 预筛靠前的候选各做一次粗 GICP，用对齐后的内点率挑选。
    double best_inliers = -1.0;
    const size_t count = std::min<size_t>(ranked.size(), static_cast<size_t>(std::max(1, config_.verify_candidates)));
    for (size_t k = 0; k < count; ++k) {
        const auto& r = ranked[k];
        // 预筛得分明显落后的候选不值得做 GICP；超出时间预算时只保留已验证结果。
        if (r.score3d < config_.verify_ratio * ranked.front().score3d) break;
        if (k > 0 && std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() >
                         config_.verify_time_budget_ms) break;
        Cloud aligned;
        gicp_->align(aligned, r.pose);
        ++best.gicp_trials;
        if (!gicp_->hasConverged() || !gicp_->getFinalTransformation().allFinite()) continue;
        const double inliers = inlierRatio(aligned);
        if (inliers <= best_inliers) continue;
        best_inliers = inliers;
        best.valid = true;
        best.pose = gicp_->getFinalTransformation();
        best.fitness = inliers;
        best.sc_rank = static_cast<int>(r.rank + 1);
        best.sc_distance = 1.0 - candidates[r.rank].score;
    }
    if (!best.valid || config_.fine_iterations <= 0) return best;

    // 精配准只对胜出候选做一次，且只用近处点：累积地图远处的漂移/弯曲不应拉偏局部位姿。
    Cloud::Ptr near(new Cloud);
    const float fine_range = static_cast<float>(config_.fine_max_range);
    for (const auto& p : *source)
        if (fine_range <= 0.0f || std::hypot(p.x, p.y) <= fine_range) near->push_back(p);
    if (near->size() < 100) near = source;
    fine_gicp_->setInputSource(near);
    fine_gicp_->setMaxCorrespondenceDistance(config_.fine_correspondence);
    fine_gicp_->setMaximumIterations(config_.fine_iterations);
    Cloud aligned;
    fine_gicp_->align(aligned, best.pose);
    ++best.gicp_trials;
    if (fine_gicp_->hasConverged() && fine_gicp_->getFinalTransformation().allFinite()) {
        const Eigen::Matrix4f delta = best.pose.inverse() * fine_gicp_->getFinalTransformation();
        // 精修只应是小幅修正；跳得太远说明近处结构退化，保留粗结果。
        if (delta.block<2, 1>(0, 3).norm() < config_.fine_correspondence * 2.0) best.pose = fine_gicp_->getFinalTransformation();
    }
    return best;
}
