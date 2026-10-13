#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_map>
#include <vector>

// 2D 俯视似然场 + 分支定界（Cartographer FastCorrelativeScanMatcher 思路）。
// 对全图所有平移和离散 yaw 做穷举等价的全局搜索，返回彼此分离的若干高分候选。
class BevMatcher {
public:
    struct Params {
        double resolution = 0.1;   // 似然场格子 [m]
        double sigma = 0.2;        // 似然场高斯宽度 [m]
        double yaw_step_deg = 1.0;
        int depth = 6;             // 金字塔层数，根节点覆盖 2^depth 格
        double min_score = 0.3;    // 绝对最低分
        double keep_ratio = 0.8;   // 只收集 >= keep_ratio * best 的叶子
        double nms_distance = 1.0; // 候选去重距离 [m]
        double nms_yaw_deg = 10.0;
        int max_candidates = 8;
        double time_budget_ms = 1500.0;  // 分支定界时间上限，超时返回已找到的最好候选
    };
    struct Candidate {
        double x = 0.0, y = 0.0, yaw = 0.0;  // yaw 弧度
        double score = 0.0;
    };

    // occupied: 地图中“结构”点（高度带内）的 2D 坐标
    // allowed(x, y)：机器人可能所在的位置（如观测到地面处）；为空则全图搜索。
    void build(const std::vector<Eigen::Vector2f>& occupied, const Params& params,
               const std::function<bool(float, float)>& allowed = {}) {
        params_ = params;
        if (occupied.empty()) return;
        Eigen::Vector2f lo = occupied.front(), hi = occupied.front();
        for (const auto& p : occupied) { lo = lo.cwiseMin(p); hi = hi.cwiseMax(p); }
        const float margin = static_cast<float>(4.0 * params.sigma + params.resolution);
        origin_ = lo.array() - margin;
        const double r = params.resolution;
        width_ = static_cast<int>(std::ceil((hi.x() + margin - origin_.x()) / r)) + 1;
        height_ = static_cast<int>(std::ceil((hi.y() + margin - origin_.y()) / r)) + 1;
        const size_t cells = static_cast<size_t>(width_) * height_;

        // 精确欧氏距离变换（Felzenszwalb），再转似然。
        std::vector<double> d2(cells, 1e20);
        for (const auto& p : occupied) {
            const int i = static_cast<int>((p.x() - origin_.x()) / r);
            const int j = static_cast<int>((p.y() - origin_.y()) / r);
            if (i >= 0 && j >= 0 && i < width_ && j < height_) d2[idx(i, j)] = 0.0;
        }
        // 可行位置的 max 金字塔：窗口内没有任何可行位置的节点直接剪掉。
        allowed_.assign(static_cast<size_t>(params.depth) + 1, {});
        allowed_[0].assign(cells, 1);
        if (allowed) {
            for (int i = 0; i < width_; ++i) for (int j = 0; j < height_; ++j)
                allowed_[0][idx(i, j)] = allowed(static_cast<float>(origin_.x() + i * r),
                                                 static_cast<float>(origin_.y() + j * r)) ? 1 : 0;
        }
        for (int d = 1; d <= params.depth; ++d) {
            const int half = 1 << (d - 1);
            const auto& prev = allowed_[d - 1];
            auto& cur = allowed_[d];
            cur.assign(cells, 0);
            for (int i = 0; i < width_; ++i) for (int j = 0; j < height_; ++j) {
                uint8_t m = prev[idx(i, j)];
                if (i + half < width_) m |= prev[idx(i + half, j)];
                if (j + half < height_) m |= prev[idx(i, j + half)];
                if (i + half < width_ && j + half < height_) m |= prev[idx(i + half, j + half)];
                cur[idx(i, j)] = m;
            }
        }
        distanceTransform(d2);
        const double inv = 1.0 / (2.0 * params.sigma * params.sigma);
        levels_.assign(static_cast<size_t>(params.depth) + 1, {});
        levels_[0].resize(cells);
        for (size_t k = 0; k < cells; ++k)
            levels_[0][k] = static_cast<float>(std::exp(-d2[k] * r * r * inv));
        // levels_[d](i,j) = max levels_[0] over [i, i+2^d) x [j, j+2^d)
        for (int d = 1; d <= params.depth; ++d) {
            const int half = 1 << (d - 1);
            const auto& prev = levels_[d - 1];
            auto& cur = levels_[d];
            cur.assign(cells, 0.0f);
            for (int i = 0; i < width_; ++i) for (int j = 0; j < height_; ++j) {
                float m = prev[idx(i, j)];
                if (i + half < width_) m = std::max(m, prev[idx(i + half, j)]);
                if (j + half < height_) m = std::max(m, prev[idx(i, j + half)]);
                if (i + half < width_ && j + half < height_) m = std::max(m, prev[idx(i + half, j + half)]);
                cur[idx(i, j)] = m;
            }
        }
    }

    bool empty() const { return levels_.empty(); }

    // scan: 已调平、已选高度带的 query 点（base 系 2D）
    std::vector<Candidate> search(const std::vector<Eigen::Vector2f>& scan) const {
        std::vector<Candidate> result;
        if (levels_.empty() || scan.empty()) return result;
        const auto start = std::chrono::steady_clock::now();
        const double r = params_.resolution;
        const int yaw_count = std::max(1, static_cast<int>(std::lround(360.0 / params_.yaw_step_deg)));
        // 每个 yaw 下的离散化 scan（格坐标，去重）
        std::vector<std::vector<Eigen::Vector2i>> scans(static_cast<size_t>(yaw_count));
        for (int k = 0; k < yaw_count; ++k) {
            const double yaw = k * 2.0 * M_PI / yaw_count;
            const float c = static_cast<float>(std::cos(yaw)), s = static_cast<float>(std::sin(yaw));
            auto& cells = scans[static_cast<size_t>(k)];
            cells.reserve(scan.size());
            for (const auto& p : scan) {
                cells.emplace_back(static_cast<int>(std::lround((c * p.x() - s * p.y()) / r)),
                                   static_cast<int>(std::lround((s * p.x() + c * p.y()) / r)));
            }
            std::sort(cells.begin(), cells.end(), [](const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
                return a.x() != b.x() ? a.x() < b.x() : a.y() < b.y();
            });
            cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
        }
        const auto score = [&](int yaw, int depth, int tx, int ty) {
            return evaluate(scans[static_cast<size_t>(yaw)], depth, tx, ty);
        };
        std::atomic<long> visited{0};
        std::atomic<bool> timeout{false};

        struct Node { int yaw, x, y, depth; float score; };
        const int top = params_.depth;
        const int step = 1 << top;
        std::vector<Node> roots;
        for (int k = 0; k < yaw_count; ++k)
            for (int x = 0; x < width_; x += step)
                for (int y = 0; y < height_; y += step)
                    if (allowed_[static_cast<size_t>(top)][idx(x, y)]) roots.push_back({k, x, y, top, 0.0f});
        const long root_count = static_cast<long>(roots.size());
#pragma omp parallel for schedule(dynamic, 64)
        for (long n = 0; n < root_count; ++n) {
            auto& root = roots[static_cast<size_t>(n)];
            root.score = score(root.yaw, top, root.x, root.y);
        }
        std::sort(roots.begin(), roots.end(), [](const Node& a, const Node& b) { return a.score > b.score; });

        // 叶子按 (x, y, yaw) 粗桶保留最优，保证候选多样性而不是挤在一个峰上。
        struct Leaf { int yaw, x, y; float score; };
        const int bucket_cells = std::max(1, static_cast<int>(std::lround(0.5 * params_.nms_distance / r)));
        const int bucket_yaws = std::max(1, static_cast<int>(std::lround(0.5 * params_.nms_yaw_deg / params_.yaw_step_deg)));
        std::atomic<float> best{0.0f};
        const auto threshold = [&] {
            return static_cast<float>(std::max(params_.min_score, params_.keep_ratio * best.load(std::memory_order_relaxed)));
        };
        std::unordered_map<int64_t, Leaf> buckets;
        std::atomic<bool> done{false};
#pragma omp parallel
        {
            std::unordered_map<int64_t, Leaf> local;
            std::vector<Node> stack;
#pragma omp for schedule(dynamic, 1)
            for (long n = 0; n < root_count; ++n) {
                if (done.load(std::memory_order_relaxed) || timeout.load(std::memory_order_relaxed)) continue;
                const Node& root = roots[static_cast<size_t>(n)];
                if (root.score < threshold()) { done = true; continue; }
                stack.push_back(root);
                while (!stack.empty()) {
                    if ((++visited & 1023) == 0 &&
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() >
                            params_.time_budget_ms) {
                        timeout = true;
                    }
                    if (timeout.load(std::memory_order_relaxed)) { stack.clear(); break; }
                    const Node node = stack.back();
                    stack.pop_back();
                    if (node.score < threshold()) continue;
                    if (node.depth == 0) {
                        float current = best.load(std::memory_order_relaxed);
                        while (node.score > current && !best.compare_exchange_weak(current, node.score)) {}
                        const int64_t key = (static_cast<int64_t>(node.x / bucket_cells) << 40) ^
                                            (static_cast<int64_t>(node.y / bucket_cells) << 16) ^
                                            static_cast<int64_t>(node.yaw / bucket_yaws);
                        auto it = local.find(key);
                        if (it == local.end()) local.emplace(key, Leaf{node.yaw, node.x, node.y, node.score});
                        else if (node.score > it->second.score) it->second = {node.yaw, node.x, node.y, node.score};
                        continue;
                    }
                    const int d = node.depth - 1;
                    const int half = 1 << d;
                    Node children[4];
                    int count = 0;
                    for (int dx = 0; dx < 2; ++dx) for (int dy = 0; dy < 2; ++dy) {
                        const int x = node.x + dx * half, y = node.y + dy * half;
                        if (x >= width_ || y >= height_ || !allowed_[static_cast<size_t>(d)][idx(x, y)]) continue;
                        children[count++] = {node.yaw, x, y, d, score(node.yaw, d, x, y)};
                    }
                    std::sort(children, children + count, [](const Node& a, const Node& b) { return a.score < b.score; });
                    for (int c = 0; c < count; ++c) if (children[c].score >= threshold()) stack.push_back(children[c]);
                }
            }
#pragma omp critical
            for (const auto& kv : local) {
                auto it = buckets.find(kv.first);
                if (it == buckets.end() || kv.second.score > it->second.score) buckets[kv.first] = kv.second;
            }
        }

        std::vector<Leaf> leaves;
        leaves.reserve(buckets.size());
        for (const auto& kv : buckets) if (kv.second.score >= threshold()) leaves.push_back(kv.second);
        std::sort(leaves.begin(), leaves.end(), [](const Leaf& a, const Leaf& b) { return a.score > b.score; });
        const double yaw_tol = params_.nms_yaw_deg * M_PI / 180.0;
        for (const auto& leaf : leaves) {
            Candidate c;
            c.x = origin_.x() + leaf.x * r;
            c.y = origin_.y() + leaf.y * r;
            c.yaw = leaf.yaw * 2.0 * M_PI / yaw_count;
            c.score = leaf.score;
            bool duplicate = false;
            for (const auto& kept : result) {
                if (std::hypot(kept.x - c.x, kept.y - c.y) < params_.nms_distance &&
                    std::abs(std::remainder(kept.yaw - c.yaw, 2.0 * M_PI)) < yaw_tol) { duplicate = true; break; }
            }
            if (duplicate) continue;
            result.push_back(c);
            if (static_cast<int>(result.size()) >= params_.max_candidates) break;
        }
        return result;
    }

private:
    size_t idx(int i, int j) const { return static_cast<size_t>(i) * height_ + j; }

    // max 金字塔第 depth 层的得分是该节点下所有平移的上界；depth=0 时即精确得分。
    float evaluate(const std::vector<Eigen::Vector2i>& cells, int depth, int tx, int ty) const {
        const auto& grid = levels_[static_cast<size_t>(depth)];
        float sum = 0.0f;
        for (const auto& c : cells) {
            const int i = c.x() + tx, j = c.y() + ty;
            if (i < 0 || j < 0 || i >= width_ || j >= height_) continue;
            sum += grid[idx(i, j)];
        }
        return sum / static_cast<float>(cells.size());
    }

    static void transform1d(const double* f, double* d, int n, std::vector<int>& v, std::vector<double>& z) {
        int k = 0;
        v[0] = 0;
        z[0] = -std::numeric_limits<double>::infinity();
        z[1] = std::numeric_limits<double>::infinity();
        for (int q = 1; q < n; ++q) {
            double s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * (q - v[k]));
            while (s <= z[k]) {
                --k;
                s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * (q - v[k]));
            }
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::infinity();
        }
        k = 0;
        for (int q = 0; q < n; ++q) {
            while (z[k + 1] < q) ++k;
            d[q] = double(q - v[k]) * (q - v[k]) + f[v[k]];
        }
    }

    // 平方距离（格单位）
    void distanceTransform(std::vector<double>& grid) const {
        const int n = std::max(width_, height_);
        std::vector<double> f(n), d(n), z(n + 1);
        std::vector<int> v(n);
        for (int i = 0; i < width_; ++i) {
            for (int j = 0; j < height_; ++j) f[j] = grid[idx(i, j)];
            transform1d(f.data(), d.data(), height_, v, z);
            for (int j = 0; j < height_; ++j) grid[idx(i, j)] = d[j];
        }
        for (int j = 0; j < height_; ++j) {
            for (int i = 0; i < width_; ++i) f[i] = grid[idx(i, j)];
            transform1d(f.data(), d.data(), width_, v, z);
            for (int i = 0; i < width_; ++i) grid[idx(i, j)] = d[i];
        }
    }

    Params params_;
    Eigen::Vector2f origin_ = Eigen::Vector2f::Zero();
    int width_ = 0, height_ = 0;
    std::vector<std::vector<float>> levels_;     // 似然 max 金字塔
    std::vector<std::vector<uint8_t>> allowed_;  // 可行位置的按位或金字塔
};
