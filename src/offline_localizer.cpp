#include "offline_localizer.h"
#include "common/visibility_utils.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
template<class Function>
void parallelFor(size_t count, int workers, Function function) {
    workers=std::max(1,std::min(workers,int(count)));
    std::vector<std::thread> threads;
    for(int worker=1;worker<workers;++worker) threads.emplace_back([&,worker]() {
        for(size_t i=worker;i<count;i+=workers) function(i);
    });
    for(size_t i=0;i<count;i+=workers) function(i);
    for(auto& thread:threads) thread.join();
}

class CachedGICP : public pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> {
public:
    MatricesVectorPtr covariances(const OfflineLocalizer::Cloud::ConstPtr& cloud,
                                  const pcl::search::KdTree<pcl::PointXYZI>::Ptr& tree) {
        MatricesVectorPtr result(new MatricesVector);
        computeCovariances<pcl::PointXYZI>(cloud, tree, *result);
        return result;
    }
};
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
    if (!std::isfinite(config_.grid_resolution) || config_.grid_resolution <= 0 ||
        !std::isfinite(config_.sc.radius) || config_.sc.radius <= 0 ||
        !std::isfinite(config_.gicp_voxel) || config_.gicp_voxel <= 0 ||
        !std::isfinite(config_.query_budget_ms) || config_.query_budget_ms <= 0 ||
        config_.sc.rings < 1 || config_.sc.sectors < 18 || config_.coarse_candidates < 1 ||
        config_.fine_candidates < 1 || config_.geometric_candidates < 0 || config_.gicp_iterations < 1 ||
        config_.retrieval_threads < 1 || config_.retrieval_threads > 64)
        return false;
    if (pcl::io::loadPCDFile(path, *map_) < 0 || map_->size() < 20) return false;
    database_.clear();
    coarse_map_ = downsample(map_, 0.6f);
    gicp_map_ = downsample(map_, static_cast<float>(config_.gicp_voxel));
    fine_tree_->setInputCloud(gicp_map_);
    CachedGICP cache;
    target_covariances_ = cache.covariances(gicp_map_, fine_tree_);
    target_normals_.clear();
    for (const auto& covariance : *target_covariances_) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        target_normals_.push_back(solver.eigenvectors().col(0));
    }
    std::vector<float> intensities;
    for (const auto& p : *map_) if (std::isfinite(p.intensity)) intensities.push_back(p.intensity);
    if (!intensities.empty()) {
        const size_t p95 = (intensities.size()-1)*0.95;
        std::nth_element(intensities.begin(), intensities.begin()+p95, intensities.end());
        intensity_scale_ = std::max(intensities[p95], 1.0f);
    }
    Eigen::Vector4f min_point, max_point;
    pcl::getMinMax3D(*map_, min_point, max_point);
    // A bounded 3D likelihood field supplies an independent, descriptor-free recall path.
    field_origin_ = min_point.head<3>() - Eigen::Vector3f::Ones();
    field_size_ = ((max_point.head<3>() - field_origin_) * 2).cast<int>() + Eigen::Vector3i::Constant(3);
    const size_t cells = size_t(field_size_.x()) * field_size_.y() * field_size_.z();
    field_.clear();
    if (config_.geometric_candidates > 0 && cells < 20000000) {
        field_.assign(cells, 0.0f);
        for (const auto& p : *gicp_map_) {
            const Eigen::Vector3i v = ((p.getVector3fMap() - field_origin_) * 2).cast<int>();
            for (int dz=-2; dz<=2; ++dz) for (int dy=-2; dy<=2; ++dy) for (int dx=-2; dx<=2; ++dx) {
                const int x=v.x()+dx, y=v.y()+dy, z=v.z()+dz;
                if (x<0 || y<0 || z<0 || x>=field_size_.x() || y>=field_size_.y() || z>=field_size_.z()) continue;
                const size_t index = (size_t(z)*field_size_.y()+y)*field_size_.x()+x;
                field_[index] = std::max(field_[index], float(std::exp(-0.25*(dx*dx+dy*dy+dz*dz))));
            }
        }
    }
    const double step = config_.grid_resolution;
    if (step <= 0.0 || config_.sc.radius <= 0.0 || config_.sc.rings < 1 ||
        config_.sc.sectors < 1 || config_.coarse_candidates < 1) return false;
    const Cloud::Ptr sc_map = config_.sc_index_voxel > 0 ? downsample(map_, float(config_.sc_index_voxel)) : map_;
    const double start_x = std::ceil(min_point.x() / step) * step;
    const double start_y = std::ceil(min_point.y() / step) * step;
    for (double x = start_x; x <= max_point.x(); x += step) {
        for (double y = start_y; y <= max_point.y(); y += step) {
            pcl::CropBox<pcl::PointXYZI> crop;
            crop.setInputCloud(sc_map);
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
                const auto& point = (*sc_map)[static_cast<size_t>(index)];
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
            database_.push_back({center, config_.sc.normalize(config_.sc.describe(*local))});
        }
    }
    std::cout << "[INDEX] SC viewpoints=" << database_.size()
              << " GICP map points=" << gicp_map_->size() << '\n';
    return !database_.empty();
}

Estimate OfflineLocalizer::localize(const Cloud::ConstPtr& query_base) const {
    const auto start = std::chrono::steady_clock::now();
    const auto elapsed = [&]() { return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count(); };
    Estimate best;
    if (!query_base || query_base->empty() || database_.empty()) return best;
    const Eigen::MatrixXf descriptor = config_.sc.normalize(config_.sc.describe(*query_base));
    if (descriptor.isZero()) return best;
    struct Match { double distance; int index; int shift; int rank = 0; };
    std::vector<Match> matches;
    matches.resize(database_.size());
    parallelFor(database_.size(),config_.retrieval_threads,[&](size_t i) {
        auto [distance, shift] = config_.sc.compareNormalized(descriptor, database_[i].descriptor);
        matches[i]={distance,static_cast<int>(i),shift};
    });
    std::sort(matches.begin(), matches.end(),
              [](const Match& a, const Match& b) { return a.distance < b.distance; });
    for (size_t i=0;i<matches.size();++i) matches[i].rank=int(i+1);
    Cloud::Ptr source = downsample(query_base, query_base->size() < 20000 ? 0.6f : 1.0f);
    if (source->size() < 20) return best;
    std::vector<Match> geometric;
    if (!field_.empty()) {
        std::vector<float> heights;
        for (const auto& p : *source) heights.push_back(p.z);
        const size_t low = heights.size()/20;
        std::nth_element(heights.begin(), heights.begin()+low, heights.end());
        // Balance near, middle and far returns. Dense nearby walls must not drown out
        // the sparse distant structure that separates repeated rooms/corridors.
        std::vector<Eigen::Vector3f> bands[3], points;
        for (const auto& p : *source) {
            const float range=std::hypot(p.x,p.y);
            if (p.z > heights[low]+0.3f && p.z < heights[low]+8.0f && range<60)
                bands[range<5 ? 0 : range<12 ? 1 : 2].push_back(p.getVector3fMap());
        }
        for(const auto& band:bands) {
            const size_t count=std::min(size_t(32),band.size());
            for(size_t i=0;i<count;++i) points.push_back(band[i*band.size()/count]);
        }
        if (points.size()>=20) {
            std::vector<Match> raw(database_.size()*config_.sc.sectors);
            parallelFor(config_.sc.sectors,config_.retrieval_threads,[&](size_t shift) {
                const float angle=config_.sc.yawForShift(static_cast<int>(shift));
                const Eigen::Matrix3f rotation=Eigen::AngleAxisf(angle,Eigen::Vector3f::UnitZ()).toRotationMatrix();
                std::vector<Eigen::Vector3f> rotated;
                for(const auto& p:points) rotated.push_back(rotation*p*2);
                for(size_t i=0;i<database_.size();++i) {
                    const Eigen::Vector3f center=(database_[i].center-field_origin_)*2;
                    float sum=0;
                    for(const auto& p:rotated) {
                        const Eigen::Vector3f v=p+center;
                        if(v.x()<0 || v.y()<0 || v.z()<0) continue;
                        const int x=int(v.x()), y=int(v.y()), z=int(v.z());
                        if(x>=field_size_.x() || y>=field_size_.y() || z>=field_size_.z()) continue;
                        sum+=field_[(size_t(z)*field_size_.y()+y)*field_size_.x()+x];
                    }
                    raw[shift*database_.size()+i]={1.0-sum/points.size(),int(i),int(shift)};
                }
            });
            const size_t shortlist=std::min(raw.size(),size_t(8192));
            std::partial_sort(raw.begin(),raw.begin()+shortlist,raw.end(),
                [](const Match& a,const Match& b){return a.distance<b.distance;});
            raw.resize(shortlist);
            for(const auto& m:raw) {
                bool duplicate=false;
                for(const auto& n:geometric) {
                    int delta=std::abs(m.shift-n.shift);
                    if((database_[m.index].center-database_[n.index].center).head<2>().norm()<2.0 &&
                       std::min(delta,config_.sc.sectors-delta)<config_.sc.sectors/18) {duplicate=true;break;}
                }
                if(!duplicate) geometric.push_back(m);
                if(int(geometric.size())>=config_.geometric_candidates) break;
            }
        }
    }
    // Interleave independent geometric and SC proposals; neither score selects final poses.
    std::vector<Match> proposals;
    size_t sc_i=0;
    std::vector<Match> sc_selected;
    while (sc_i<matches.size() && int(sc_selected.size())<config_.coarse_candidates) {
        const auto m=matches[sc_i++]; bool duplicate=false;
        for(const auto& n:sc_selected) {
            int delta=std::abs(m.shift-n.shift);
            if((database_[m.index].center-database_[n.index].center).head<2>().norm()<1.8 &&
               std::min(delta,config_.sc.sectors-delta)<config_.sc.sectors/12) {duplicate=true;break;}
        }
        if(!duplicate) sc_selected.push_back(m);
    }
    for(size_t i=0;i<std::max(sc_selected.size(),geometric.size());++i) {
        if(i<geometric.size()) proposals.push_back(geometric[i]);
        if(i<sc_selected.size()) proposals.push_back(sc_selected[i]);
    }
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(source);
    icp.setInputTarget(coarse_map_);
    icp.setMaximumIterations(10);
    icp.setMaxCorrespondenceDistance(2.0);
    pcl::KdTreeFLANN<pcl::PointXYZI> tree;
    tree.setInputCloud(gicp_map_);
    const Cloud::Ptr scoring_source = downsample(query_base, 0.4f);
    const auto score = [&](const Eigen::Matrix4f& transform, bool coarse=false) {
        // Capped loss includes EVERY source point: unmatched points cannot improve fitness.
        double loss = 0;
        std::vector<int> indices(5);
        std::vector<float> distances(5);
        const size_t count=std::min(scoring_source->size(),size_t(coarse ? 192 : 1500));
        for (size_t i=0;i<count;++i) {
            const auto& p=(*scoring_source)[i*scoring_source->size()/count];
            const Eigen::Vector3f v = transform.block<3,3>(0,0) * p.getVector3fMap() +
                                      transform.block<3,1>(0,3);
            pcl::PointXYZI q; q.getVector3fMap() = v;
            const int found = tree.nearestKSearch(q, coarse ? 1 : 5, indices, distances);
            double point_loss = 1.0;
            for (int j=0;j<found;++j) {
                const float map_intensity = (*gicp_map_)[indices[j]].intensity;
                const double difference = std::isfinite(p.intensity) && std::isfinite(map_intensity) ?
                    std::min(1.0, double(std::abs(p.intensity-map_intensity)/intensity_scale_)) : 0;
                point_loss = std::min(point_loss, double(distances[j]) +
                    (coarse ? 0 : config_.intensity_score_weight) * difference * difference);
            }
            loss += point_loss;
        }
        return loss / count;
    };
    struct Candidate { Eigen::Matrix4f pose; double loss; int rank; double distance; };
    std::vector<Candidate> candidates;
    std::vector<Match> selected;
    for (size_t rank = 0; rank < proposals.size(); ++rank) {
        if (int(selected.size()) >= config_.coarse_candidates ||
            (elapsed() > config_.query_budget_ms * 0.70 && !selected.empty())) break;
        const auto& match = proposals[rank];
        bool duplicate = false;
        for (const auto& prior : selected) {
            const int difference = std::abs(prior.shift - match.shift);
            if ((database_[prior.index].center - database_[match.index].center).head<2>().norm() < 1.8 &&
                std::min(difference, config_.sc.sectors - difference) < config_.sc.sectors / 12) {
                duplicate = true; break;
            }
        }
        if (duplicate) continue;
        selected.push_back(match);
        const double yaw = config_.sc.yawForShift(match.shift);
        Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
        initial.block<3,3>(0,0) = Eigen::AngleAxisf(float(yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
        initial.block<3,1>(0,3) = database_[match.index].center;
        Cloud aligned;
        icp.align(aligned, initial);
        ++best.coarse_trials;
        if (!icp.getFinalTransformation().allFinite()) continue;
        const auto transform = icp.getFinalTransformation();
        candidates.push_back({transform, score(transform, true), match.rank,
            match.rank ? match.distance : std::numeric_limits<double>::quiet_NaN()});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.loss < b.loss; });
    if (candidates.size()>8) candidates.resize(8);
    for(auto& candidate:candidates) candidate.loss=score(candidate.pose);
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.loss < b.loss; });
    if (candidates.empty()) return best;
    best.valid = true;
    best.pose = candidates.front().pose;
    best.fitness = candidates.front().loss;
    best.sc_rank = candidates.front().rank;
    best.sc_distance = candidates.front().distance;
    Cloud::Ptr fine_source = downsample(query_base, float(config_.gicp_voxel));
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(fine_source);
    gicp.setInputTarget(gicp_map_);
    gicp.setSearchMethodTarget(fine_tree_, true);
    gicp.setTargetCovariances(target_covariances_);
    gicp.setMaximumIterations(config_.gicp_iterations);
    gicp.setMaxCorrespondenceDistance(config_.gicp_correspondence);
    double last_fine_ms=0;
    for (size_t i = 0; i < candidates.size() && int(i) < config_.fine_candidates; ++i) {
        // Reserve time for the final normal-based refinement, especially the upright mode.
        if (elapsed()+std::max(last_fine_ms,150.0)+250.0 > config_.query_budget_ms) break;
        if (candidates[i].loss > candidates.front().loss*1.5+0.01) continue;
        const double fine_start=elapsed();
        Cloud aligned;
        gicp.align(aligned, candidates[i].pose);
        ++best.gicp_trials;
        last_fine_ms=elapsed()-fine_start;
        if (!gicp.hasConverged() || !gicp.getFinalTransformation().allFinite()) continue;
        const double loss = score(gicp.getFinalTransformation());
        if (loss < best.fitness) {
            best.pose = gicp.getFinalTransformation();
            best.fitness = loss;
            best.sc_rank = candidates[i].rank;
            best.sc_distance = candidates[i].distance;
        }
    }
    if (best.valid && config_.robust_refinement) {
        Eigen::Matrix4f refined=best.pose;
        if(config_.upright_refinement) {
            const float yaw=std::atan2(refined(1,0),refined(0,0));
            refined.block<3,3>(0,0)=Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ()).toRotationMatrix();
        }
        std::vector<int> indices(1); std::vector<float> distances(1);
        int refinement_steps=0;
        for(int iteration=0;iteration<15 && elapsed()<config_.query_budget_ms;++iteration) {
            Eigen::Matrix<double,6,6> h=Eigen::Matrix<double,6,6>::Zero();
            Eigen::Matrix<double,6,1> b=Eigen::Matrix<double,6,1>::Zero();
            int count=0;
            const size_t sample_count=std::min(fine_source->size(),size_t(5000));
            for(size_t point_index=0;point_index<sample_count;++point_index) {
                const auto& p=(*fine_source)[point_index*fine_source->size()/sample_count];
                const Eigen::Vector3d rotated=(refined.block<3,3>(0,0)*p.getVector3fMap()).cast<double>();
                const Eigen::Vector3d v=rotated+refined.block<3,1>(0,3).cast<double>();
                pcl::PointXYZI q; q.getVector3fMap()=v.cast<float>();
                if(!tree.nearestKSearch(q,1,indices,distances) || distances[0]>0.36) continue;
                const auto& n=target_normals_[indices[0]];
                const Eigen::Vector3d target=(*gicp_map_)[indices[0]].getVector3fMap().cast<double>();
                const double residual=n.dot(v-target);
                const double weight=std::min(1.0,0.08/std::max(std::abs(residual),1e-9));
                Eigen::Matrix<double,6,1> jacobian;
                jacobian.head<3>()=rotated.cross(n);jacobian.tail<3>()=n;
                h.noalias()+=weight*jacobian*jacobian.transpose();
                b.noalias()+=weight*jacobian*residual;
                ++count;
            }
            if(count<30) break;
            if(config_.upright_refinement) {
                for(int axis=0;axis<2;++axis) {
                    h.row(axis).setZero();h.col(axis).setZero();h(axis,axis)=1;b[axis]=0;
                }
            }
            h.diagonal().array()+=1e-6;
            const Eigen::Matrix<double,6,1> delta=-h.ldlt().solve(b);
            if(!delta.allFinite() || delta.head<3>().norm()>0.1 || delta.tail<3>().norm()>0.3) break;
            const double angle=delta.head<3>().norm();
            if(angle>1e-12) refined.block<3,3>(0,0)=
                (Eigen::AngleAxisd(angle,delta.head<3>()/angle).toRotationMatrix()*refined.block<3,3>(0,0).cast<double>()).cast<float>();
            refined.block<3,1>(0,3)+=delta.tail<3>().cast<float>();
            ++refinement_steps;
            if(delta.norm()<1e-5) break;
        }
        const double loss=score(refined);
        if(refinement_steps>0 && (config_.upright_refinement || loss<=best.fitness*1.15)) {best.pose=refined;best.fitness=loss;}
    }
    return best;
}
