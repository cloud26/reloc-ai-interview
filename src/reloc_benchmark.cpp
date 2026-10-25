// 真值只留在评测器里；OfflineLocalizer 只接收地图和 base 系 query。
#include "offline_localizer.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct GroundTruth {
    std::string filename;
    double map_base[6]{};
    double odom_base[6]{};
};
struct Row {
    std::string filename;
    bool localized = false;
    bool success = false;
    double position_error = 0.0;
    double yaw_error = 0.0;
    double fitness = 0.0;
    double sc_distance = 0.0;
    int sc_rank = 0;
    int gicp_trials = 0;
    int coarse_trials = 0;
    Eigen::Matrix4f estimated_pose = Eigen::Matrix4f::Identity();
    double duration_ms = 0.0;
    std::string reason;
};

Eigen::Matrix4f pose(const double fields[6]) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    constexpr double deg = M_PI / 180.0;
    result.block<3, 3>(0, 0) =
        (Eigen::AngleAxisf(fields[3] * deg, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(fields[4] * deg, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(fields[5] * deg, Eigen::Vector3f::UnitX())).matrix();
    result.block<3, 1>(0, 3) = Eigen::Vector3f(fields[0], fields[1], fields[2]);
    return result;
}

std::vector<GroundTruth> readGroundTruth(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open " + path);
    std::string line;
    std::getline(input, line);
    std::vector<GroundTruth> rows;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        std::stringstream stream(line);
        std::vector<std::string> cells;
        std::string cell;
        while (std::getline(stream, cell, ',')) cells.push_back(cell);
        if (cells.size() != 13) throw std::runtime_error("Bad groundtruth row: " + line);
        GroundTruth row;
        row.filename = cells[0];
        for (int j = 0; j < 6; ++j) {
            row.map_base[j] = std::stod(cells[1 + j]);
            row.odom_base[j] = std::stod(cells[7 + j]);
        }
        rows.push_back(row);
    }
    return rows;
}

double yaw(const Eigen::Matrix4f& matrix) {
    return std::atan2(matrix(1, 0), matrix(0, 0)) * 180.0 / M_PI;
}
double angleError(double a, double b) {
    return std::abs(std::remainder(a - b, 360.0));
}
double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const size_t middle = values.size() / 2;
    return values.size() % 2 ? values[middle] : (values[middle - 1] + values[middle]) / 2.0;
}
}

int main(int argc, char** argv) {
    try {
        std::string map_path, evalset, config_path;
        double position_threshold = 0.2, yaw_threshold = 2.0, time_budget_ms = 3000.0;
        for (int i = 1; i < argc; ++i) {
            const std::string flag = argv[i];
            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + flag);
            const std::string value = argv[++i];
            if (flag == "--map") map_path = value;
            else if (flag == "--evalset") evalset = value;
            else if (flag == "--config") config_path = value;
            else if (flag == "--pos-thresh") position_threshold = std::stod(value);
            else if (flag == "--yaw-thresh") yaw_threshold = std::stod(value);
            else if (flag == "--time-budget-ms") time_budget_ms = std::stod(value);
            else throw std::runtime_error("Unknown flag: " + flag);
        }
        if (map_path.empty() || evalset.empty() || config_path.empty()) {
            throw std::runtime_error("Usage: reloc_benchmark --map map.pcd --evalset DIR --config config.yaml");
        }
        if (!std::isfinite(time_budget_ms) || time_budget_ms <= 0)
            throw std::runtime_error("Time budget must be positive and finite");
        const YAML::Node yaml = YAML::LoadFile(config_path);
        BaselineConfig config;
        config.sc.radius = yaml["sc_max_radius"].as<double>();
        config.sc.rings = yaml["sc_num_rings"].as<int>();
        config.sc.sectors = yaml["sc_num_sectors"].as<int>();
        config.sc.intensity_weight = yaml["sc_intensity_weight"].as<double>();
        config.grid_resolution = yaml["sc_grid_resolution"].as<double>();
        config.sensor_height = yaml["sc_sensor_height"].as<double>();
        if (yaml["sc_top_k"]) config.coarse_candidates = yaml["sc_top_k"].as<int>();
        config.gicp_voxel = yaml["gicp_voxel"].as<double>();
        config.gicp_correspondence = yaml["gicp_max_correspondence"].as<double>();
        config.gicp_iterations = yaml["gicp_max_iterations"].as<int>();

        if (yaml["query_budget_ms"]) config.query_budget_ms = yaml["query_budget_ms"].as<double>();
        if (yaml["coarse_candidates"]) config.coarse_candidates = yaml["coarse_candidates"].as<int>();
        if (yaml["fine_candidates"]) config.fine_candidates = yaml["fine_candidates"].as<int>();

        if (yaml["geometric_candidates"]) config.geometric_candidates = yaml["geometric_candidates"].as<int>();

        if (yaml["intensity_score_weight"]) config.intensity_score_weight = yaml["intensity_score_weight"].as<double>();

        if (yaml["robust_refinement"]) config.robust_refinement = yaml["robust_refinement"].as<bool>();

        if (yaml["sc_index_voxel"]) config.sc_index_voxel = yaml["sc_index_voxel"].as<double>();

        if (yaml["upright_refinement"]) config.upright_refinement = yaml["upright_refinement"].as<bool>();

        if (yaml["retrieval_threads"]) config.retrieval_threads = yaml["retrieval_threads"].as<int>();

        OfflineLocalizer localizer(config);
        const auto index_start = std::chrono::steady_clock::now();
        if (!localizer.loadMap(map_path)) throw std::runtime_error("Map/index load failed");
        const double index_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - index_start).count();
        std::cout << "[BENCH] map_and_index_load_ms=" << index_ms << '\n';

        const auto groundtruth = readGroundTruth(evalset + "/groundtruth.csv");
        if (groundtruth.empty()) throw std::runtime_error("No groundtruth rows");
        std::vector<Row> results;
        for (const auto& gt : groundtruth) {
            Row row;
            row.filename = gt.filename;
            OfflineLocalizer::Cloud::Ptr query_odom(new OfflineLocalizer::Cloud);
            const std::string query_path = evalset + "/queries/" + gt.filename;
            if (pcl::io::loadPCDFile(query_path, *query_odom) < 0 || query_odom->empty()) {
                row.reason = "load_query_failed";
                results.push_back(row);
                continue;
            }
            // 这一步只用记录时 odom→base，消除 query 的记录坐标系；绝不传 map 真值。
            OfflineLocalizer::Cloud::Ptr query_base(new OfflineLocalizer::Cloud);
            pcl::transformPointCloud(*query_odom, *query_base, pose(gt.odom_base).inverse().eval());
            const auto start = std::chrono::steady_clock::now();
            const Estimate estimate = localizer.localize(query_base);
            row.duration_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            row.localized = estimate.valid;
            row.fitness = estimate.fitness;
            row.sc_rank = estimate.sc_rank;
            row.sc_distance = estimate.sc_distance;
            row.gicp_trials = estimate.gicp_trials;
            row.coarse_trials = estimate.coarse_trials;
            row.estimated_pose = estimate.pose;
            if (!estimate.valid) {
                row.reason = "reloc_failed";
            } else {
                const Eigen::Matrix4f truth = pose(gt.map_base);
                row.position_error = std::hypot(estimate.pose(0, 3) - truth(0, 3),
                                                estimate.pose(1, 3) - truth(1, 3));
                row.yaw_error = angleError(yaw(estimate.pose), yaw(truth));
                row.success = row.position_error <= position_threshold && row.yaw_error <= yaw_threshold;
                row.reason = row.success ? "-" : row.position_error > 2.0 ? "wrong_place" : "imprecise";
            }
            results.push_back(row);
            std::cout << gt.filename << " pos=" << std::fixed << std::setprecision(3)
                      << row.position_error << "m yaw=" << row.yaw_error << "deg"
                      << " sc_rank=" << row.sc_rank << " fitness=" << row.fitness
                      << " time_ms=" << row.duration_ms << " " << row.reason << '\n';
        }

        const std::string output = evalset + "/benchmark_result.csv";
        std::ofstream csv(output);
        csv << "filename,success,reloc_ok,pos_err_m,yaw_err_deg,fitness,sc_rank,sc_distance,gicp_trials,time_ms,fail_reason,coarse_trials,est_x,est_y,est_z,est_yaw_deg,est_pitch_deg,est_roll_deg\n";
        std::vector<double> durations;
        int successes = 0, wrong = 0, imprecise = 0, failed = 0;
        int within_budget = 0, over_budget = 0;
        for (const auto& row : results) {
            csv << row.filename << ',' << row.success << ',' << row.localized << ','
                << row.position_error << ',' << row.yaw_error << ',' << row.fitness << ','
                << row.sc_rank << ',' << row.sc_distance << ',' << row.gicp_trials << ','
                << row.duration_ms << ',' << row.reason << ',' << row.coarse_trials << ','
                << row.estimated_pose(0,3) << ',' << row.estimated_pose(1,3) << ',' << row.estimated_pose(2,3) << ','
                << yaw(row.estimated_pose) << ',' << std::asin(-std::clamp(double(row.estimated_pose(2,0)),-1.0,1.0))*180/M_PI << ','
                << std::atan2(row.estimated_pose(2,1),row.estimated_pose(2,2))*180/M_PI << '\n';
            durations.push_back(row.duration_ms);
            successes += row.success;
            within_budget += row.success && row.duration_ms <= time_budget_ms;
            over_budget += row.duration_ms > time_budget_ms;
            wrong += row.reason == "wrong_place";
            imprecise += row.reason == "imprecise";
            failed += row.reason == "reloc_failed" || row.reason == "load_query_failed";
        }
        std::cout << "Summary: " << successes << '/' << results.size()
                  << " success, wrong_place=" << wrong << " imprecise=" << imprecise
                  << " reloc_failed=" << failed << " median_query_ms=" << median(durations)
                  << " success_under_budget=" << within_budget << '/' << results.size()
                  << " time_budget_ms=" << time_budget_ms << " over_budget=" << over_budget
                  << " max_query_ms=" << *std::max_element(durations.begin(), durations.end())
                  << "\nCSV: " << output << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Benchmark error: " << error.what() << '\n';
        return 1;
    }
}
