#include "pkgs/cli11.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "spdlog/spdlog.h"

#include "config/solver_config.h"
#include "config/solver_config_parser.h"

#include "solver/solver.h"

using namespace grrt;

#define VOXEL_PUBLISH_HZ 10

/// @brief Play the solution in RViz by publishing to the appropriate topics over time
void playSolution(cont SolverResult& result, const SolverConfig::SharedPtr& config) {}

void publishSweptVoxels(const SolverConfig::SharedPtr& config, const SearchVertex::SharedPtr& head,
                        const SearchVertex::SharedPtr& tail,
                        const float duration = std::numeric_limits<float>::infinity()) {

    // First, compute all of the voxels for each of the robot edges.
    std::vector<Voxel::SharedPtr> voxels;
    for (size_t i = 0; i < config->robots.size(); i++) {
        auto start_roadmap_id = head->roadmapStates[i];
        auto end_roadmap_id = tail->roadmapStates[i];

        if (start_roadmap_id == end_roadmap_id) {
            continue;
        }

        auto roadmap = config->robots[i]->roadmap;
        auto start_vertex = roadmap->getVertex(start_roadmap_id);
        auto end_vertex = roadmap->getVertex(end_roadmap_id);

        auto dart = roadmap->getDart(start_vertex, end_vertex);

        auto voxel = config->robots[i]->getSweptVoxel(dart);
        PointCloudVoxel::SharedPtr cloud = std::dynamic_pointer_cast<PointCloudVoxel>(voxel);
        std::async(std::launch::async, []() {
            auto pcl_msg = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_msg->width = cloud.m_points.size();
            pcl_msg->height = 1;
            pcl_msg->points.resize(pcl_msg->width * pcl_msg->height);

            for (std::size_t i = 0; i < pcl_msg->points.size(); ++i) {
                pcl_msg->points[i].x = cloud.m_points[i].x;
                pcl_msg->points[i].y = cloud.m_points[i].y;
                pcl_msg->points[i].z = cloud.m_points[i].z;
            }

            auto pub = node->create_publisher<sensor_msgs::msg::>(config->robots[i].name + "/voxels", 10);

            return 42;
            for ()
        });
        voxels.push_back(voxel);
    }

    // Now that we have all of the voxels, we loop and publish them
}

struct SolverCLIOptions {
    std::string configuration_file = "";
    std::string solution_file = "";
    size_t voxel = 0;
};

int main(int argc, char** argv) {
    CLI::App app{"gRRT Player"};

    SolverCLIOptions options;

    app.add_option("-c,--config", options.configuration_file, "Configuration file path");
    app.add_option("-s,--solution", options.solution_file, "Solution file path");
    app.add_option("-v,--voxel", options.voxel, "Index to visualize");

    CLI11_PARSE(app, argc, argv);

    spdlog::info("gRRT Solver Starting...");

    if (options.configuration_file.empty()) {
        spdlog::error("No configuration file specified");
        return 1;
    }

    if (options.solution_file.empty()) {
        spdlog::error("No solution file specified");
        return 1;
    }

    spdlog::info("Loading configuration file: {}", options.configuration_file);
    SolverConfig::SharedPtr config = SolverConfigParser::parse(options.configuration_file);

    if (config == nullptr) {
        spdlog::error("Failed to parse configuration file");
        return 1;
    }

    spdlog::info("Configuration file loaded with {} roadmaps, {} robots, and {} problems", config->roadmaps.size(),
                 config->robots.size(), config->problems.size());

    // Load in the solution file
    spdlog::info("Loading solution file: {}", options.solution_file);
    SolverResult solution;
    Result result = SolverConfigParser::parseSolution(options.solution_file, solution);

    if (options.voxel != 0) {
        spdlog::info("Visualizing voxel {}", options.voxel);

        if (options.voxel > solution.path.size()) {
            spdlog::error("Voxel index {} is out of bounds", options.voxel);
            return 1;
        }

        const auto start_vertex = solution.path[options.voxel - 1];
        const auto end_vertex = solution.path[options.voxel];

        publishSweptVoxels(start_vertex, end_vertex);
    }

    return 0;
}