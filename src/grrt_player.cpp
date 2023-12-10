#include "config/solver_config.h"
#include "config/solver_config_parser.h"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "pkgs/cli11.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

#include "voxels/point_cloud/point_cloud_voxel.hpp"

#include "solver/solver.h"

using namespace grrt;
using namespace sensor_msgs::msg;

#define DRONE_SPEED 5.0
#define VOXEL_PUBLISH_HZ 10

class Publisher : public rclcpp::Node {
   public:
    Publisher(SolverResult::SharedPtr& result, const SolverConfig::SharedPtr& config)
        : result(result), config(config), Node("point_cloud_publisher") {
        m_positionPublisher = this->create_publisher<PointCloud2>("positions", 10);
        m_voxelPublisher = this->create_publisher<PointCloud2>("voxels", 10);
        m_edgePublisher = this->create_publisher<visualization_msgs::msg::Marker>("edges", 10);
        m_vertexPublisher = this->create_publisher<visualization_msgs::msg::Marker>("vertices", 10);
    }

    std::vector<RoadmapDart::SharedPtr> getRoadmapDarts(const SearchVertex::SharedPtr& start,
                                                        const SearchVertex::SharedPtr& end) {
        std::vector<RoadmapDart::SharedPtr> darts;

        const size_t num_robots = config->robots.size();
        for (size_t i = 0; i < num_robots; i++) {
            auto roadmap = config->robots[i]->roadmap;
            auto start_vertex = roadmap->vertices[start->roadmapStates[i]];
            auto end_vertex = roadmap->vertices[end->roadmapStates[i]];

            auto dart = roadmap->getDart(start_vertex, end_vertex);
            if (dart == nullptr) {
                spdlog::error("Failed to find dart from {} to {}", start_vertex->m_id, end_vertex->m_id);
                exit(1);
            }
            darts.push_back(dart);
        }

        return darts;
    }

    /// @brief Play the solution in RViz by publishing to the appropriate topics over time
    void playSolution(const SolverResult::SharedPtr& result, const SolverConfig::SharedPtr& config) {

        // Wrap everything in an infinite loop to have everything repeat
        while (true) {
            SearchVertex::SharedPtr current_vertex = nullptr;
            for (auto& vertex : result->path) {
                if (current_vertex == nullptr) {
                    current_vertex = vertex;
                    continue;
                }

                SearchVertex::SharedPtr next_vertex = vertex;

                auto roadmap_darts = getRoadmapDarts(current_vertex, next_vertex);

                // Publish the voxels once
                publishSweptVoxels(roadmap_darts);
                publishPositions(roadmap_darts);
                current_vertex = vertex;
            }
        }
    }

    void publishGraphEdges(const SolverConfig::SharedPtr& config) {
        // Publish the graph edges as a line strip
        auto msg = visualization_msgs::msg::Marker();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.ns = "graph";
        msg.id = 0;
        msg.type = visualization_msgs::msg::Marker::LINE_LIST;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = 0.1;
        msg.color.b = 1.0;
        msg.color.a = 1.0;

        auto points_marker = visualization_msgs::msg::Marker();
        points_marker.header.frame_id = "map";
        points_marker.header.stamp = this->now();
        points_marker.ns = "graph";
        points_marker.id = 1;
        points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::msg::Marker::ADD;
        points_marker.pose.orientation.w = 1.0;
        points_marker.scale.x = 0.5;
        points_marker.scale.y = 0.5;
        points_marker.scale.z = 0.5;
        points_marker.color.r = 1.0;
        points_marker.color.a = 1.0;

        for (const auto& roadmap : config->roadmaps) {
            for (const auto& dart : roadmap.second->darts) {
                auto start_state = dart->getStartState();
                auto end_state = dart->getEndState();

                geometry_msgs::msg::Point start_point;
                start_point.x = start_state->getPosition().x;
                start_point.y = start_state->getPosition().y;
                start_point.z = start_state->getPosition().z;

                geometry_msgs::msg::Point end_point;
                end_point.x = end_state->getPosition().x;
                end_point.y = end_state->getPosition().y;
                end_point.z = end_state->getPosition().z;

                spdlog::debug("Publishing edge from {} ({}) to {} ({})", start_state->getId(),
                              start_state->getPosition().toString(), end_state->getId(),
                              start_state->getPosition().toString());

                msg.points.push_back(start_point);
                msg.points.push_back(end_point);
            }

            for (const auto& vertex : roadmap.second->vertices) {
                geometry_msgs::msg::Point point;
                point.x = vertex->getState()->getPosition().x;
                point.y = vertex->getState()->getPosition().y;
                point.z = vertex->getState()->getPosition().z;

                points_marker.points.push_back(point);
            }
        }

        m_edgePublisher->publish(msg);
        m_vertexPublisher->publish(points_marker);
    }

    void publishPositions(const std::vector<RoadmapDart::SharedPtr>& darts, bool publish_once = false) {
        // Publish the updated positions every 0.1 seconds
        auto positions_msg = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        positions_msg->height = 1;
        positions_msg->width = config->robots.size();
        positions_msg->points.resize(config->robots.size());

        size_t num_robots_moving = config->robots.size();
        float t = 0;
        while (true) {
            size_t robots_finished = 0;
            for (size_t i = 0; i < config->robots.size(); i++) {
                auto roadmap = config->robots[i]->roadmap;
                auto dart = darts[i];

                Point start_position = dart->getStartState()->getPosition();
                Point end_position = dart->getEndState()->getPosition();

                float distance = start_position.distance(end_position);

                // Interpolate the current position based on the speed and time t
                float progress = t / (distance / DRONE_SPEED);
                if (progress > 1) {
                    progress = 1;
                    robots_finished++;
                }

                Point current_position = start_position + (end_position - start_position) * progress;
                positions_msg->points[i] = pcl::PointXYZ(current_position.x, current_position.y, current_position.z);
            }

            // Publish the point cloud
            auto msg = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*positions_msg, msg);
            msg.header.frame_id = "map";
            msg.header.stamp = this->now();
            m_positionPublisher->publish(msg);

            if (robots_finished == config->robots.size() || publish_once) {
                break;
            }

            t += 0.05;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void publishSweptVoxels(const std::vector<RoadmapDart::SharedPtr>& darts) {

        // Print a _single_ voxel message for _all of the robots_

        auto pcl_msg = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl_msg->height = 1;
        pcl_msg->width = 0;

        spdlog::debug("Publishing voxels for {} robots", config->robots.size());
        for (size_t i = 0; i < config->robots.size(); i++) {
            auto roadmap = config->robots[i]->roadmap;
            auto dart = darts[i];

            auto voxel = dart->voxel;
            if (voxel == nullptr) {
                voxel = config->robots[i]->getSweptVoxel(dart);
            }

            PointCloudVoxel::SharedPtr cloud = std::dynamic_pointer_cast<PointCloudVoxel>(voxel);
            spdlog::debug("Publishing voxel for robot {} with {} points", i, cloud->m_points.size());

            pcl_msg->width += cloud->m_points.size();
            pcl_msg->points.reserve(cloud->m_points.size());

            for (std::size_t i = 0; i < cloud->m_points.size(); ++i) {
                pcl_msg->points.push_back(
                    pcl::PointXYZ(cloud->m_points[i].x, cloud->m_points[i].y, cloud->m_points[i].z));
            }
        }

        auto msg = sensor_msgs::msg::PointCloud2();
        pcl::toROSMsg(*pcl_msg, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        spdlog::debug("Publishing voxel with {}, {} points", msg.data.size(), pcl_msg->points.size());
        m_voxelPublisher->publish(msg);
    }

    const SolverResult::SharedPtr result;
    const SolverConfig::SharedPtr config;
    rclcpp::Publisher<PointCloud2>::SharedPtr m_voxelPublisher;
    rclcpp::Publisher<PointCloud2>::SharedPtr m_positionPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_edgePublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_vertexPublisher;
};

struct SolverCLIOptions {
    std::string configuration_file = "";
    std::string solution_file = "";
    size_t voxel = 0;
};

int main(int argc, char** argv) {
    CLI::App app{"gRRT Player"};

    rclcpp::init(argc, argv);

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
    SolverConfig::SharedPtr config = SolverConfigParser::parse(options.configuration_file, VoxelType::POINT_CLOUD);

    if (config == nullptr) {
        spdlog::error("Failed to parse configuration file");
        return 1;
    }

    spdlog::info("Configuration file loaded with {} roadmaps, {} robots, and {} problems", config->roadmaps.size(),
                 config->robots.size(), config->problems.size());

    // Load in the solution file
    spdlog::info("Loading solution file: {}", options.solution_file);
    SolverResult::SharedPtr solution = std::make_shared<SolverResult>(true);
    Result result = SolverConfigParser::parseSolution(options.solution_file, solution);

    std::shared_ptr<Publisher> publisher = std::make_shared<Publisher>(solution, config);

    auto thread = std::thread([&]() {
        while (true) {
            publisher->publishGraphEdges(config);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
    });

    if (options.voxel != 0) {
        spdlog::info("Visualizing voxel {}", options.voxel);

        if (options.voxel > solution->path.size()) {
            spdlog::error("Voxel index {} is out of bounds", options.voxel);
            return 1;
        }

        const auto start_vertex = solution->path[options.voxel - 1];
        const auto end_vertex = solution->path[options.voxel];

        auto roadmap_darts = publisher->getRoadmapDarts(start_vertex, end_vertex);

        while (true) {
            auto thread = std::thread([&]() {
                while (true) {
                    publisher->publishPositions(roadmap_darts, true);
                    publisher->publishSweptVoxels(roadmap_darts);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            });

            rclcpp::spin(publisher);
        }
        publisher->publishSweptVoxels(roadmap_darts);
        rclcpp::spin(publisher);
    } else {
        publisher->playSolution(solution, config);
    }

    return 0;
}