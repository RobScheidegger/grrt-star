#include "pkgs/cli11.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

#include "config/solver_config.h"
#include "config/solver_config_parser.h"

#include "solver/solver.h"

using namespace grrt;

struct SolverCLIOptions {
    std::string configuration_file = "";
    std::string solution_file = "";
};

int main(int argc, char** argv) {
    CLI::App app{"gRRT Player"};

    SolverCLIOptions options;

    app.add_option("-c,--config", options.configuration_file, "Configuration file path");
    app.add_option("-s,--solution", options.solution_file, "Solution file path");

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

    return 0;
}