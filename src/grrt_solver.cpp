
#include "pkgs/cli11.hpp"
#include "spdlog/spdlog.h"

#include "robots/drone/drone.h"

using namespace grrt;

struct SolverCLIOptions {
    std::string configuration_file = "";
};

int main(int argc, char** argv) {
    CLI::App app{"gRRT Solver"};

    SolverCLIOptions options;

    app.add_option("-c,--config", options.configuration_file, "Configuration file path");

    CLI11_PARSE(app, argc, argv);

    spdlog::info("gRRT Solver Starting...");

    if (options.configuration_file.empty()) {
        spdlog::error("No configuration file specified");
        return 1;
    }

    spdlog::info("Loading configuration file: {}", options.configuration_file);
    SolverConfig::SharedPtr config = SolverConfigParser::parse(options.configuration_file);

    return 0;
}