
#include "cli11.hpp"
#include "spdlog/spdlog.h"

#include "robots/drone/drone.h"

struct SolverCLIOptions {
    std::string configuration_file = "";
};

int main(int argc, char** argv) {
    CLI::App app{"gRRT Solver"};

    SolverCLIOptions options;

    app.add_option("-c,--config", options.configuration_file, "Configuration file path");

    CLI11_PARSE(app, argc, argv);

    grrt::Drone drone;
    return 0;
}