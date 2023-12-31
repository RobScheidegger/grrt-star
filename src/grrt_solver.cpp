#include <mpi.h>

#include "pkgs/cli11.hpp"
#include "spdlog/spdlog.h"

#include "config/solver_config.h"
#include "config/solver_config_parser.h"

#include "solver/solver.h"

using namespace grrt;

struct SolverCLIOptions {
    std::string configuration_file = "";
    std::string voxelType = "PCL";
    bool useMPI = false;
    size_t timeoutSeconds = 10;
};

int main(int argc, char** argv) {
    CLI::App app{"gRRT Solver"};

    SolverCLIOptions options;

    app.add_option("-c,--config", options.configuration_file, "Configuration file path");
    app.add_option("-m,--mpi", options.useMPI, "Use MPI");
    app.add_option("-t,--timeout", options.timeoutSeconds, "Timeout in seconds");
    app.add_option("-v,--voxel-type", options.voxelType, "Voxel type");

    CLI11_PARSE(app, argc, argv);

    spdlog::info("gRRT Solver Starting...");

    if (options.configuration_file.empty()) {
        spdlog::error("No configuration file specified");
        return 1;
    }

    VoxelType voxelType;
    Result result = SolverConfigParser::parseVoxelType(options.voxelType, voxelType);
    if (result.isError) {
        spdlog::error("Failed to parse voxel type: {}", result.msg);
        return 1;
    }

    spdlog::info("Loading configuration file: {}", options.configuration_file);
    SolverConfig::SharedPtr config = SolverConfigParser::parse(options.configuration_file, voxelType);

    if (config == nullptr) {
        spdlog::error("Failed to parse configuration file");
        return 1;
    }

    config->timeoutSeconds = options.timeoutSeconds;

    int mpi_size = -1;
    int mpi_rank = -1;

    if (options.useMPI) {
        spdlog::info("Using MPI");
        MPI_Init(&argc, &argv);
        MPI_Comm_rank(MPI_COMM_WORLD, &mpi_rank);
        MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);
        spdlog::info("MPI rank {} of {}", mpi_rank, mpi_size);
        config->useMPI = true;
        config->mpiSize = mpi_size;
        config->mpiRank = mpi_rank;
    }

    spdlog::info("Configuration file loaded with {} roadmaps, {} robots, and {} problems", config->roadmaps.size(),
                 config->robots.size(), config->problems.size());

    Solver::SharedPtr solver = std::make_shared<Solver>(config);

    if (options.useMPI && mpi_rank != 0) {
        spdlog::info("Launching MPI worker {}", mpi_rank);
        solver->launchMPIWorker();
        MPI_Finalize();
        return 0;
    }

    auto solutions = solver->solve();
    for (const auto& [solution_name, solution] : *solutions) {
        spdlog::info("Solution: cost {} success {}", solution->cost, solution->success);
        // Open a file to write the solution to.
        std::string solutionFileName = "solution_" + solution_name + ".yaml.sol";
        std::ofstream solutionFile(solutionFileName);
        SolverConfigParser::printSolution(solutionFile, config, solution);

        spdlog::info("Graph Points Considered: {}", solver->m_pointsConsidered);
    }

    if (options.useMPI) {
        MPI_Finalize();
    }

    return 0;
}