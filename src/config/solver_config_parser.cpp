#include "config/solver_config_parser.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include "config/solver_config.h"
#include "graphs/roadmap.h"

using namespace grrt;

SolverConfig::SharedPtr SolverConfigParser::parse(const std::string& fileName) {
    SolverConfig::SharedPtr config = std::make_shared<SolverConfig>();

    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << fileName << std::endl;
        return nullptr;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    auto l = ryml::parse_in_place(buffer);
}