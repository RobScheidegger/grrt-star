#pragma once

#include <string>

namespace grrt {
    struct Result {
        const std::string msg;
        const bool isError;
        Result(bool isError, const std::string& msg) : msg(msg), isError(isError) {}

        static Result Ok(const std::string& msg) { return Result(false, msg); }
        static Result Ok() { return Result(false, "Success"); }
        static Result Error(const std::string& msg) { return Result(true, msg); }
        static Result Error() { return Result(true, "Error"); }
    };
}  // namespace grrt