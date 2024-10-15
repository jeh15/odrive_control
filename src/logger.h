#pragma once

#include <memory>
#include <fstream>
#include <mutex>

namespace base {
    // @brief Class for logging data
    class Logger {
    public:
        using Ptr = std::shared_ptr<Logger>;

        // @brief Constructor
        Logger(const std::string &filename, const std::string &dir = "/tmp");

        // @brief Destructor
        ~Logger();

        // @brief Log data
        void log(const std::string &data);

    private:
        std::ofstream file_;
        std::mutex mutex_;
    };
}
