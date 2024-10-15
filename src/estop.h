#pragma once

#include <csignal>
#include <functional>
#include <vector>
#include <mutex>

namespace base {
    /// @brief Abstract class for emergency stop callbacks
    class Estop {
    protected:

        /// @brief Constructor
        Estop();

        /// @brief E-stop callback
        /// @param sig Signal number
        virtual void estop(int sig) = 0;

    private:
        static void init_estop_handler();
        static void estop_handler(int sig);

        static bool estop_init_;
        static std::vector<std::function<void(int)>> estop_functions_;
        static std::mutex mutex_;
    };

}
