#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "controllers/SuaveFlightController.h"
#include "util/Util.h"

std::unique_ptr<SuaveFlightController> s_controller;

void signal_handler(int sig)
{
    suave_warn << "Signal received: " << sig << std::endl;
    if (s_controller)
    {
        s_controller->shutdown();
    }
    rclcpp::shutdown();
    suave_log << "Exiting suave_main" << std::endl;
    exit(0);
}

int main(int argc, char** argv)
{
    suave_log << "Starting suave_main" << std::endl;
    rclcpp::init(argc, argv);
    for (const auto sig : { SIGINT, SIGTERM, SIGQUIT })
    {
        std::signal(sig, signal_handler);
    }

    try
    {
        s_controller = std::make_unique<SuaveFlightController>();
        s_controller->start();
    }
    catch (const std::exception& e)
    {
        suave_err << "Caught exception: " << e.what() << std::endl;
    }

    if (s_controller)
    {
        s_controller->shutdown();
    }
    rclcpp::shutdown();
    suave_log << "Exiting suave_main" << std::endl;
    return 0;
}
