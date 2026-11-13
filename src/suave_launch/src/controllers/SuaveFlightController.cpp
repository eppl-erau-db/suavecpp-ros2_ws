#include "SuaveFlightController.h"

#include <cmath>
#include <iostream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../common/SystemTask.h"
#include "../mavutil/MavUtil.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/VIOBridge.h"
#include "ControllerMacros.h"

namespace
{
constexpr float DEFAULT_TAKEOFF_ALT_M = 1.75f;
constexpr float MAX_ALT_M = 5.0f;   // takeoff altitude limit
constexpr float MAX_STEP_M = 5.0f;  // largest single move/goto distance accepted
}

SuaveFlightController::SuaveFlightController() :
    ISuaveController()
{
    m_sitl = is_sitl_connection();
    // Throws if the connection cannot be opened; caught in main().
    m_drone = std::make_unique<Drone>(connectToPX4(m_mavsdk));
}

void SuaveFlightController::start()
{
    if (m_sitl)
    {
        suave_log << "SITL connection: skipping RealSense / RTAB-Map / VIO" << std::endl;
    }
    else
    {
        start_vio_stack();
    }

    print_help();
    command_loop();
    shutdown();
}

void SuaveFlightController::start_vio_stack()
{
    const auto slam_launch = ament_index_cpp::get_package_share_directory("suave_launch") + "/launch/suave_slam.py";

    auto realsense_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true"
        }
    );
    auto rtabmap_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "ros2 param set /camera/camera depth_module.emitter_enabled 0",
            "ros2 launch " + slam_launch
        }
    );
    auto vio_spinner = std::make_shared<RosNodeSpinner>();
    vio_spinner->add_node(std::make_shared<VIOBridge>(m_drone->system(), m_drone->initial_heading_rad()));

    m_tasks = { realsense_task, rtabmap_task, vio_spinner };

    suave_log << "Starting RealSense" << std::endl;
    realsense_task->start_in_thread();
    sleep(3)

    suave_log << "Starting RTAB-Map" << std::endl;
    rtabmap_task->start_in_thread();
    sleep(5)

    suave_log << "Move the drone around by hand to build the initial map, then press enter" << std::endl;
    std::cin.clear();
    await_confirmation;

    suave_log << "Starting VIO bridge (RTAB-Map /odom -> PX4 external vision)" << std::endl;
    vio_spinner->start_in_thread();
}

bool SuaveFlightController::enter_offboard()
{
    if (m_offboard_active)
    {
        return true;
    }

    // Capture the takeoff point (position + yaw); goto targets are relative to it.
    m_drone->set_local_position_setpoint();

    const auto arm_result = m_drone->action().arm();
    if (arm_result != Action::Result::Success)
    {
        suave_err << "Arm failed: " << arm_result << std::endl;
        return false;
    }
    // PX4 needs a setpoint streaming before it accepts offboard mode.
    if (!check("offboard setpoint", m_drone->offboard_setpoint()) ||
        !check("offboard start", m_drone->offboard().start()))
    {
        m_drone->action().disarm();
        return false;
    }
    m_offboard_active = true;
    return true;
}

void SuaveFlightController::command_loop()
{
    std::string line{};
    while (!m_end_controller)
    {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line))
        {
            break;  // EOF (Ctrl-D)
        }

        std::istringstream in{ line };
        std::string cmd{};
        if (!(in >> cmd))
        {
            continue;
        }

        try
        {
            if (cmd == "help")
            {
                print_help();
            }
            else if (cmd == "takeoff")
            {
                float alt = DEFAULT_TAKEOFF_ALT_M;
                in >> alt;
                if (!(alt > 0.0f && alt <= MAX_ALT_M))
                {
                    suave_err << "takeoff altitude must be in (0, " << MAX_ALT_M << "] m" << std::endl;
                    continue;
                }
                if (enter_offboard())
                {
                    check("takeoff", m_drone->set_local_position_ned(0, 0, -alt));
                }
            }
            else if (cmd == "move" || cmd == "goto")
            {
                float n{}, e{}, d{};
                if (!(in >> n >> e >> d))
                {
                    suave_err << "usage: " << cmd << " <north> <east> <down>   (metres, down < 0 is up)" << std::endl;
                    continue;
                }
                if (!m_offboard_active)
                {
                    suave_err << "not in offboard; use takeoff first" << std::endl;
                    continue;
                }
                if (std::sqrt(n * n + e * e + d * d) > MAX_STEP_M)
                {
                    suave_err << "rejected: more than " << MAX_STEP_M << " m" << std::endl;
                    continue;
                }
                if (cmd == "move")
                {
                    check("move", m_drone->set_relative_position_ned(n, e, d));
                }
                else
                {
                    check("goto", m_drone->set_local_position_ned(n, e, d));
                }
            }
            else if (cmd == "hold")
            {
                if (m_offboard_active)
                {
                    check("hold", m_drone->offboard_hold());
                }
            }
            else if (cmd == "land")
            {
                if (m_offboard_active)
                {
                    check("land", m_drone->offboard_wait_for_land());
                    m_offboard_active = false;
                }
            }
            else if (cmd == "status")
            {
                suave_log << m_drone->get_ned_position_string() << std::endl;
                suave_log << "Heading: " << m_drone->heading_deg() << " deg, armed: " << m_drone->armed()
                          << ", in air: " << m_drone->in_air() << ", offboard: " << m_offboard_active << std::endl;
            }
            else if (cmd == "exit" || cmd == "quit")
            {
                break;
            }
            else
            {
                suave_err << "unknown command '" << cmd << "' (type help)" << std::endl;
            }
        }
        catch (const std::exception& ex)
        {
            // e.g. telemetry not received yet
            suave_err << cmd << " failed: " << ex.what() << std::endl;
        }
    }
}

void SuaveFlightController::print_help()
{
    std::cout <<
        "Commands (NED frame, metres; down < 0 is up):\n"
        "  takeoff [alt]      arm, enter offboard, climb to alt above the takeoff point (default 1.75)\n"
        "  move dn de dd      step relative to the current position\n"
        "  goto n e d         fly to a point relative to the takeoff point\n"
        "  hold               hold the current position\n"
        "  land               descend at 0.5 m/s, disarm on touchdown\n"
        "  status             position, heading, armed / in-air / offboard state\n"
        "  exit               land if airborne, stop camera/VIO, quit\n"
        "Ctrl-C at any time lands and exits." << std::endl;
}

bool SuaveFlightController::check(const std::string& what, const Offboard::Result result)
{
    if (result == Offboard::Result::Success)
    {
        suave_log << what << ": ok" << std::endl;
        return true;
    }
    suave_err << what << " failed: " << result << std::endl;
    return false;
}

void SuaveFlightController::shutdown()
{
    if (m_shutdown_done.exchange(true))
    {
        return;
    }
    m_end_controller = true;
    suave_log << "SuaveFlightController::shutdown()" << std::endl;

    if (m_drone)
    {
        try
        {
            if (m_offboard_active)
            {
                m_drone->offboard_wait_for_land();
            }
            else if (m_drone->in_air())
            {
                m_drone->action().land();
            }
        }
        catch (const std::exception& ex)
        {
            suave_err << "Landing during shutdown failed: " << ex.what() << std::endl;
        }
    }

    for (auto& task : m_tasks)
    {
        task->stop();
    }
}
