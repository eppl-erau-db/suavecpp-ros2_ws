//
// Created by suave on 4/17/24.
//

#include "Drone.h"
#include <cmath>
#include <sstream>

#include "../controllers/ControllerMacros.h"

Drone::Drone(std::shared_ptr<System> system): m_system(std::move(system))
{
    // VIOBridge rotates the camera frame into NED with this angle; 0 assumes the drone
    // starts facing the direction you want to be +North in the map.
    m_initial_heading_rad = 0;
    suave_log << "Initial heading: " << m_initial_heading_rad << " rad\n";
}

std::string Drone::get_quaternion_string() const
{
    auto quaternion = get_quaternion();
    std::stringstream ss;
    ss << "Quaternion- w: " << quaternion.w << ", x: " << quaternion.x << ", y: " << quaternion.y << ", z: " << quaternion.z;
    return ss.str();
}

std::string Drone::get_ned_position_string() const
{
    auto position_velocity_ned = m_position_velocity_ned.get().unwrap();
    auto position = position_velocity_ned.position;
    std::stringstream ss;
    ss << "NED Position- North: " << position.north_m << ", East: " << position.east_m << ", Down: " << position.down_m;
    return ss.str();
}

Offboard::Result Drone::offboard_setpoint()
{
    return offboard().set_velocity_body({0, 0, 0, 0});
}

Offboard::Result Drone::set_relative_position_ned(const float& n, const float& e, const float& d)
{
    const auto [north_m, east_m, down_m] = m_position_velocity_ned.wait_for_next().unwrap().position;
    const auto yaw = m_attitude_euler.wait_for_next(1).unwrap().yaw_deg;
    return offboard().set_position_ned({north_m + n, east_m + e, down_m + d, yaw});
}

void Drone::set_local_position_setpoint()
{
    const auto [north_m, east_m, down_m] = m_position_velocity_ned.wait_for_next().unwrap().position;
    const auto yaw = m_attitude_euler.wait_for_next(1).unwrap().yaw_deg;
    m_local_setpoint = { north_m, east_m, down_m, yaw };
}

Offboard::Result Drone::set_local_position_ned(double n, double e, double d)
{
    if (!m_local_setpoint)
    {
        set_local_position_setpoint();
    }
    return offboard().set_position_ned({
        static_cast<float>(m_local_setpoint->north + n),
        static_cast<float>(m_local_setpoint->east + e),
        static_cast<float>(m_local_setpoint->down + d),
        m_local_setpoint->yaw });
}

Offboard::Result Drone::offboard_land()
{
    const auto yaw = m_attitude_euler.wait_for_next().unwrap().yaw_deg;
    return offboard().set_velocity_ned({0, 0, 0.5, yaw});
}

Offboard::Result Drone::offboard_wait_for_land()
{
    auto land_result = offboard_land();
    if (land_result != Offboard::Result::Success)
    {
        suave_err << "Offboard descent failed (" << land_result << "), switching to PX4 land mode" << std::endl;
        action().land();
        return land_result;
    }

    // Descend at 0.5 m/s until PX4 reports landed. 30 s covers ~15 m of descent.
    int elapse_sec = 0;
    constexpr auto MAX_ELAPSE = 30;
    while (in_air())
    {
        suave_log << "Waiting for drone to land, elapse_sec: " << elapse_sec << std::endl;

        if (elapse_sec >= MAX_ELAPSE)
        {
            // Never disarm while airborne: hand over to PX4's own landing instead.
            suave_err << "Still in air after " << MAX_ELAPSE << " s, switching to PX4 land mode" << std::endl;
            action().land();
            return land_result;
        }

        sleep(1)
        elapse_sec++;
    }
    sleep(3)
    auto disarm_result = action().disarm();
    if (disarm_result != Action::Result::Success)
    {
        suave_err << "Disarm failed: " << disarm_result << std::endl;
    }

    return land_result;
}

Offboard::Result Drone::offboard_hold()
{
    return set_relative_position_ned(0, 0, 0);
}
