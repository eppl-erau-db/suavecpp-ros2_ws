//
// Created by suave on 4/17/24.
//

#include "MavUtil.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "../util/Util.h"

std::string get_env_var( std::string const & key ) {
    char * val;
    val = getenv( key.c_str() );
    std::string retval = "";
    if (val != NULL) {
        retval = val;
    }
    return retval;
}

bool is_sitl_connection()
{
    const auto id = get_env_var("SUAVE_MAVLINK_SERIAL_ID");
    return id == "SITL" || id.size() <= 2;
}

std::shared_ptr<mavsdk::System> connectToPX4(mavsdk::Mavsdk& m_mavsdk)
{
    const auto id = get_env_var("SUAVE_MAVLINK_SERIAL_ID");

    std::string connection_url{};
    if (is_sitl_connection())
    {
        connection_url = "udp://:14540";
    }
    else
    {
        const std::string connection_string = "/dev/serial/by-id/" + id;
        connection_url = "serial://" + connection_string + ":57600";
    }

    // Connect to the PX4 SITL.

    suave_log << "Attempting to connect to " << connection_url << std::endl;
    mavsdk::ConnectionResult connection_result = m_mavsdk.add_any_connection(connection_url);
    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        throw std::runtime_error((std::stringstream() << "Connection failed: " << connection_result << '\n').str());
    }

    // Wait until the vehicle is ready.
    while (m_mavsdk.systems().empty())
    {
        suave_log << "Waiting for system to connect...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return *m_mavsdk.systems().begin();
}
