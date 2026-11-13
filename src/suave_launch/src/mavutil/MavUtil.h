//
// Created by suave on 4/17/24.
//

#ifndef MAVUTIL_H
#define MAVUTIL_H
#include <cstdint>  // MAVSDK 1.4 headers rely on it transitively
#include <mavsdk.h>
#include <memory>

// True when SUAVE_MAVLINK_SERIAL_ID is unset or "SITL" (connect over udp://:14540)
bool is_sitl_connection();

std::shared_ptr<mavsdk::System> connectToPX4(mavsdk::Mavsdk& m_mavsdk);

#endif //MAVUTIL_H
