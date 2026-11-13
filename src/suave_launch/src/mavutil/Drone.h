//
// Created by suave on 4/17/24.
//

#ifndef DRONE_H
#define DRONE_H
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <system.h>
#include <plugins/action/action.h>
#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>

#include "TelemetryProperty.h"
#include "../util/Util.h"

using namespace mavsdk;

// Thin wrapper around a MAVSDK System: Action, Offboard and cached Telemetry.
class Drone {
public:
    explicit Drone(std::shared_ptr<System> system);

    std::shared_ptr<System> system() { return m_system; }
    Action& action() { return m_action; }
    Offboard& offboard() { return m_offboard; }
    Telemetry& telemetry() { return m_telemetry; }

    [[nodiscard]]
    double initial_heading_rad() const { return m_initial_heading_rad; }

    // Telemetry Properties
    [[nodiscard]]
    double heading_rad() const
    {
        return heading_deg() * M_PI / 180;
    }
    [[nodiscard]]
    double heading_deg() const
    {
        return m_heading.get().unwrap().heading_deg;
    }
    [[nodiscard]]
    bool in_air() const
    {
        return m_in_air.get().unwrap();
    }
    [[nodiscard]]
    bool armed() const
    {
        return m_armed.get().unwrap();
    }
    [[nodiscard]]
    Telemetry::Quaternion get_quaternion() const
    {
        return m_quaternion.get().unwrap();
    }

    std::string get_quaternion_string() const;
    std::string get_ned_position_string() const;

    // Zero body-velocity setpoint; must be sent before offboard().start()
    Offboard::Result offboard_setpoint();

    // Position relative to the current position (blocks until the next position update)
    Offboard::Result set_relative_position_ned(const float& n, const float& e, const float& d);

    // Position relative to the stored local setpoint (captured by set_local_position_setpoint)
    void set_local_position_setpoint();
    Offboard::Result set_local_position_ned(double n, double e, double d);

    Offboard::Result offboard_land();
    Offboard::Result offboard_wait_for_land();

    Offboard::Result offboard_hold();

private:
    // Mavsdk
    std::shared_ptr<System> m_system{};
    Action m_action{*m_system};
    Offboard m_offboard{*m_system};
    Telemetry m_telemetry{*m_system};

    // Telemetry Properties
    TelemetryProperty<Telemetry::Heading> m_heading {m_telemetry, &Telemetry::subscribe_heading};
    TelemetryProperty<bool> m_in_air {m_telemetry, &Telemetry::subscribe_in_air};
    TelemetryProperty<bool> m_armed {m_telemetry, &Telemetry::subscribe_armed};
    TelemetryProperty<Telemetry::PositionVelocityNed> m_position_velocity_ned {m_telemetry, &Telemetry::subscribe_position_velocity_ned};
    TelemetryProperty<Telemetry::EulerAngle> m_attitude_euler {m_telemetry, &Telemetry::subscribe_attitude_euler};
    TelemetryProperty<Telemetry::Quaternion> m_quaternion {m_telemetry, &Telemetry::subscribe_attitude_quaternion};

    // Misc
    double m_initial_heading_rad{ NAN };
    struct LocalPosition
    {
        float north;
        float east;
        float down;
        float yaw;
    };
    std::optional<LocalPosition> m_local_setpoint{};
};

#endif //DRONE_H
