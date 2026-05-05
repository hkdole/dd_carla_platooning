#pragma once

// MIT License
// Copyright (c) 2023 Valerio Cislaghi, Christian Quadri

/*
 * Messages exchanged between carlanetpp and pycarlanet
 */

#include <array>
#include <list>
#include <string>

#include "lib/json.hpp"

using json = nlohmann::json;

#define SIM_STATUS_RUNNING 0
#define SIM_STATUS_FINISHED_OK 1
#define SIM_STATUS_FINISHED_ACCIDENT 2
#define SIM_STATUS_FINISHED_TIME_LIMIT 3
#define SIM_STATUS_ERROR -1

namespace carla_api_base {

struct init_actor {
    std::string actor_id;
    std::string actor_type;
    json actor_configuration;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    init_actor,
    actor_id,
    actor_type,
    actor_configuration
)

struct actor_position {
    std::string actor_id;

    // Raw CARLA state
    std::array<double, 3> position{{0.0, 0.0, 0.0}};              // x, y, z
    std::array<double, 3> velocity{{0.0, 0.0, 0.0}};              // vx, vy, vz
    std::array<double, 3> acceleration{{0.0, 0.0, 0.0}};          // ax, ay, az

    // Python sends [pitch, yaw, roll] in degrees
    std::array<double, 3> rotation{{0.0, 0.0, 0.0}};              // pitch, yaw, roll

    // Optional raw angular state from CARLA
    std::array<double, 3> angular_velocity{{0.0, 0.0, 0.0}};
    std::array<double, 3> angular_acceleration{{0.0, 0.0, 0.0}};

    bool is_net_active = true;
};

inline void to_json(json& j, const actor_position& p)
{
    j = json{
        {"actor_id", p.actor_id},
        {"position", p.position},
        {"velocity", p.velocity},
        {"acceleration", p.acceleration},
        {"rotation", p.rotation},
        {"angular_velocity", p.angular_velocity},
        {"angular_acceleration", p.angular_acceleration},
        {"is_net_active", p.is_net_active},
    };
}

inline void from_json(const json& j, actor_position& p)
{
    j.at("actor_id").get_to(p.actor_id);
    j.at("position").get_to(p.position);
    j.at("velocity").get_to(p.velocity);
    j.at("rotation").get_to(p.rotation);

    // Backward-compatible defaults.
    // Old Python snapshots without these fields still parse.
    p.acceleration = j.value(
        "acceleration",
        std::array<double, 3>{{0.0, 0.0, 0.0}}
    );

    p.angular_velocity = j.value(
        "angular_velocity",
        std::array<double, 3>{{0.0, 0.0, 0.0}}
    );

    p.angular_acceleration = j.value(
        "angular_acceleration",
        std::array<double, 3>{{0.0, 0.0, 0.0}}
    );

    p.is_net_active = j.value("is_net_active", true);
}

struct carla_configuration {
    int seed = 0;
    double carla_timestep = 0.05;
    double sim_time_limit = -1.0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    carla_configuration,
    seed,
    carla_timestep,
    sim_time_limit
)

} // namespace carla_api_base

namespace carla_api {

/* OMNET --> CARLA */
struct init {
    std::string message_type = "INIT";
    double timestamp = 0.0;
    std::string run_id;

    std::list<carla_api_base::init_actor> moving_actors;
    carla_api_base::carla_configuration carla_configuration;

    json user_defined;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    init,
    message_type,
    timestamp,
    run_id,
    moving_actors,
    carla_configuration,
    user_defined
)

/* CARLA --> OMNET */
struct init_completed {
    std::string message_type = "INIT_COMPLETED";
    double initial_timestamp = 0.0;
    std::list<carla_api_base::actor_position> actor_positions;
    int simulation_status = SIM_STATUS_RUNNING;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    init_completed,
    message_type,
    initial_timestamp,
    actor_positions,
    simulation_status
)

/* OMNET --> CARLA */
struct simulation_step {
    std::string message_type = "SIMULATION_STEP";
    double carla_timestep = 0.05;
    double timestamp = 0.0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    simulation_step,
    message_type,
    carla_timestep,
    timestamp
)

/* CARLA --> OMNET */
struct updated_postion {
    std::string message_type = "UPDATED_POSITIONS";
    std::list<carla_api_base::actor_position> actor_positions;
    int simulation_status = SIM_STATUS_RUNNING;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    updated_postion,
    message_type,
    actor_positions,
    simulation_status
)

/* OMNET --> CARLA */
struct generic_message {
    std::string message_type = "GENERIC_MESSAGE";
    double timestamp = 0.0;
    json user_defined;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    generic_message,
    message_type,
    timestamp,
    user_defined
)

/* CARLA --> OMNET */
struct generic_response {
    std::string message_type = "GENERIC_RESPONSE";
    json user_defined;
    int simulation_status = SIM_STATUS_RUNNING;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    generic_response,
    message_type,
    user_defined,
    simulation_status
)

/* OMNET --> CARLA */
struct spawn_actor {
    std::string message_type = "SPAWN_ACTOR";
    carla_api_base::init_actor actor;
    double timestamp = 0.0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    spawn_actor,
    message_type,
    actor,
    timestamp
)

/* CARLA --> OMNET */
struct spawn_completed {
    std::string message_type = "SPAWN_COMPLETED";
    int status = 0;
    std::list<carla_api_base::actor_position> actor_positions;
    int simulation_status = SIM_STATUS_RUNNING;
    double timestamp = 0.0;
    std::string error = "";
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    spawn_completed,
    message_type,
    status,
    actor_positions,
    simulation_status,
    timestamp,
    error
)

} // namespace carla_api
