namespace carla_api {

struct spawn_actor {
    std::string message_type = "SPAWN_ACTOR";
    carla_api_base::init_actor actor;
    double timestamp = 0.0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(spawn_actor, message_type, actor, timestamp)

struct spawn_completed {
    std::string message_type = "SPAWN_COMPLETED";
    int status = 0; // 0=OK
    std::list<carla_api_base::actor_position> actor_positions; // keep list
    int simulation_status = 0; // REQUIRED by your C++ receiveFromCarla()
    double timestamp = 0.0;
    std::string error; // optional; Python can omit
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(spawn_completed, message_type, status, actor_positions, simulation_status, timestamp, error)

} // namespace carla_api
