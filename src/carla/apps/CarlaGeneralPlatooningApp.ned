package carla.apps;

// Application-layer platooning module for one CARLA vehicle.
// Runs on top of Veins 802.11p, sends/receives V2V platooning messages,
// computes longitudinal control intent, and emits signals consumed by BridgeApp.
simple CarlaGeneralPlatooningApp extends org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer
{
    parameters:
        // CARLA actor identifier used by the Python bridge and logs.
        string actor_id @mutable = default(parent.actor_id);

        // What type of controller the protocol uses
        string controller_type = default("CACC");

        // Logical platoon role for this vehicle: "leader", "follower", "joiner", or none.
        string platoon_role @mutable = default("follower");

        // Logical OMNeT++/platoon vehicle id. This is not necessarily the CARLA actor id.
        int node_id @mutable = default(parent.node_id);

        // Logical platoon id carried in maneuver messages.
        int platoon_id @mutable = default(0);

        // Logical vehicle id of the platoon leader.
        int leader_id @mutable = default(0);

        // Comma-separated formation order, e.g. "0,1,2,3" means 0 leads, 1 follows 0, etc.
        string initial_formation @mutable = default("");

        // Requested join position. -1 means use the maneuver's default placement.
        int join_position @mutable = default(-1);

        // Nominal steady platoon speed used by followers/joiners when no mode override exists.
        double nominal_platoon_speed @mutable @unit(mps) = default(27.777777mps);

        // Leader cruise-control speed target. Followers should not independently consume this as a shared target.
        double leader_target_speed @mutable @unit(mps) = default(27.777777mps);

        // beaconInterval is inherited from DemoBaseApplLayer.
        // It controls how often this app broadcasts V2V beacons.

        // Longitudinal controller update period. Usually matches CARLA fixed delta time.
        double control_interval @mutable @unit(s) = default(0.05s);

        // Maximum accepted age of neighbor beacon data before it is treated as stale.
        double max_age @mutable @unit(s) = default(0.5s);

        // Time-headway parameter for constant-time-spacing style controllers.
        double headway @mutable @unit(s) = default(0.5s);

        // Minimum bumper-to-bumper gap. For CDS, this is the target gap.
        double gap_min @mutable @unit(m) = default(5m);

        // Gain on spacing error: positive when the vehicle is too far from its predecessor.
        double k_gap @mutable = default(0.2);

        // Gain on relative speed error: predecessor speed minus ego speed.
        double k_dv @mutable = default(0.7);

        // Cruise-control proportional gain on speed error.
        double k_speed_p @mutable = default(0.7);

        // Feed-forward gain on predecessor/leader controller acceleration from beacons.
        double k_acc_ff @mutable = default(1.0);

        // Gain on speed difference relative to the leader.
        double k_leader_dv @mutable = default(0.15);

        // Converts gap error into a debug/reference target-speed bias.
        double k_gap_speed @mutable = default(0.25);

        // Maximum allowed closing-speed bias created from gap error.
        double max_closure_speed @mutable @unit(mps) = default(5mps);

        // Minimum controller acceleration command, usually braking.
        double a_min @mutable = default(-4.0);

        // Maximum controller acceleration command.
        double a_max @mutable = default(2.5);

        // Vehicle length used to convert center-to-center distance into bumper gap.
        double vehicle_length @mutable @unit(m) = default(4.5m);

        // Time when a joiner starts its join maneuver. -1 disables automatic start.
        double start_maneuver_at @mutable @unit(s) = default(-1s);

        // Extra speed added while a joiner approaches the platoon during MoveToPosition.
        double approach_delta_v @mutable @unit(mps) = default(0mps);

        // Distance tolerance used to decide when a joiner is close enough to send MoveToPositionAck.
        double in_pos_slack @mutable @unit(m) = default(0.5m);

        // Plexe/SUMO CACC damping-ratio parameter, preserved for controller parity and analysis labels.
        double caccXi @mutable = default(1.0);

        // Plexe/SUMO CACC natural-frequency parameter, preserved for controller parity and analysis labels.
        double caccOmegaN @mutable = default(0.2);

        // Parameters needed to run the distributed protocol
        double alpha @mutable = default(0.6);
        double deviation @mutable = default(0.4);
        double range @mutable = default(400.0);
        double desired_speed @mutable = default(10.0);

        // Lateral control
        bool lateral_control_enabled = default(false);
        bool multilane_join_enabled = default(false);
        double lane_change_penalty = default(5.0);
        double lane_width_m @unit(m) = default(3.5m);

        // Lane spawn
        int spawn_lane_index = default(1);   // 0=innermost(lane -1), 1=default(lane -2), 2=(lane -3), 3=outermost(lane -4)

        // EXTRA FOR TESTING PURPOSES ONLY
        double brake_at @mutable @unit(s) = default(-1s); // Tells car to brake at a certain time

        // C++ implementation class for this NED module.
        @class(carla::CarlaGeneralPlatooningApp);

        // Result signals emitted by CarlaGeneralPlatooningApp.cc.
        @signal[speed](type=double);                    // Ego scalar speed in m/s.
        @signal[acceleration](type=double);             // Measured ego longitudinal acceleration in m/s^2.
        @signal[controllerAcceleration](type=double);   // Controller-requested acceleration in m/s^2.
        @signal[distance](type=double);                 // Bumper gap to predecessor in m; -1 when unavailable.
        @signal[relativeSpeed](type=double);            // Predecessor speed minus ego speed in m/s.

        // OMNeT++ result vectors used by parser/R analysis scripts.
        @statistic[speed](title="vehicle speed"; source=speed; record=vector);
        @statistic[acceleration](title="vehicle acceleration"; source=acceleration; record=vector);
        @statistic[controllerAcceleration](title="controller acceleration"; source=controllerAcceleration; record=vector);
        @statistic[distance](title="gap to predecessor"; source=distance; record=vector);
        @statistic[relativeSpeed](title="relative speed to predecessor"; source=relativeSpeed; record=vector);

        // Sinusoidal movement
        bool sinusoidal_speed = default(false);
        double sinusoidal_amplitude @unit(mps) = default(3.0mps);
        double sinusoidal_period @unit(s) = default(20.0s);
        double sinusoidal_start_at @unit(s) = default(10.0s);

        // Exit 
        bool exit_enabled = default(false);
        double exit_at @unit(s) = default(-1s);
}
