package carla.apps;

import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple CarlaPlexeApp extends DemoBaseApplLayer
{
    parameters:
        // CRITICAL: do NOT inherit DemoBaseApplLayer's @class()
        // This fixes: "Class veins::DemoBaseApplLayer not found"
        @class(carla::CarlaPlexeApp);

        string actor_id = default("");
        string platoon_role = default("follower");   // leader | follower | joiner

        int node_id      = default(-1);
        int platoon_id = default(0);
        int leader_id  = default(0);

        // leader-only: e.g. "0,1,2,3"
        string initial_formation = default("");

        // NOTE: beaconInterval is inherited from DemoBaseApplLayer (do NOT redeclare)
        double control_interval @unit(s) = default(0.05s);

        double gap_min  @unit(m) = default(5m);
        double approach_spacing @unit(m) = default(15m);
        double k_gap    = default(0.2);
        double k_dv     = default(0.7);

        double platoon_speed @unit(mps) @mutable = default(27.777777mps);
        double k_speed_p @mutable = default(0.7);

        double a_min = default(-4.0);
        double a_max = default( 2.5);

        double start_maneuver_at @unit(s) = default(10s);
        double join_req_retry    @unit(s) = default(0.5s);
        int    join_req_max_tries = default(20);

        double approach_delta_v @unit(mps) = default(8.333333mps);
        double in_pos_slack     @unit(m)   = default(11m);
        double vehicle_length  @unit(m)   = default(4.5m);

        double max_age @unit(s) = default(0.5s);

        bool debug = default(false);

        // For braking scenario only
        // --- leader brake profile ---
        double leader_brake_at @unit(s) = default(-1s);          // <0 disables
        double leader_brake_speed @unit(mps) = default(0mps);    // target speed during brake window
        double leader_brake_hold_for @unit(s) = default(0s);     // 0 => hold forever
        double leader_post_brake_speed @unit(mps) = default(-1mps); // <0 => return to platoon_speed

        @display("i=block/app;is=vl;tt=%platoon_role%/%node_id%");
}
