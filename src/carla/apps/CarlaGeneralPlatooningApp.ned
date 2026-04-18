package carla.apps;

simple CarlaGeneralPlatooningApp extends org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer
{
    parameters:
        string actor_id = default(parent.actor_id);
        string platoon_role = default("follower");
        int node_id = default(parent.node_id);
        int platoon_id = default(0);
        int leader_id = default(0);

        string initial_formation = default("");
        int join_position = default(-1);

        double nominal_platoon_speed @unit(mps) = default(27.777777mps);
        double leader_target_speed @unit(mps) = default(27.777777mps);

        // beaconInterval is inherited from DemoBaseApplLayer
        double control_interval @unit(s) = default(0.05s);
        double max_age @unit(s) = default(0.5s);

        double headway @unit(s) = default(0.5s);
        double gap_min @unit(m) = default(5m);

        double k_gap = default(0.2);
        double k_dv = default(0.7);
        double k_speed_p = default(0.7);

        // follower / joiner feedforward terms
        double k_acc_ff = default(1.0);
        double k_leader_dv = default(0.15);
        double k_gap_speed = default(0.25);
        double max_closure_speed @unit(mps) = default(5mps);

        double a_min = default(-4.0);
        double a_max = default(2.5);

        double vehicle_length @unit(m) = default(4.5m);
        double start_maneuver_at @unit(s) = default(-1s);

        // carried through to exported comparison schema, matching PLEXE analysis labels
        double caccXi = default(1.0);
        double caccOmegaN = default(0.2);

        @class(carla::CarlaGeneralPlatooningApp);

        // PLEXE-style exported traces
        @signal[speed](type=double);
        @signal[acceleration](type=double);
        @signal[controllerAcceleration](type=double);
        @signal[distance](type=double);
        @signal[relativeSpeed](type=double);

        // record them as vectors so OMNeT writes them into .vec output
        @statistic[speed](title="vehicle speed"; source=speed; record=vector);
        @statistic[acceleration](title="vehicle acceleration"; source=acceleration; record=vector);
        @statistic[controllerAcceleration](title="controller acceleration"; source=controllerAcceleration; record=vector);
        @statistic[distance](title="gap to predecessor"; source=distance; record=vector);
        @statistic[relativeSpeed](title="relative speed to predecessor"; source=relativeSpeed; record=vector);
}