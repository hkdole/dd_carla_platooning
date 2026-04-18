package joinAtBack;

import carla.carlanetpp.CarlaInetMobility;
import carla.nic.CarlaNic80211p;
import carla.apps.CarlaGeneralPlatooningApp;
import carla.apps.BridgeApp;

module CarlaPlatooningNode
{
    parameters:
        string actor_id @mutable = ("veh" + string(index));
        string platoon_role @mutable = default(index == 0 ? "leader" : "follower"); // leader|follower|joiner|none
        int node_id @mutable = default(index);

        int platoon_id @mutable = default(0);
        int leader_id @mutable = default(0);
        string initial_formation @mutable = default("");
        int join_position @mutable = default(-1);

        @display("i=misc/car;is=vl;tt=%actor_id%/%platoon_role%/%node_id%");
    gates:
        output toManager;
    submodules:
        mobility: CarlaInetMobility {
            @display("p=80,60;i=block/cogwheel");
        }

        nic: CarlaNic80211p {
            @display("p=240,60;i=block/wifirouter");
        }

        appl: CarlaGeneralPlatooningApp {
            parameters:
                actor_id          = default(parent.actor_id);
                platoon_role      = default(parent.platoon_role);
                node_id           = default(parent.node_id);

                platoon_id        = default(parent.platoon_id);
                leader_id         = default(parent.leader_id);
                initial_formation = default(parent.initial_formation);
                join_position     = default(parent.join_position);
            @display("p=360,60;i=block/app");
        }

        bridge: BridgeApp {
            parameters:
                actor_id = default(parent.actor_id);
            @display("p=140,130;i=block/app");
        }

    connections allowunconnected:
        appl.lowerLayerOut --> nic.upperLayerIn;
        appl.lowerLayerIn  <-- nic.upperLayerOut;

        appl.lowerControlOut --> nic.upperControlIn;
        appl.lowerControlIn  <-- nic.upperControlOut;

        bridge.toManager --> toManager;
}