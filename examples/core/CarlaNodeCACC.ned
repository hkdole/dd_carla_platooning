package core;

import carla.carlanetpp.CarlaInetMobility;
import carla.nic.CarlaNic80211p;
import carla.apps.CarlaPlexeApp;
import carla.apps.BridgeApp;

module CarlaNodeCACC
{
    parameters:
        string roleName = default("car");
        @display("i=misc/car;is=vl;tt=%roleName%");
    gates:
        output toManager;
    submodules:
        mobility: CarlaInetMobility {
            @display("p=80,60;i=block/cogwheel");
        }

        nic: CarlaNic80211p {
            parameters:
                connectionManagerName = default("connectionManager");
            @display("p=240,60;i=block/wifirouter");
        }

        appl: CarlaPlexeApp {
            parameters:
                node_id = default(parent.node_id);
            @display("p=360,60;i=block/app");
        }

        bridge: BridgeApp {
            parameters:
                actor_id = default(parent.actor_id);
            @display("p=140,130;i=block/app");
        }
    connections allowunconnected:
        // App (Plexe) ↔ NIC
        appl.lowerLayerOut --> nic.upperLayerIn;
        appl.lowerLayerIn  <-- nic.upperLayerOut;

        appl.lowerControlOut --> nic.upperControlIn;
        appl.lowerControlIn  <-- nic.upperControlOut;

        // Bridge → Manager (your existing control path)
        bridge.toManager --> toManager;

}
