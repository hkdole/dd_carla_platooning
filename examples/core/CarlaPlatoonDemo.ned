package core;

import carla.carlanetpp.CarlanetManager;
import core.CarlaNodeCACC;

import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.modules.obstacle.ObstacleControl;

network CarlaPlatoonDemo
{
    parameters:
        @display("bgb=900,520");
    submodules:
        world: BaseWorldUtility {
            @display("p=120,420;i=block/globe");
        }
        connectionManager: ConnectionManager {
            @display("p=120,340;i=block/wlan");
        }
        obstacles: ObstacleControl {
            @display("p=120,260;i=block/wall");
        }
        manager: carla.carlanetpp.CarlanetManager {
            parameters:
                host = default("localhost");
                port = default(5555);
                protocol = default("tcp");
                simulationTimeStep = default(0.05s);
                networkActiveModuleType  = default("carla_platooning.CarlaNodeCACC");
                networkPassiveModuleType = default("carla_platooning.CarlaNodeCACC");
            @display("p=120,80;i=block/process");
        }

        // Dynamic nodes created by CarlanetManager under this vector
        actors[0]: CarlaNodeCACC {
            @display("p=640,260;is=vs");
        }
    connections allowunconnected:
}