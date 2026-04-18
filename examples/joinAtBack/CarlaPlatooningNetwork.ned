package joinAtBack;

import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.base.modules.BaseWorldUtility;
import inet.common.scenario.ScenarioManager;

import carla.carlanetpp.CarlanetManager;
import joinAtBack.CarlaPlatooningNode;

network CarlaPlatooningNetwork
{
    parameters:
        bool withStatic = default(false);
        int numActors = default(4);
        @display("bgb=900,600");
    submodules:
        world: BaseWorldUtility {
            @display("p=70,60;i=block/globe");
        }
        connectionManager: ConnectionManager {
            @display("p=250,60;i=block/network");
        }
        manager: CarlanetManager {
            @display("p=430,60;i=block/process");
        }
        scenarioManager: ScenarioManager {
            @display("p=60,360");
        }
        actors[numActors]: CarlaPlatooningNode {
            @display("p=250,220,row,90");
        }

    connections allowunconnected:
        for i=0..numActors-1 {
            actors[i].toManager --> manager.controlIn++;
        }
}