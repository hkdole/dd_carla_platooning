package carlanet;

simple CarlanetManager
{
    parameters:
        @class(CarlanetManager);

        string host = default("localhost");
        string protocol = default("tcp");
        int port = default(5555);
        int communicationTimeoutms = default(1000);

        double simulationTimeStep @unit(s) = default(0.05s);
        int carlaSeed = default(-1);

        object extraInitParams = default(parseJSON("{}"));

        bool enableLateSpawn = default(true);
        string lateSpawnActorId = default("veh4");
        string lateSpawnActorType = default("car");
        string lateSpawnActorConfig = default("{}");
        double lateSpawnTime @unit(s) = default(10s);

        string networkActiveModuleType;
        string networkPassiveModuleType = default("inet.node.base.NodeBase");

        @display("i=block/cogwheel");

    gates:
        input controlIn[];
}
