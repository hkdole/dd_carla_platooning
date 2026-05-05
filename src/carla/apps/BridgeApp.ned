package carla.apps;

// BridgeApp is the boundary between OMNeT++ control logic and CarlanetManager.
// It listens for control signals emitted by the platooning app, packages the latest
// control intent as JSON, and sends it to CarlanetManager for delivery to pyCARLANeT.
simple BridgeApp
{
    parameters:
        // C++ implementation class for this NED module.
        @class(BridgeApp);

        // CARLA actor identifier used in CONTROL JSON messages.
        // This must match the vehicle name/id expected by pyCARLANeT, such as "veh0".
        string actor_id;

        // Minimum time between CONTROL messages sent to CarlanetManager.
        // 50 ms matches a 20 Hz control loop when simulationTimeStep/control_interval is 0.05 s.
        int send_every_ms = default(50);

        @display("i=block/app;is=vl;tt=%actor_id%");
    gates:
        // Optional lower-layer input gate. Present for module compatibility.
        input  lowerLayerIn;

        // Optional lower-layer output gate. Present for module compatibility.
        output lowerLayerOut;

        // Sends JSON CONTROL messages to CarlanetManager.controlIn[].
        // CarlanetManager forwards these to the Python CARLA bridge in the next ZMQ step.
        output toManager;
}