// BridgeApp.ned
package carla.apps;

simple BridgeApp
{
    parameters:
        @class(BridgeApp);

        // CONTROL id to Python (veh0..vehN)
        string actor_id;

        // Max CONTROL cadence to manager/Python
        int send_every_ms = default(50);

        @display("i=block/app;is=vl;tt=%actor_id%");
    gates:
        input  lowerLayerIn;
        output lowerLayerOut;

        // JSON CONTROL → CarlanetManager.controlIn[]
        output toManager;
}
