package carla.carlanetpp;

import inet.mobility.base.MobilityBase;

simple CarlaInetMobility extends MobilityBase
{
    parameters:
        @class(CarlaInetMobility);
        @display("i=block/cogwheel");
        @signal[mobilityStateChanged](type=inet::MobilityBase);
        bool initFromDisplayString = default(true);

        // Actor info
        string carlaActorType = default("car");
        object carlaActorConfiguration = default(parseJSON("{}"));

        // Canvas transform (STATIC; no overlay)
        double scale = default(1);
        double rotDeg = default(0);
        double offsetX @unit(m) = default(0m);
        double offsetY @unit(m) = default(0m);
        bool flipX = default(false);
        bool flipY = default(false); // identity by default

        // Optional net offset & alignment (STATIC)
        double sumoNetOffsetX @unit(m) = default(0m);
        double sumoNetOffsetY @unit(m) = default(0m);
        int    netOffsetSign = default(0);     // 0=disable, +1=subtract, -1=add
        bool   swapXY = default(false);
        double preRotDeg = default(0);

        // Global unit scale
        double carlaToSumoScale = default(1);

        // Final canvas nudge
        double deltaCanvasX @unit(m) = default(0m);
        double deltaCanvasY @unit(m) = default(0m);
}
