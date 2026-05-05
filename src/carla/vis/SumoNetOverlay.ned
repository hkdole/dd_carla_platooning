// file: ~/workspace/plexe/src/org/car2x/plexe/vis/SumoNetOverlay.ned 
package carla.vis;

simple SumoNetOverlay
{
    parameters:
        xml   netXml;

        // Manual transform (mutable so code can write final values)
        double scale            = default(1)    @mutable(true);
        double rotDeg           = default(0)    @mutable(true);
        double offsetX @unit(m) = default(0m)   @mutable(true);
        double offsetY @unit(m) = default(0m)   @mutable(true);
        bool   flipX            = default(false)@mutable(true);
        bool   flipY            = default(true) @mutable(true);

        // Auto-fit OR Unit-locked (exact SUMO meters on canvas)
        bool   autoFit          = default(false);
        bool   unitLock         = default(true);   // when true: scale=1, offsets from convBoundary
        double fitWidth  @unit(m) = default(395m);
        double fitHeight @unit(m) = default(330m);
        double fitMargin @unit(m) = default(0m);
        bool   preserveAspect = default(true);
        bool   antiAlias = default(true);

        // Published SUMO metadata
        double sumoNetOffsetX @unit(m) = default(0m) @mutable(true);
        double sumoNetOffsetY @unit(m) = default(0m) @mutable(true);
        double netMinX @unit(m) = default(0m) @mutable(true);
        double netMinY @unit(m) = default(0m) @mutable(true);
        double netMaxX @unit(m) = default(0m) @mutable(true);
        double netMaxY @unit(m) = default(0m) @mutable(true);

        @display("i=block/map");
}
