package carla.phy;

import org.car2x.veins.modules.phy.PhyLayer80211p;

simple CarlaPhyLayer80211p extends PhyLayer80211p
{
    parameters:
        double registrationDelay @unit(s) = default(0.001s); // 1ms
        @class(plexe::CarlaPhyLayer80211p);
}
