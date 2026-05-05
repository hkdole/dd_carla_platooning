package carla.phy;

import org.car2x.veins.modules.phy.PhyLayer80211p;

// PHY module for a CARLA-backed 802.11p NIC.
// It extends Veins' PhyLayer80211p and adds support for INET/CARLA mobility updates.
simple CarlaPhyLayer80211p extends PhyLayer80211p
{
    parameters:
        // Delay before registering this NIC with Veins' ConnectionManager.
        // This can help ensure mobility is initialized before the PHY registers.
        // The current C++ schedules registration at simTime(); To use this parameter, change:
        //     scheduleAt(simTime(), regMsg);
        // to:
        //     scheduleAt(simTime() + par("registrationDelay"), regMsg);
        double registrationDelay @unit(s) = default(0.001s);

        // C++ implementation class. Namespace is plexe even though this NED package is carla.phy.
        @class(plexe::CarlaPhyLayer80211p);
}