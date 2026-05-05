package carla.nic;

import org.car2x.veins.modules.nic.INic80211p;
import org.car2x.veins.modules.mac.ieee80211p.Mac1609_4;
import carla.phy.CarlaPhyLayer80211p;

// 802.11p network interface card for CARLA-backed vehicles.
//
// This module wires together:
//   - Mac1609_4: Veins IEEE 1609.4 / 802.11p MAC layer
//   - CarlaPhyLayer80211p: Veins PHY layer adapted to CARLA/INET mobility
//
// Platooning protocol logic does not belong here. This module only defines how
// packets move between the application, MAC, PHY, and radio channel.
module CarlaNic80211p like INic80211p
{
    parameters:
        // Name of the Veins ConnectionManager module.
        // The PHY uses it to register the NIC and query radio connectivity/interference.
        string connectionManagerName = default("connectionManager");

    gates:
        // Packet input from the application layer into the NIC.
        // Example path: CarlaGeneralPlatooningApp -> upperLayerIn -> MAC -> PHY.
        input upperLayerIn;

        // Packet output from the NIC up to the application layer.
        // Example path: PHY -> MAC -> upperLayerOut -> CarlaGeneralPlatooningApp.
        output upperLayerOut;

        // Control output from the NIC to upper layers.
        // Carries MAC/PHY control indications when used by Veins modules.
        output upperControlOut;

        // Control input from upper layers into the NIC.
        // Carries MAC/PHY control requests when used by Veins modules.
        input upperControlIn;

        // Direct radio input gate used by Veins sendDirect delivery.
        // The ConnectionManager/radio channel injects received frames here.
        input radioIn;

    submodules:
        // CARLA-aware 802.11p PHY layer.
        // This extends Veins PhyLayer80211p and updates antenna position from CarlaInetMobility.
        phy80211p: CarlaPhyLayer80211p {
            @display("p=69,218;i=block/process_s");
        }

        // Veins IEEE 1609.4 MAC layer.
        // Handles channel access, MAC addressing, and forwarding between app and PHY.
        mac1609_4: Mac1609_4 {
            @display("p=69,82");
        }

    connections:
        // Radio frames delivered by the Veins channel enter the PHY here.
        radioIn --> phy80211p.radioIn;

        // MAC-to-PHY control path.
        mac1609_4.lowerControlOut --> phy80211p.upperControlIn;

        // MAC-to-PHY data path for outgoing packets.
        mac1609_4.lowerLayerOut --> phy80211p.upperLayerIn;

        // PHY-to-MAC data path for received packets.
        phy80211p.upperLayerOut --> mac1609_4.lowerLayerIn;

        // PHY-to-MAC control path.
        phy80211p.upperControlOut --> mac1609_4.lowerControlIn;

        // Upper-layer control requests enter the MAC.
        mac1609_4.upperControlIn <-- upperControlIn;

        // Upper-layer packets enter the MAC.
        mac1609_4.upperLayerIn <-- upperLayerIn;

        // Received packets leave the MAC toward the application.
        mac1609_4.upperLayerOut --> upperLayerOut;

        // MAC control indications leave toward the application.
        mac1609_4.upperControlOut --> upperControlOut;
}
