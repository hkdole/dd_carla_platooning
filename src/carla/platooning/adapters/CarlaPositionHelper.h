#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "carla/platooning/state/PlatoonTypes.h"

namespace carla {

/**
 * Stores the logical platoon state used by the OMNeT++ platooning layer.
 *
 * This helper is a CARLA-backed replacement for the small subset of Plexe's
 * BasePositionHelper needed by the maneuver and control code. It does not read
 * CARLA positions, velocities, lanes, or accelerations. Physical state comes
 * from CarlaInetMobility and V2V beacons; this class stores ids, formation
 * order, platoon metadata, and controller-spacing parameters.
 *
 * The formation vector is ordered from front to back:
 *
 *     [leader, follower1, follower2, ..., tail]
 *
 * A vehicle's predecessor is the vehicle immediately before it in that vector.
 */
class CarlaPositionHelper {
public:
    CarlaPositionHelper() = default;

    // ---------- identity ----------

    // Logical vehicle id used in OMNeT++ messages and platoon formations.
    int getId() const { return id_; }
    void setId(int v) { id_ = v; }

    // External CARLA actor id/name used for tracing and bridge/debug logs.
    const std::string& getExternalId() const { return externalId_; }
    void setExternalId(const std::string& v) { externalId_ = v; }

    // ---------- platoon metadata ----------

    // Logical platoon id carried in maneuver messages.
    int getPlatoonId() const { return platoonId_; }
    void setPlatoonId(int v) { platoonId_ = v; }

    // Returns the current leader id.
    // If a formation is known, the first member is authoritative.
    int getLeaderId() const {
        if (!platoonFormation_.empty()) return platoonFormation_.front();
        return leaderId_;
    }

    // Stores a fallback leader id for cases where formation is not known yet.
    void setLeaderId(int v) { leaderId_ = v; }

    // Logical lane metadata used by maneuver messages.
    // This does not move the CARLA vehicle into a lane by itself.
    int getPlatoonLane() const { return platoonLane_; }
    void setPlatoonLane(int v) { platoonLane_ = v; }

    // Nominal platoon cruise speed in m/s.
    double getPlatoonSpeed() const { return platoonSpeed_; }
    void setPlatoonSpeed(double v) { platoonSpeed_ = v; }

    // Ordered platoon membership, front to back.
    // The order determines leader, position index, and predecessor id.
    const std::vector<int>& getPlatoonFormation() const { return platoonFormation_; }
    void setPlatoonFormation(const std::vector<int>& f)
    {
        platoonFormation_ = f;
        if (!platoonFormation_.empty()) leaderId_ = platoonFormation_.front();
    }

    // Number of vehicles currently known in the local formation view.
    int getPlatoonSize() const { return static_cast<int>(platoonFormation_.size()); }

    // Returns the vehicle id at a formation index.
    // Throws on invalid index because this usually indicates a protocol/state bug.
    int getMemberId(int idx) const
    {
        if (idx < 0 || idx >= static_cast<int>(platoonFormation_.size())) {
            throw std::out_of_range("CarlaPositionHelper::getMemberId invalid index");
        }
        return platoonFormation_[idx];
    }

    // True when this vehicle is the first member of the known formation.
    bool isLeader() const
    {
        return !platoonFormation_.empty() && platoonFormation_.front() == id_;
    }

    // Returns this vehicle's zero-based index in the formation.
    // Returns -1 if the vehicle is not currently in the local formation view.
    int getPositionInPlatoon() const
    {
        auto it = std::find(platoonFormation_.begin(), platoonFormation_.end(), id_);
        if (it == platoonFormation_.end()) return -1;
        return static_cast<int>(std::distance(platoonFormation_.begin(), it));
    }

    // Returns the vehicle directly ahead in the formation.
    // Returns -1 for the leader, unknown vehicles, or empty formations.
    int getPredecessorId() const
    {
        int pos = getPositionInPlatoon();
        if (pos <= 0) return -1;
        return platoonFormation_[pos - 1];
    }

    // ---------- controller metadata ----------

    // Active controller label for this vehicle's logical platoon role.
    // The actual control computation is done by the controller dispatcher.
    ActiveController getController() const { return controller_; }
    void setController(ActiveController c) { controller_ = c; }

    // Standstill spacing in meters.
    // For CDS, this is effectively the desired constant bumper gap.
    double getDistance() const { return standstillDistance_; }
    void setDistance(double d) { standstillDistance_ = d; }

    // Time headway in seconds.
    // Used by CTS-style spacing policies or controllers that depend on speed.
    double getHeadway() const { return headway_; }
    void setHeadway(double h) { headway_ = h; }

private:
    // Logical OMNeT++ vehicle id. -1 means unset.
    int id_ = -1;

    // CARLA-facing actor identifier used for logs/debugging.
    std::string externalId_;

    // Logical platoon metadata; mostly carried in maneuver messages.
    int platoonId_ = -1;
    int leaderId_ = -1;
    int platoonLane_ = -1;
    double platoonSpeed_ = 0.0; // m/s.

    // Ordered front-to-back membership list.
    std::vector<int> platoonFormation_;

    // Controller and spacing metadata mirrored from the app/maneuver layer.
    ActiveController controller_ = ActiveController::UNKNOWN;
    double standstillDistance_ = 5.0; // m.
    double headway_ = 0.5;            // s.
};

} // namespace carla
