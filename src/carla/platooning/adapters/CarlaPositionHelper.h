#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "carla/platooning/state/PlatoonTypes.h"

namespace carla {

/**
 * CARLA-backed replacement for the subset of Plexe BasePositionHelper
 * needed by the platooning/maneuver layer.
 */
class CarlaPositionHelper {
public:
    CarlaPositionHelper() = default;

    // ---------- identity ----------
    int getId() const { return id_; }
    void setId(int v) { id_ = v; }

    const std::string& getExternalId() const { return externalId_; }
    void setExternalId(const std::string& v) { externalId_ = v; }

    // ---------- platoon metadata ----------
    int getPlatoonId() const { return platoonId_; }
    void setPlatoonId(int v) { platoonId_ = v; }

    int getLeaderId() const {
        if (!platoonFormation_.empty()) return platoonFormation_.front();
        return leaderId_;
    }
    void setLeaderId(int v) { leaderId_ = v; }

    int getPlatoonLane() const { return platoonLane_; }
    void setPlatoonLane(int v) { platoonLane_ = v; }

    double getPlatoonSpeed() const { return platoonSpeed_; }
    void setPlatoonSpeed(double v) { platoonSpeed_ = v; }

    const std::vector<int>& getPlatoonFormation() const { return platoonFormation_; }
    void setPlatoonFormation(const std::vector<int>& f)
    {
        platoonFormation_ = f;
        if (!platoonFormation_.empty()) leaderId_ = platoonFormation_.front();
    }

    int getPlatoonSize() const { return static_cast<int>(platoonFormation_.size()); }

    int getMemberId(int idx) const
    {
        if (idx < 0 || idx >= static_cast<int>(platoonFormation_.size())) {
            throw std::out_of_range("CarlaPositionHelper::getMemberId invalid index");
        }
        return platoonFormation_[idx];
    }

    bool isLeader() const
    {
        return !platoonFormation_.empty() && platoonFormation_.front() == id_;
    }

    int getPositionInPlatoon() const
    {
        auto it = std::find(platoonFormation_.begin(), platoonFormation_.end(), id_);
        if (it == platoonFormation_.end()) return -1;
        return static_cast<int>(std::distance(platoonFormation_.begin(), it));
    }

    int getPredecessorId() const
    {
        int pos = getPositionInPlatoon();
        if (pos <= 0) return -1;
        return platoonFormation_[pos - 1];
    }

    // ---------- controller metadata ----------
    ActiveController getController() const { return controller_; }
    void setController(ActiveController c) { controller_ = c; }

    double getDistance() const { return standstillDistance_; }
    void setDistance(double d) { standstillDistance_ = d; }

    double getHeadway() const { return headway_; }
    void setHeadway(double h) { headway_ = h; }

private:
    int id_ = -1;
    std::string externalId_;

    int platoonId_ = -1;
    int leaderId_ = -1;
    int platoonLane_ = -1;
    double platoonSpeed_ = 0.0;
    std::vector<int> platoonFormation_;

    ActiveController controller_ = ActiveController::UNKNOWN;
    double standstillDistance_ = 5.0;
    double headway_ = 0.5;
};

} // namespace carla