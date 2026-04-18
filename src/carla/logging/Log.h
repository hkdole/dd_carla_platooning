#pragma once
#include <sstream>
#include <iomanip>
#include <omnetpp.h>

// You control ordering by writing fields in order in the call site.
// Output:
// [CarlaJoinAtBackApp][sendManeuverMsg] hop=JAB_TO_V2V simulation_time=... actor_id=... ...
#define LOGI(TAG, KV_STREAM) do {                                      \
    std::ostringstream _oss;                                           \
    _oss.setf(std::ios::fixed);                                        \
    _oss << std::setprecision(6);                                      \
    _oss << "[" << (TAG) << "][" << __func__ << "] " << KV_STREAM;     \
    _oss << "\n";                                                      \
    EV_INFO << _oss.str();                                             \
} while(0)