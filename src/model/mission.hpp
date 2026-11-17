#pragma once
#include "model/types.hpp"

namespace chronos {

enum class MissionStatus : uint8_t {
    PENDING = 0, ASSIGNED, IN_TRANSIT, COMPLETED, FAILED, DELAYED
};

struct Mission {
    Id id = NONE;
    Id origin = NONE;
    Id dest = NONE;
    int resource = FOOD;
    double amount = 0;
    double release = 0;        // earliest departure hour
    double deadline = 0;       // latest arrival hour
    int priority = 1;          // 1 (low) .. 5 (emergency)
    double importance = 0;
    uint32_t required_caps = CAP_CARGO;
    MissionStatus status = MissionStatus::PENDING;
};

}  // namespace chronos
