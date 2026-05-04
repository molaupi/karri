#pragma once
#include <Algorithms/KaRRi/BaseObjects/Assignment.h>

#include "Algorithms/KaRRi/BaseObjects/BestAsgnType.h"
#include "Tools/Constants.h"

namespace karri {
    struct TaxiResult {
        TaxiResult() noexcept = default;

        int cost = INFTY;
        int inVehicleTime = INFTY; // in tenths of seconds
        int waitTime = INFTY; // in tenths of seconds
        int walkTime = INFTY; // in tenths of seconds
        BestAsgnType asgnType = INVALID;

        bool isValid() const {
            return cost != INFTY && inVehicleTime != INFTY && waitTime != INFTY && walkTime != INFTY && asgnType != INVALID;
        }
    };
}
