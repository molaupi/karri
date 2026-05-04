#pragma once

#include "Tools/Constants.h"

namespace karri {
    struct WalkingResult {
        int walkingDist = INFTY;
        int cost = INFTY;

        bool isValid() const {
            return walkingDist != INFTY;
        }
    };
}