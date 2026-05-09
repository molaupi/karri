#pragma once

#include "Tools/Constants.h"

namespace karri {
    struct CarResult {
        int carDist = INFTY;

        bool isValid() const {
            return carDist != INFTY;
        }
    };
}
