#pragma once

#include "Tools/Constants.h"

struct PTJourneyData {
    static constexpr double MAX_VAL = static_cast<double>(INFTY);
    double travelTimeMinutes = MAX_VAL; // in minutes
    double waitTimeMinutes = MAX_VAL;   // in minutes
    double accessEgressTimeMinutes = MAX_VAL; // in minutes

    bool isValid() const {
        return travelTimeMinutes != MAX_VAL && waitTimeMinutes != MAX_VAL && accessEgressTimeMinutes != MAX_VAL;
    }
};