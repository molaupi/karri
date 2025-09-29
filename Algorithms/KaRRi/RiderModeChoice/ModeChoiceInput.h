#pragma once

#include "Tools/Constants.h"

struct ModeChoiceInput {
    static constexpr double MAX_VAL = static_cast<double>(INFTY);
    double walkTravelTime = MAX_VAL;        // in minutes
    double privateCarTravelTime = MAX_VAL;  // in minutes
    double taxiTravelTime = MAX_VAL;        // in minutes
    double taxiWaitTime = MAX_VAL;          // in minutes
    double taxiAccEgrTime = MAX_VAL;        // in minutes
};