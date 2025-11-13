#pragma once

#include "Tools/Constants.h"

struct ModeChoiceInput {
    static constexpr double MAX_VAL = static_cast<double>(INFTY);
    double walkTravelTime = MAX_VAL;        // in minutes
    double privateCarTravelTime = MAX_VAL;  // in minutes
    double taxiTravelTime = MAX_VAL;        // in minutes
    double taxiWaitTime = MAX_VAL;          // in minutes
    double taxiAccEgrTime = MAX_VAL;        // in minutes
    double ptTravelTime = MAX_VAL;          // in minutes
    double ptWaitTime = MAX_VAL;            // in minutes
    double ptAccEgrTime = MAX_VAL;          // in minutes

    bool isWalkValid() const {
        return walkTravelTime != MAX_VAL;
    }

    bool isPrivateCarValid() const {
        return privateCarTravelTime != MAX_VAL;
    }

    bool isTaxiValid() const {
        return taxiTravelTime != MAX_VAL && taxiWaitTime != MAX_VAL && taxiAccEgrTime != MAX_VAL;
    }

    bool isPublicTransportValid() const {
        return ptTravelTime != MAX_VAL && ptWaitTime != MAX_VAL && ptAccEgrTime != MAX_VAL;
    }
};