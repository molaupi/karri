//
// Created by tim on 01.12.23.
//

#pragma once

#include "DataStructures/Utilities/IteratorRange.h"

namespace karri {

    // Scheduled stop interface for event simulation
    struct ScheduledStop {
        int stopId;
        int arrTime;
        int depTime;
        int occupancyInFollowingLeg;
        ConstantVectorRange<int> requestsPickedUpHere;
        ConstantVectorRange<int> requestsDroppedOffHere;
    };
}
