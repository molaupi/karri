//
// Created by tim on 16.03.24.
//

#pragma once

#include "Assignment.h"

namespace karri {

    struct AssignmentData {
        int requestId;
        int requestTime;
        int cost;
        int walkingTimeToPickup;
        int walkingTimeFromDropoff;
        int assignedVehicleId;
        bool isUsingVehicle;
        int tripTime;
        int waitTime;
        int directDist;
    };

    struct ChangesWrapper {
        AssignmentData initialAssignment;
        std::vector<AssignmentData> reassignments;
        std::vector<int> affectedVehicles;
        int costsSaved = 0;
    };
}