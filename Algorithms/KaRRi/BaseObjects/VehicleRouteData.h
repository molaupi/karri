//
// Created by tim on 11.03.24.
//
#pragma once

#include <vector>

namespace karri {
    struct VehicleRouteData {

        explicit VehicleRouteData(const Vehicle &veh): veh(veh) {};

        const Vehicle &veh;

        std::vector<int> stopIds;
        std::vector<int> stopLocations;
        std::vector<int> schedArrTimes;
        std::vector<int> schedDepTimes;
        std::vector<int> maxArrTimes;
        std::vector<int> occupancies;
        std::vector<int> vehWaitTimesPrefixSum;
        std::vector<int> vehWaitTimesUntilDropoffsPrefixSum;
        std::vector<int> numDropoffsPrefixSum;
        std::vector<int> leeways;

        // pickedUpRequest[pickupIndex[i] ... pickupIndex[i+1]) are the requests picked up at the i-th stop
        std::vector<int> pickUpIndex;
        std::vector<int> pickedUpRequests;
        std::vector<int> dropOffIndex;
        std::vector<int> droppedOffRequests;
    };
}