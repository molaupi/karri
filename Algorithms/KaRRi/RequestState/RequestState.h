/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once


#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "Algorithms/KaRRi/Stats/OsmRoadCategoryStats.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Tools/Simd/AlignedVector.h"
#include "DataStructures/Containers/Subset.h"
#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/BaseObjects/PDLocs.h"

namespace karri {

// Holds information relating to a specific request like its pickups and dropoffs and the best known assignment.
    struct RequestState {

        RequestState()
                : originalRequest(),
                    dispatchingTime(INFTY),
                  originalReqDirectDist(-1),
                  minDirectPDDist(-1),
                  bestAssignment(),
                  bestCost(INFTY),
                  notUsingVehicleIsBest(false),
                  notUsingVehicleDist(INFTY) {}


        // Information about current request itself
        Request originalRequest;
        int dispatchingTime; // time at which request is dispatched, i.e., assignment is decided upon. Must be >= originalRequest.requestTime.
        int originalReqDirectDist; // direct distance from origin to destination
        int minDirectPDDist; // smallest distance between any pickup and any dropoff

        int getOriginalReqMaxTripTime() const {
            assert(originalReqDirectDist >= 0);
            return static_cast<int>(InputConfig::getInstance().alpha * static_cast<double>(originalReqDirectDist)) + InputConfig::getInstance().beta;
        }

        int getPassengerArrAtPickup(const PDLoc& pickup) const {
            return dispatchingTime + pickup.walkingDist;
        }

        int getMaxArrTimeAtDropoff(const PDLoc& dropoff) const {
            return originalRequest.requestTime + getOriginalReqMaxTripTime() - dropoff.walkingDist;
        }

        int getMaxDepTimeAtPickup() const {
            return originalRequest.requestTime + InputConfig::getInstance().maxWaitTime;
        }

        // Information about best known assignment for current request

        const Assignment &getBestAssignment() const {
            return bestAssignment;
        }

        const int &getBestCost() const {
            return bestCost;
        }

        bool isNotUsingVehicleBest() const {
            return notUsingVehicleIsBest;
        }

        const int &getNotUsingVehicleDist() const {
            return notUsingVehicleDist;
        }

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const int cost) {
            if (cost < INFTY && (cost < bestCost || (cost == bestCost &&
                                    breakCostTie(asgn, bestAssignment)))) {

                bestAssignment = asgn;
                bestCost = cost;
                notUsingVehicleIsBest = false;
                notUsingVehicleDist = INFTY;
                return true;
            }
            return false;
        }

        void tryNotUsingVehicleAssignment(const int notUsingVehDist, const int travelTimeOfDestEdge) {
            const int cost = CostCalculator::calcCostForNotUsingVehicle(notUsingVehDist, travelTimeOfDestEdge, *this);
            if (cost < bestCost) {
                bestAssignment = Assignment();
                bestCost = cost;
                notUsingVehicleIsBest = true;
                notUsingVehicleDist = notUsingVehDist;
            }
        }

    private:

        // Information about best known assignment for current request
        Assignment bestAssignment;
        int bestCost;
        bool notUsingVehicleIsBest;
        int notUsingVehicleDist;
    };
}