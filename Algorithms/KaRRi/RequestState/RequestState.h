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
#include "Algorithms/KaRRi/RequestState/FindPDLocsInRadiusQuery.h"

namespace karri {

// Holds information relating to a specific request like its pickups and dropoffs.
    struct RequestState {

        RequestState()
                : originalRequest(),
                  originalReqDirectDist(-1),
                  minDirectPDDist(-1),
                  pickups(),
                  dropoffs() {}

        // Information about current request itself
        Request originalRequest;
        int originalReqDirectDist;
        int minDirectPDDist;

        int directWalkingDist;

        std::vector<PDLoc> pickups;
        std::vector<PDLoc> dropoffs;

        int numPickups() const {
            return pickups.size();
        }

        int numDropoffs() const {
            return dropoffs.size();
        }

        // Shorthand for requestTime
        int now() const {
            return originalRequest.requestTime;
        }

        int getOriginalReqMaxTripTime() const {
            assert(originalReqDirectDist >= 0);
            return static_cast<int>(InputConfig::getInstance().alpha * static_cast<double>(originalReqDirectDist)) + InputConfig::getInstance().beta;
        }

        int getPassengerArrAtPickup(const int pickupId) const {
            assert(pickupId < numPickups());
            return originalRequest.requestTime + pickups[pickupId].walkingDist;
        }

        int getMaxPDTripTime(const int pickupId, const int dropoffId) const {
            assert(pickupId < numPickups() && dropoffId < numDropoffs());
            assert(originalReqDirectDist >= 0);
            return getOriginalReqMaxTripTime() - (pickups[pickupId].walkingDist + dropoffs[dropoffId].walkingDist);
        }

        int getMaxArrTimeAtDropoff(const int pickupId, const int dropoffId) const {
            assert(pickupId < numPickups() && dropoffId < numDropoffs());
            return getPassengerArrAtPickup(pickupId) + getMaxPDTripTime(pickupId, dropoffId);
        }

        int getMaxDepTimeAtPickup() const {
            return originalRequest.requestTime + InputConfig::getInstance().maxWaitTime;
        }

        void reset() {
            originalRequest = {};
            originalReqDirectDist = INFTY;
            minDirectPDDist = INFTY;
            pickups.clear();
            dropoffs.clear();
            directWalkingDist = INFTY;
        }
    };
}