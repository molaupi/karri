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
#include "Algorithms/KaRRi/RequestState/FindPDLocsInRadiusQuery.h"

namespace karri {

// Holds information relating to a specific request like its pickups and dropoffs and the best known assignment.
template<typename CostCalculatorT>
    struct RequestState {

        RequestState(const CostCalculatorT &calculator, const InputConfig &inputConfig, const RouteStateDataType type)
                : type(type),
                  originalRequest(),
                  originalReqDirectDist(-1),
                  minDirectPDDist(-1),
                  pickups(),
                  dropoffs(),
                  calculator(calculator),
                  inputConfig(inputConfig){}


        ~RequestState() {
            auto &roadCatLogger = LogManager<std::ofstream>::getLogger(karri::stats::OsmRoadCategoryStats::LOGGER_NAME,
                                                                       "type," +
                                                                       karri::stats::OsmRoadCategoryStats::getLoggerCols());
            roadCatLogger << "all_pd_locs, " << allPDLocsRoadCatStats.getLoggerRow() << "\n";
            roadCatLogger << "chosen_pd_locs, " << chosenPDLocsRoadCatStats.getLoggerRow() << "\n";
        }


        // Information about current request itself
        RouteStateDataType type;

        Request originalRequest;
        int originalReqDirectDist;
        int minDirectPDDist;

        std::vector<PDLoc> pickups;
        std::vector<PDLoc> dropoffs;

        int blockedVehId = INVALID_ID;


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
            return static_cast<int>(inputConfig.alpha * static_cast<double>(originalReqDirectDist)) + inputConfig.beta;
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
            return originalRequest.requestTime + inputConfig.maxWaitTime;
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

        bool tryAssignment(const Assignment &asgn) {
            const auto cost = calculator.calc(asgn, *this);
            return tryAssignmentWithKnownCost(asgn, cost);
        }

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const int cost) {
            assert(calculator.calc(asgn, *this) == cost);

            if (asgn.vehicle->vehicleId == blockedVehId) return false;

            if (cost < bestCost || (cost == bestCost &&
                                     breakCostTie(asgn, bestAssignment))) {

                bestAssignment = asgn;
                bestCost = cost;
                notUsingVehicleIsBest = false;
                notUsingVehicleDist = INFTY;
                return true;
            }
            return false;
        }

        void tryNotUsingVehicleAssignment(const int notUsingVehDist, const int travelTimeOfDestEdge) {
            const int cost = CostCalculatorT::calcCostForNotUsingVehicle(notUsingVehDist, travelTimeOfDestEdge, *this,
                                                                        inputConfig);

            if (cost < bestCost) {
                bestAssignment = Assignment();
                bestCost = cost;
                notUsingVehicleIsBest = true;
                notUsingVehicleDist = notUsingVehDist;
            }
        }

        stats::DispatchingPerformanceStats &stats() {
            return perfStats;
        }

        const stats::DispatchingPerformanceStats &stats() const {
            return perfStats;
        }

        stats::OsmRoadCategoryStats &allPDLocsRoadCategoryStats() {
            return allPDLocsRoadCatStats;
        }

        stats::OsmRoadCategoryStats &chosenPDLocsRoadCategoryStats() {
            return chosenPDLocsRoadCatStats;
        }

        void reset() {
            perfStats.clear();

            originalRequest = {};
            originalReqDirectDist = INFTY;
            minDirectPDDist = INFTY;
            pickups.clear();
            dropoffs.clear();

            bestAssignment = Assignment();
            bestCost = INFTY;
            notUsingVehicleIsBest = false;
            notUsingVehicleDist = INFTY;
        }

    private:

        stats::DispatchingPerformanceStats perfStats;
        stats::OsmRoadCategoryStats allPDLocsRoadCatStats;
        stats::OsmRoadCategoryStats chosenPDLocsRoadCatStats;

        const CostCalculatorT &calculator;
        const InputConfig &inputConfig;

        // Information about best known assignment for current request
        Assignment bestAssignment;
        int bestCost;
        bool notUsingVehicleIsBest;
        int notUsingVehicleDist;
    };
}