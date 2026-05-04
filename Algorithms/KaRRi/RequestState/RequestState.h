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
    struct RequestState {

        enum BestAsgnType {
            NOT_USING_VEHICLE,
            ONE_LEG,
            TWO_LEGS,
            INVALID
        };

        RequestState(CostCalculator &calculator)
                : originalRequest(),
                  originalReqDirectDist(-1),
                  minDirectPDDist(-1),
                  pickups(),
                  dropoffs(),
                  calculator(calculator) {}


        ~RequestState() {
            auto &roadCatLogger = LogManager<std::ofstream>::getLogger(karri::stats::OsmRoadCategoryStats::LOGGER_NAME,
                                                                       "type," +
                                                                       karri::stats::OsmRoadCategoryStats::getLoggerCols());
            roadCatLogger << "all_pd_locs, " << allPDLocsRoadCatStats.getLoggerRow() << "\n";
            roadCatLogger << "chosen_pd_locs, " << chosenPDLocsRoadCatStats.getLoggerRow() << "\n";
        }


        // Information about current request itself
        Request originalRequest;
        int originalReqDirectDist;
        int minDirectPDDist;

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
            KASSERT(originalReqDirectDist >= 0);
            return static_cast<int>(InputConfig::getInstance().alpha * static_cast<double>(originalReqDirectDist)) + InputConfig::getInstance().beta;
        }

        int getPassengerArrAtPickup(const int pickupId) const {
            KASSERT(pickupId < numPickups());
            return originalRequest.requestTime + pickups[pickupId].walkingDist;
        }

        int getMaxPDTripTime(const int pickupId, const int dropoffId) const {
            KASSERT(pickupId < numPickups() && dropoffId < numDropoffs());
            KASSERT(originalReqDirectDist >= 0);
            return getOriginalReqMaxTripTime() - (pickups[pickupId].walkingDist + dropoffs[dropoffId].walkingDist);
        }

        int getMaxArrTimeAtDropoff(const int pickupId, const int dropoffId) const {
            KASSERT(pickupId < numPickups() && dropoffId < numDropoffs());
            return getPassengerArrAtPickup(pickupId) + getMaxPDTripTime(pickupId, dropoffId);
        }

        int getMaxDepTimeAtPickup() const {
            return originalRequest.requestTime + InputConfig::getInstance().maxWaitTime;
        }

        const Assignment &getBestAssignmentWithoutTransfer() const {
            return bestAssignmentOneLeg;
        }

        const AssignmentWithTransfer &getBestAssignmentWithTransfer() const {
            return bestAssignmentTwoLegs;
        }

        void tryFinishedTransferAssignmentWithKnownCost(AssignmentWithTransfer &asgn, const RequestCost& cost) {
            KASSERT(asgn.isFinished());
            KASSERT(asgn.pVeh->vehicleId >= 0 && asgn.dVeh->vehicleId >= 0 && asgn.pVeh->vehicleId != asgn.dVeh->vehicleId && asgn.pickup && asgn.dropoff);
            KASSERT(asgn.distToPickup >= 0 && asgn.distFromPickup >= 0 && asgn.distToTransferPVeh >= 0 && asgn.distFromTransferPVeh >= 0 && asgn.distToTransferDVeh >= 0 && asgn.distFromTransferDVeh >= 0 && asgn.distToDropoff >= 0 && asgn.distFromDropoff >= 0);
            KASSERT(asgn.pickup->loc != asgn.transfer.loc && asgn.dropoff->loc != asgn.transfer.loc);

            // These assertions do not work due to edges with travel time 0
//            KASSERT(asgn.pickupIdx == asgn.transferIdxPVeh || asgn.distFromPickup > 0 || asgn.pickupType == AFTER_LAST_STOP);
//            KASSERT(asgn.transferIdxDVeh == asgn.dropoffIdx || asgn.distFromTransferDVeh > 0 || asgn.transferTypeDVeh == AFTER_LAST_STOP);

            KASSERT(cost == calculator.calc(asgn, *this));

            if (cost.total < bestCostTwoLegs) {
                bestAssignmentTwoLegs = AssignmentWithTransfer(asgn);
                bestCostTwoLegs = cost.total;
                bestCostObjectTwoLegs = cost;

                if (cost.total < bestCostOverall) {
                    bestCostOverall = cost.total;
                    bestAsgnOverallType = TWO_LEGS;
                }
            }
        }
        
        bool improvementThroughTransfer() const {
            return bestAsgnOverallType == TWO_LEGS;
        }

        const RequestCost &getCostObjectWithoutTransfer() const {
            return bestCostObjectOneLeg;
        }

        const RequestCost &getCostObjectWithTransfer() const {
            return bestCostObjectTwoLegs;
        }

        const int &getBestCostWithoutUsingVehicle() const {
            return bestCostZeroLegs;
        }

        const int &getBestCostWithoutTransfer() const {
            return bestCostOneLeg;
        }

        const int &getBestCostWithTransfer() const {
            return bestCostTwoLegs;
        }

        int getBestCost() const {
            return bestCostOverall;
        }

        BestAsgnType getBestAsgnType() const {
            return bestAsgnOverallType;
        }

        bool isNotUsingVehicleBest() const {
            return bestAsgnOverallType == NOT_USING_VEHICLE;
        }

        const int &getNotUsingVehicleDist() const {
            return notUsingVehicleDist;
        }

        bool tryAssignment(const Assignment &asgn) {
            const auto cost = calculator.calc(asgn, *this);
            return tryAssignmentWithKnownCost(asgn, cost);
        }

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const RequestCost cost) {
            KASSERT(calculator.calc(asgn, *this).total == cost.total);

            if (cost.total < INFTY && (cost.total < bestCostOneLeg || (cost.total == bestCostOneLeg &&
                                    breakCostTie(asgn, bestAssignmentOneLeg)))) {

                bestAssignmentOneLeg = asgn;
                bestCostOneLeg = cost.total;
                bestCostObjectOneLeg = cost;

                if (cost.total < bestCostOverall) {
                    bestCostOverall = cost.total;
                    bestAsgnOverallType = ONE_LEG;
                }

                return true;
            }
            
            return false;
        }

        void tryNotUsingVehicleAssignment(const int notUsingVehDist, const int travelTimeOfDestEdge) {
            const int cost = CostCalculator::calcCostForNotUsingVehicle(notUsingVehDist, travelTimeOfDestEdge, *this);
            if (cost < bestCostZeroLegs) {
                bestCostZeroLegs = cost;
                notUsingVehicleDist = notUsingVehDist;
                if (cost < bestCostOverall) {
                    bestCostOverall = cost;
                    bestAsgnOverallType = NOT_USING_VEHICLE;
                }
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

            bestCostOverall = INFTY;
            bestAsgnOverallType = INVALID;

            bestAssignmentOneLeg = Assignment();
            bestAssignmentTwoLegs = AssignmentWithTransfer();
            bestCostZeroLegs = INFTY;
            bestCostOneLeg = INFTY;
            bestCostTwoLegs = INFTY;
            notUsingVehicleDist = INFTY;

            bestCostObjectOneLeg = RequestCost::INFTY_COST();
            bestCostObjectTwoLegs = RequestCost::INFTY_COST();
        }

    private:

        stats::DispatchingPerformanceStats perfStats;
        stats::OsmRoadCategoryStats allPDLocsRoadCatStats;
        stats::OsmRoadCategoryStats chosenPDLocsRoadCatStats;

        CostCalculator &calculator;

        // Information about best known assignment for current request
        int bestCostOverall;
        BestAsgnType bestAsgnOverallType;

        Assignment bestAssignmentOneLeg;
        AssignmentWithTransfer bestAssignmentTwoLegs;

        int bestCostZeroLegs;
        int bestCostOneLeg;
        int bestCostTwoLegs;

        RequestCost bestCostObjectOneLeg;
        RequestCost bestCostObjectTwoLegs;

        int notUsingVehicleDist;
    };
}