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
#include "Algorithms/KaRRi/BaseObjects/BestAsgnType.h"
#include "Algorithms/KaRRi/BaseObjects/PDLocs.h"

namespace karri {

    // Holds information relating to a specific request like its pickups and dropoffs and the best known assignment.
    struct RequestState {

        RequestState()
                : originalRequest(),
                    dispatchingTime(INFTY),
                  originalReqDirectDist(-1),
                  minDirectPDDist(-1),
                  odWalkingDist(INFTY),
        bestCostOverall(INFTY),
        bestAsgnOverallType(INVALID),
        bestAssignmentOneLeg(),
        bestAssignmentTwoLegs(),
        bestCostOneLeg(INFTY),
        bestCostTwoLegs(INFTY),
        bestCostObjectOneLeg(RequestCost::INFTY_COST()),
        bestCostObjectTwoLegs(RequestCost::INFTY_COST()) {}



        // Information about current request itself
        Request originalRequest;
        int dispatchingTime; // time at which request is dispatched, i.e., assignment is decided upon. Must be >= originalRequest.requestTime.
        int originalReqDirectDist; // direct distance from origin to destination
        int minDirectPDDist; // smallest distance between any pickup and any dropoff
        int odWalkingDist; // walking distance from origin to destination if rider walks the whole way

        int getPassengerArrAtPickup(const PDLoc& pickup) const {
            return dispatchingTime + pickup.walkingDist;
        }

        int getSoftConstraintOriginalReqMaxTripTime() const {
            assert(originalReqDirectDist >= 0);
            return static_cast<int>(InputConfig::getInstance().softConstraintAlpha * static_cast<double>(originalReqDirectDist)) + InputConfig::getInstance().softConstraintBeta;
        }

        int getSoftConstraintMaxDepTimeAtPickup() const {
            return originalRequest.requestTime + InputConfig::getInstance().softConstraintMaxWaitTime;
        }

        int getHardConstraintMaxTripTime(const int asgnTripTime) const {
            return static_cast<int>(InputConfig::getInstance().hardConstraintAlpha * static_cast<double>(asgnTripTime)) + InputConfig::getInstance().hardConstraintBeta;
        }

        int getHardConstraintMaxArrTimeAtDropoff(const PDLoc& dropoff, const int asgnTripTime) const {
            return originalRequest.requestTime + getHardConstraintMaxTripTime(asgnTripTime) - dropoff.walkingDist;
        }

        int getHardConstraintMaxDepTimeAtPickup(const int asgnDepTimeAtPickup) const {
            KASSERT(asgnDepTimeAtPickup >= originalRequest.requestTime);
            return asgnDepTimeAtPickup + InputConfig::getInstance().hardConstraintMaxAddedWaitTime;
        }

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const RequestCost cost) {

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

        void tryFinishedTransferAssignmentWithKnownCost(AssignmentWithTransfer &asgn, const RequestCost& cost) {
            KASSERT(asgn.isFinished());
            KASSERT(asgn.pVeh->vehicleId >= 0 && asgn.dVeh->vehicleId >= 0 && asgn.pVeh->vehicleId != asgn.dVeh->vehicleId && asgn.pickup.id != INVALID_ID && asgn.dropoff.id != INVALID_ID);
            KASSERT(asgn.distToPickup >= 0 && asgn.distFromPickup >= 0 && asgn.distToTransferPVeh >= 0 && asgn.distFromTransferPVeh >= 0 && asgn.distToTransferDVeh >= 0 && asgn.distFromTransferDVeh >= 0 && asgn.distToDropoff >= 0 && asgn.distFromDropoff >= 0);
            KASSERT(asgn.pickup.loc != asgn.transfer.loc && asgn.dropoff.loc != asgn.transfer.loc);

            // These assertions do not work due to edges with travel time 0
//            KASSERT(asgn.pickupIdx == asgn.transferIdxPVeh || asgn.distFromPickup > 0 || asgn.pickupType == AFTER_LAST_STOP);
//            KASSERT(asgn.transferIdxDVeh == asgn.dropoffIdx || asgn.distFromTransferDVeh > 0 || asgn.transferTypeDVeh == AFTER_LAST_STOP);

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

        bool validTaxiTripKnown() const {
            return bestAsgnOverallType != INVALID;
        }

        bool improvementThroughTransfer() const {
            return bestAsgnOverallType == TWO_LEGS;
        }

        const Assignment &getBestAssignmentWithoutTransfer() const {
            return bestAssignmentOneLeg;
        }

        const AssignmentWithTransfer &getBestAssignmentWithTransfer() const {
            return bestAssignmentTwoLegs;
        }

        const RequestCost &getCostObjectWithoutTransfer() const {
            return bestCostObjectOneLeg;
        }

        const RequestCost &getCostObjectWithTransfer() const {
            return bestCostObjectTwoLegs;
        }

        // const int &getBestCostWithoutUsingVehicle() const {

        //     return bestCostZeroLegs;

        // }


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

        // bool isNotUsingVehicleBest() const {

        //     return bestAsgnOverallType == NOT_USING_VEHICLE;

        // }


        // const int &getNotUsingVehicleDist() const {

        //     return notUsingVehicleDist;

        // }


        // void tryNotUsingVehicleAssignment(const int notUsingVehDist, const int travelTimeOfDestEdge) {
        //     const int cost = CostCalculator::calcCostForNotUsingVehicle(notUsingVehDist, travelTimeOfDestEdge, *this);
        //     if (cost < bestCostZeroLegs) {
        //         bestCostZeroLegs = cost;
        //         notUsingVehicleDist = notUsingVehDist;
        //         if (cost < bestCostOverall) {
        //             bestCostOverall = cost;
        //             bestAsgnOverallType = NOT_USING_VEHICLE;
        //         }
        //     }
        // }

        // void reset() {
        //
        //     originalRequest = {};
        //     originalReqDirectDist = INFTY;
        //     minDirectPDDist = INFTY;
        //
        //     bestCostOverall = INFTY;
        //     bestAsgnOverallType = INVALID;
        //
        //     bestAssignmentOneLeg = Assignment();
        //     bestAssignmentTwoLegs = AssignmentWithTransfer();
        //     // bestCostZeroLegs = INFTY;
        //     bestCostOneLeg = INFTY;
        //     bestCostTwoLegs = INFTY;
        //     // notUsingVehicleDist = INFTY;
        //
        //     bestCostObjectOneLeg = RequestCost::INFTY_COST();
        //     bestCostObjectTwoLegs = RequestCost::INFTY_COST();
        // }

    private:

        // Information about best known assignment for current request
        int bestCostOverall;
        BestAsgnType bestAsgnOverallType;

        Assignment bestAssignmentOneLeg;
        AssignmentWithTransfer bestAssignmentTwoLegs;

        // int bestCostZeroLegs;
        int bestCostOneLeg;
        int bestCostTwoLegs;

        RequestCost bestCostObjectOneLeg;
        RequestCost bestCostObjectTwoLegs;

        // int notUsingVehicleDist;
    };
}
