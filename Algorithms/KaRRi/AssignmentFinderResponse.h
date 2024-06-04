/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RequestState/BestAsgn.h"

namespace karri {

    enum BestAsgnType {
        REGULAR,
//        DISPLACING,
        NOT_USING_VEHICLE,
        NO_ASSIGNMENT
    };

    template<typename VehInputGraphT, typename PsgInputGraphT>
    class AssignmentFinderResponse {

    public:

        AssignmentFinderResponse(const VehInputGraphT &vehInputGraph, const PsgInputGraphT &psgInputGraph,
                                 const RequestState &requestState)
                : vehInputGraph(vehInputGraph), psgInputGraph(psgInputGraph), requestState(requestState),
                  bestRegularAsgn(requestState), bestDisplacingAsgn(requestState) {}

        void initNotUsingVehicle() {
            // Compute direct walking cost:
            const int destPsgTravelTime = psgInputGraph.travelTime(
                    vehInputGraph.toPsgEdge(requestState.originalRequest.destination));
            const int cost = Calc::calcCostForNotUsingVehicle(requestState.directWalkingDist, destPsgTravelTime,
                                                              requestState);
            notUsingVehicleCost = cost;
        }

        void initRegular() {
            bestRegularAsgn.reset();
            // Direct walking cost is an upper bound for the cost of any assignment.
            bestRegularAsgn.setExternalCostUpperBound(notUsingVehicleCost);
        }

        void initDisplacing() {
            bestDisplacingAsgn.reset();
            bestDisplacingAsgn.setExternalCostUpperBound(std::min(bestRegularAsgn.cost(), notUsingVehicleCost));
        }

        BestAsgnType getBestAsgnType() const {
            const auto minCost = getBestCost();
            if (minCost == INFTY)
                return BestAsgnType::NO_ASSIGNMENT;
            if (minCost == notUsingVehicleCost)
                return BestAsgnType::NOT_USING_VEHICLE;
            return BestAsgnType::REGULAR;
        }

        int getBestCost() const {
            return std::min(bestRegularAsgn.cost(), notUsingVehicleCost);
        }

        const int &getNotUsingVehicleDist() const {
            return requestState.directWalkingDist;
        }

        BestAsgn &getBestRegularAsgn() {
            return bestRegularAsgn;
        }

        const BestAsgn &getBestRegularAsgn() const {
            return bestRegularAsgn;
        }

        BestAsgn &getBestDisplacingAsgn() {
            return bestDisplacingAsgn;
        }

        const BestAsgn &getBestDisplacingAsgn() const {
            return bestDisplacingAsgn;
        }

    private:

        const VehInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const RequestState &requestState;

        BestAsgn bestRegularAsgn;
        BestAsgn bestDisplacingAsgn;
        int notUsingVehicleCost;

    };

} // end namespace karri