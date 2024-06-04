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

#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/CostCalculator.h"

namespace karri {

    // Information about best known assignment for current request
    class BestAsgn {

    public:
        BestAsgn(const RequestState& requestState)
                : bestAssignment(),
                  bestCost(-1),
                  requestState(requestState) {}

        ~BestAsgn() {}

        void reset() {
            bestAssignment = Assignment();
            bestCost = INFTY;
        }

        const Assignment &asgn() const {
            return bestAssignment;
        }

        const int &cost() const {
            return bestCost;
        }

        bool tryAssignment(const Assignment &asgn, const RouteStateData& routeState) {
            const auto cost = Calc::calc(asgn, requestState, routeState);
            return tryAssignmentWithKnownCost(asgn, cost, routeState);
        }

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const int cost, const RouteStateData& routeState) {
            KASSERT(Calc::calc(asgn, requestState, routeState) == cost);

            if (cost < INFTY && (cost < bestCost || (cost == bestCost &&
                                                     breakCostTie(asgn, bestAssignment)))) {

                bestAssignment = asgn;
                bestCost = cost;
                return true;
            }
            return false;
        }

        void setExternalCostUpperBound(const int cost) {
            if (cost < bestCost) {
                bestCost = cost;
                bestAssignment = Assignment();
            }
        }

    private:

        Assignment bestAssignment;
        int bestCost;
        const RequestState& requestState;

    };

} // end namespace karri
