/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/BaseObjects/PDLocs.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"

namespace karri {

    // Finds assignments using vehicles that are currently repositioning with the given strategy.
    template<typename StrategyT>
    class RepositioningAssignmentsFinder {
    public:
        explicit RepositioningAssignmentsFinder(StrategyT &strategy) : strategy(strategy) {}

        void init(const RequestState &requestState, const PDLocs &pdLocs,
                  stats::RepositioningAssignmentsPerformanceStats &stats) {
            strategy.init(requestState, pdLocs, stats);
        }

        template<typename PDDistancesT>
        void findAssignments(RequestState &requestState, const PDDistancesT &pdDistances, const PDLocs &pdLocs,
                             stats::RepositioningAssignmentsPerformanceStats &stats) {
            strategy.tryRepositioningAssignments(requestState, pdDistances, pdLocs, stats);
        }

    private:
        StrategyT &strategy;
    };

}
