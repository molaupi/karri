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
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

namespace karri {




    template<typename AssignmentFinderFactoryT>
    class BatchAssignmentFinder {

    public:

        BatchAssignmentFinder(AssignmentFinderFactoryT& assignmentFinderFactory) : asgnFinderFactory(assignmentFinderFactory) {}

        std::vector<RequestState> findBestAssignmentsForBatchParallel(const std::vector<Request>& requestBatch, const int now, std::vector<stats::DispatchingPerformanceStats>& stats) {

            std::vector<RequestState> responses(requestBatch.size());

            tbb::parallel_for(0, static_cast<int>(requestBatch.size()), [&](const int i) {
                auto asgnFinder = asgnFinderFactory.getThreadLocalAssignmentFinder();
                responses[i] = asgnFinder.findBestAssignment(requestBatch[i], now, stats[i]);
            });

            return responses;
        }

        std::vector<RequestState> findBestAssignmentsForBatchSequential(const std::vector<Request>& requestBatch, const int now, std::vector<stats::DispatchingPerformanceStats>& stats) {

            std::vector<RequestState> responses(requestBatch.size());

            for (int i = 0; i < requestBatch.size(); ++i) {
                auto asgnFinder = asgnFinderFactory.getThreadLocalAssignmentFinder();
                responses[i] = asgnFinder.findBestAssignment(requestBatch[i], now, stats[i]);
            }

            return responses;
        }

        // Implements same interface as non-batch AssignmentFinder
        RequestState findBestAssignment(const Request &req, const int now, stats::DispatchingPerformanceStats& stats) {
            auto asgnFinder = asgnFinderFactory.getThreadLocalAssignmentFinder();
            return asgnFinder.findBestAssignment(req, now, stats);
        }


    private:

        AssignmentFinderFactoryT& asgnFinderFactory;
    };

} // karri
