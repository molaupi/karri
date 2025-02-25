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



    struct BatchAssignmentFinderResponse {
        std::vector<RequestState> responses;
        std::vector<stats::DispatchingPerformanceStats> stats;
    };


    template<typename AssignmentFinderFactoryT>
    class BatchAssignmentFinder {

    public:

        BatchAssignmentFinder(AssignmentFinderFactoryT& assignmentFinderFactory) : asgnFinderFactory(assignmentFinderFactory) {}

        BatchAssignmentFinderResponse findBestAssignmentsForBatchParallel(const std::vector<Request>& requestBatch) {

            BatchAssignmentFinderResponse response;
            response.responses.resize(requestBatch.size());
            response.stats.resize(requestBatch.size());

            tbb::parallel_for(0, requestBatch.size(), [&](const int i) {
                auto asgnFinder = asgnFinderFactory.getThreadLocalAssignmentFinder();
                response.responses[i] = asgnFinder.findBestAssignment(requestBatch[i], response.stats[i]);
            });

            return response;
        }

        BatchAssignmentFinderResponse findBestAssignmentsForBatchSequential(const std::vector<Request>& requestBatch) {

            BatchAssignmentFinderResponse response;
            response.responses.resize(requestBatch.size());
            response.stats.resize(requestBatch.size());

            for (int i = 0; i < requestBatch.size(); ++i) {
                auto asgnFinder = asgnFinderFactory.getThreadLocalAssignmentFinder();
                response.responses[i] = asgnFinder.findBestAssignment(requestBatch[i], response.stats[i]);
            }

            return response;
        }

        // Implements same interface as non-batch AssignmentFinder
        RequestState findBestAssignment(const Request &req, stats::DispatchingPerformanceStats& stats) {
            auto asgnFinder = asgnFinderFactory.getThreadLocalAssignmentFinder();
            return asgnFinder.findBestAssignment(req, stats);
        }


    private:

        AssignmentFinderFactoryT& asgnFinderFactory;
    };

} // karri
