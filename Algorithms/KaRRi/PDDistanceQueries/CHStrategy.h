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

#include "Tools/Timer.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "PDDistances.h"

namespace karri::PDDistanceQueryStrategies {

    template<typename InputGraphT,
            typename CHEnvT,
            typename LabelSetT>
    class CHStrategy {

    public:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        CHStrategy(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                    PDDistances<LabelSetT> &distances,
                    RequestState &requestState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  requestState(requestState),
                  distances(distances),
                  queries([&]{return chEnv.template getFullCHQuery<LabelSetT>();}) {}


        // Computes all distances from every pickup to every dropoff and stores them in the given DirectPDDistances.
        void run() {
            assert(requestState.pickups[0].loc == requestState.originalRequest.origin
                   && requestState.dropoffs[0].loc == requestState.originalRequest.destination);
            Timer timer;

            if (requestState.numPickups() == 1 && requestState.numDropoffs() == 1) {
                requestState.minDirectPDDist = requestState.originalReqDirectDist;
                return;
            }

            std::vector<std::pair<int, int>> jobs;
            for (int firstPickupIdInBatch = 0; firstPickupIdInBatch < requestState.numPickups(); firstPickupIdInBatch += K) 
                for (int dropoffId = 0; dropoffId < requestState.numDropoffs(); ++dropoffId) 
                    jobs.emplace_back(firstPickupIdInBatch, dropoffId);

            tbb::parallel_for(int(0), static_cast<int>(jobs.size()), 1, [&](int jobIdx) {
                runQueryForPickupBatchAndDropoff(jobs[jobIdx].first, jobs[jobIdx].second);
            });


            requestState.minDirectPDDist = distances.getMinDirectDistance();

            const int64_t pickupSearchesTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.pickupBchSearchTime += pickupSearchesTime;
        }

        void init() {
            Timer timer;
            distances.clear();
            distances.updateDistanceIfSmaller(0, 0, requestState.originalReqDirectDist);
            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.initializationTime += time;
        }

    private:

        void runQueryForPickupBatchAndDropoff(const int firstPickupIdInBatch, const int dropoffId) {
            // Run query for batch of pickups and single
            std::array<int, K> pickupHeadRanks = {};
            for (int i = 0; i < K; ++i) {
                if (firstPickupIdInBatch + i >= requestState.numPickups()) {
                    pickupHeadRanks[i] = pickupHeadRanks[0];
                } else {
                    pickupHeadRanks[i] = ch.rank(inputGraph.edgeHead(requestState.pickups[firstPickupIdInBatch + i].loc));
                }
            }

            const auto& d = requestState.dropoffs[dropoffId];
            std::array<int, K> dropoffTailRank = {};
            dropoffTailRank.fill(ch.rank(inputGraph.edgeTail(d.loc)));
            const int offset = inputGraph.travelTime(d.loc);

            auto& query = queries.local();
            query.run(pickupHeadRanks, dropoffTailRank);
            const DistanceLabel dist = query.getAllDistances() + DistanceLabel(offset);
            distances.updateDistanceBatchIfSmaller(firstPickupIdInBatch, d.id, dist);
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        RequestState &requestState;

        PDDistances<LabelSetT> &distances;

        tbb::enumerable_thread_specific<typename CHEnvT::template FullCHQuery<LabelSetT>> queries;
    };

}