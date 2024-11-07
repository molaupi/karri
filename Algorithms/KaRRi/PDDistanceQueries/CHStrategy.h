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
        using PDDistancesT = PDDistances<LabelSetT>;

        CHStrategy(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                    RequestState &requestState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  requestState(requestState),
                  query(chEnv.template getFullCHQuery<LabelSetT>()) {}


        // Computes all distances from every pickup to every dropoff and stores them in the given DirectPDDistances.
        void run(PDDistancesT& pdDistances) {
            assert(requestState.pickups[0].loc == requestState.originalRequest.origin
                   && requestState.dropoffs[0].loc == requestState.originalRequest.destination);
            Timer timer;

            if (requestState.numPickups() == 1 && requestState.numDropoffs() == 1) {
                requestState.minDirectPDDist = requestState.originalReqDirectDist;
                return;
            }

            // Run batched queries from all pickups to all dropoffs
            std::array<int, K> pickupHeadRanks;

            int pickupId = 0;
            while (pickupId < requestState.numPickups()) {
                pickupHeadRanks[pickupId % K] = ch.rank(inputGraph.edgeHead(requestState.pickups[pickupId].loc));
                ++pickupId;
                if (pickupId % K == 0) {
                    runWithAllDropoffs(pickupHeadRanks, pickupId - K, pdDistances);
                }
            }

            // Finish potential partially filled batch. Fill with copies of first element.
            if (pickupId % K != 0) {
                for (int i = pickupId % K; i < K; ++i)
                    pickupHeadRanks[i] = pickupHeadRanks[0];
                runWithAllDropoffs(pickupHeadRanks, requestState.numPickups() / K * K, pdDistances);
            }


            requestState.minDirectPDDist = pdDistances.getMinDirectDistance();

            const int64_t pickupSearchesTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.pickupBchSearchTime += pickupSearchesTime;
        }

        void init() {
            // no op
        }

    private:

        void runWithAllDropoffs(const std::array<int, K>& pickupHeadRanks, const int firstPickupIdInBatch, PDDistancesT & pdDistances) {
            std::array<int, K> dropoffTailRank = {};

            for (const auto& d : requestState.dropoffs) {
                dropoffTailRank.fill(ch.rank(inputGraph.edgeTail(d.loc)));
                const int offset = inputGraph.travelTime(d.loc);

                query.run(pickupHeadRanks, dropoffTailRank);
                const DistanceLabel dist = query.getAllDistances() + DistanceLabel(offset);
                pdDistances.updateDistanceBatchIfSmaller(firstPickupIdInBatch, d.id, dist);
            }
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        RequestState &requestState;

        typename CHEnvT::template FullCHQuery<LabelSetT> query;
    };

}