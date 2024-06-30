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
                  query(chEnv.template getFullCHQuery<LabelSetT>()),
                  unpacker(chEnv.getCH()) {}


        // Computes all distances from every pickup to every dropoff and stores them in the given DirectPDDistances.
        void run() {
            Timer timer;

            // Run batched queries from all pickups to all dropoffs
            std::array<int, K> pickupHeadRanks;

            int pickupId = 0;
            while (pickupId < requestState.numPickups()) {
                pickupHeadRanks[pickupId % K] = ch.rank(inputGraph.edgeHead(requestState.pickups[pickupId].loc));
                ++pickupId;
                if (pickupId % K == 0) {
                    runWithAllDropoffs(pickupHeadRanks, pickupId - K);
                }
            }

            // Finish potential partially filled batch. Fill with copies of first element.
            if (pickupId % K != 0) {
                for (int i = pickupId % K; i < K; ++i)
                    pickupHeadRanks[i] = pickupHeadRanks[0];
                runWithAllDropoffs(pickupHeadRanks, requestState.numPickups() / K * K);
            }

            requestState.minDirectPDCost = distances.getMinCost();
            requestState.minDirectPDTravelTime = distances.getMinTravelTime();

            const int64_t pickupSearchesTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.pickupBchSearchTime += pickupSearchesTime;
        }

        void init() {
            Timer timer;
            distances.clear();
//            distances.updateDistanceIfSmaller(0, 0, requestState.directDistInFullVeh);
            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().pdDistancesStats.initializationTime += time;
        }

    private:

        void runWithAllDropoffs(const std::array<int, K>& pickupHeadRanks, const int firstPickupIdInBatch) {
            std::array<int, K> dropoffTailRank = {};

            for (const auto& d : requestState.dropoffs) {
                dropoffTailRank.fill(ch.rank(inputGraph.edgeTail(d.loc)));
                const int offset = inputGraph.travelTime(d.loc);

                query.run(pickupHeadRanks, dropoffTailRank);
                const DistanceLabel dist = query.getAllDistances() + DistanceLabel(offset);

                // Unpack path to find travel time along SP according to traversal cost
                DistanceLabel travelTime = 0;
                for (int i = 0; i < K; ++i) {
                    path.clear();
                    unpacker.unpackUpDownPath(query.getUpEdgePath(i), query.getDownEdgePath(i), path);
                    for (const auto& e : path)
                        travelTime[i] += inputGraph.travelTime(e);
                }

                distances.updateDistanceBatchIfSmaller(firstPickupIdInBatch, d.id, dist, travelTime);
            }
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        RequestState &requestState;

        PDDistances<LabelSetT> &distances;

        typename CHEnvT::template FullCHQuery<LabelSetT> query;
        CHPathUnpacker unpacker;
        std::vector<int> path;
    };

}