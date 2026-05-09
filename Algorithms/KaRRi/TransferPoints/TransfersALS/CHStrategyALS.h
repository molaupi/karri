/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
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

#include "FlatRegular2DDistanceArray.h"

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT, typename LabelSetT>
    class CHStrategyALS {

        static constexpr int K = LabelSetT::K;

    public:

        CHStrategyALS(
                const RouteState &routeState,
                const Fleet &fleet,
                const InputGraphT &inputGraph,
                const VehCHEnvT &vehChEnv
        ) : routeState(routeState),
            fleet(fleet),
            inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            vehChQuery(vehChEnv.template getFullCHQuery<LabelSetT>()) {}

        template<bool = true>
        void init(const std::vector<int> &) {
            // No selection phase in CHStrategyALS, so we do nothing here.
        }

        const FlatRegular2DDistanceArray &
        calculateDistancesFromLastStopToAllTransfers(const std::vector<int> &lastStopLocs,
                                                     const std::vector<int> &transferPoints) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            lastStopsToTransfersDistances.init(lastStopLocs.size(), numTransferPoints);

            if (lastStopLocs.empty() || transferPoints.empty())
                return lastStopsToTransfersDistances;

            for (int i = 0; i < lastStopLocs.size(); ++i) {
                const int idxOffset = i * numTransferPoints;
                auto distancesForRow = lastStopsToTransfersDistances.distances.begin() + idxOffset;
                runOneToMany(lastStopLocs[i], transferPoints, distancesForRow);
                KASSERT(numTransferPoints != 0);
                lastStopsToTransfersDistances.minDistancePerRow[i] = *std::min_element(distancesForRow,
                                                                                       distancesForRow +
                                                                                       numTransferPoints);
            }

            return lastStopsToTransfersDistances;
        }

        const FlatRegular2DDistanceArray &
        calculateDistancesFromPickupsToAllTransfers(const std::vector<int> &pickupLocs,
                                                    const std::vector<int> &transferPoints) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            pickupsToTransfersDistances.init(pickupLocs.size(), numTransferPoints);

            if (pickupLocs.empty() || transferPoints.empty())
                return pickupsToTransfersDistances;

            for (int i = 0; i < pickupLocs.size(); ++i) {
                const int idxOffset = i * numTransferPoints;
                auto distancesForRow = pickupsToTransfersDistances.distances.begin() + idxOffset;
                runOneToMany(pickupLocs[i], transferPoints, distancesForRow);
                KASSERT(numTransferPoints != 0);
                pickupsToTransfersDistances.minDistancePerRow[i] = *std::min_element(distancesForRow, distancesForRow + numTransferPoints);
            }

            return pickupsToTransfersDistances;

        }

        const FlatRegular2DDistanceArray &
        calculateDistancesFromAllTransfersToDropoffs(const std::vector<int> &transferPoints,
                                                     const std::vector<int> &dropoffLocs) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            transfersToDropoffsDistances.init(dropoffLocs.size(), numTransferPoints);

            if (dropoffLocs.empty() || transferPoints.empty())
                return transfersToDropoffsDistances;

            for (int i = 0; i < dropoffLocs.size(); ++i) {
                const int idxOffset = i * numTransferPoints;
                auto distancesForRow = transfersToDropoffsDistances.distances.begin() + idxOffset;
                runManyToOne(transferPoints, dropoffLocs[i], distancesForRow);
                KASSERT(numTransferPoints != 0);
                transfersToDropoffsDistances.minDistancePerRow[i] = *std::min_element(distancesForRow, distancesForRow + numTransferPoints);
            }

            return transfersToDropoffsDistances;
        }


    private:
        template<typename DistanceStoreIt>
        void runOneToMany(const int sourceLoc, const std::vector<int> &targetLocs, DistanceStoreIt distances) {

            const auto sourceRank = vehCh.rank(inputGraph.edgeHead(sourceLoc));
            std::array<int, K> sourceAsArr;
            sourceAsArr.fill(sourceRank);

            std::array<int, K> targets{};
            std::array<int, K> offsets{};
            const int numBatches = targetLocs.size() / K + (targetLocs.size() % K != 0);
            for (int batchIdx = 0; batchIdx < numBatches; ++batchIdx) {
                const int batchStart = batchIdx * K;
                const int batchEnd = std::min(batchStart + K, static_cast<int>(targetLocs.size()));
                for (int i = batchStart; i < batchEnd; ++i) {
                    const int loc = targetLocs[i];
                    targets[i - batchStart] = vehCh.rank(inputGraph.edgeTail(loc));
                    offsets[i - batchStart] = inputGraph.travelTime(loc);
                }
                for (int i = batchEnd; i < batchStart + K; ++i) {
                    targets[i - batchStart] = targets[0]; // copy of first target to fill partial batch
                    offsets[i - batchStart] = offsets[0]; // copy of first offset to fill partial batch
                }

                vehChQuery.run(sourceAsArr, targets);

                for (int i = batchStart; i < batchEnd; ++i) {
                    distances[i] = vehChQuery.getDistance(i - batchStart) + offsets[i - batchStart];
                }
            }
        }

        template<typename DistanceStoreIt>
        void runManyToOne(const std::vector<int> &sourceLocs, const int targetLoc, DistanceStoreIt distances) {

            const auto targetRank = vehCh.rank(inputGraph.edgeTail(targetLoc));
            std::array<int, K> targetAsArr;
            targetAsArr.fill(targetRank);
            const int offset = inputGraph.travelTime(targetLoc);

            std::array<int, K> sources{};
            const int numBatches = sourceLocs.size() / K + (sourceLocs.size() % K != 0);
            for (int batchIdx = 0; batchIdx < numBatches; ++batchIdx) {
                const int batchStart = batchIdx * K;
                const int batchEnd = std::min(batchStart + K, static_cast<int>(sourceLocs.size()));
                for (int i = batchStart; i < batchEnd; ++i) {
                    const int loc = sourceLocs[i];
                    sources[i - batchStart] = vehCh.rank(inputGraph.edgeHead(loc));
                }
                for (int i = batchEnd; i < batchStart + K; ++i) {
                    sources[i - batchStart] = sources[0]; // copy of first sources to fill partial batch
                }

                vehChQuery.run(sources, targetAsArr);

                for (int i = batchStart; i < batchEnd; ++i) {
                    distances[i] = vehChQuery.getDistance(i - batchStart) + offset;
                }
            }
        }

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<LabelSetT>;

        const RouteState &routeState;
        const Fleet &fleet;
        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

        // Results of queries. Stored here to avoid reallocation in each query.
        FlatRegular2DDistanceArray lastStopsToTransfersDistances;
        FlatRegular2DDistanceArray pickupsToTransfersDistances;
        FlatRegular2DDistanceArray transfersToDropoffsDistances;

    };

}