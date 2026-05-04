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

#include "Tools/Simd/AlignedVector.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {

// Representation of direct transfer distances, i.e. distances from pickups to transfer points or from transfer points
// to dropoffs.
    template<typename LabelSetT>
    struct DirectTransferDistances {
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        static constexpr int K = LabelSetT::K;

        DirectTransferDistances(const int numVertices) : rankToDistanceIdx(numVertices) {}

        void init(const int newNumPdLocs) {
            numPdLocs = newNumPdLocs;
            std::fill(rankToDistanceIdx.begin(), rankToDistanceIdx.end(), INVALID_INDEX);
            distances.clear();
        }

        bool knowsDistancesForTransferRank(const int transferRank) const {
            return rankToDistanceIdx[transferRank] != INVALID_INDEX;
        }

        int getDistance(const unsigned int pdLocId, const int transferRank) const {
            KASSERT(pdLocId < numPdLocs);
            if (rankToDistanceIdx[transferRank] == INVALID_INDEX)
                return INFTY;
            const int res = labelFor(pdLocId, transferRank)[pdLocId % K];
            KASSERT(res < INFTY);
            return res;
        }

        void allocateEntriesFor(const int transferRank) {
            auto& idx = rankToDistanceIdx[transferRank];
            KASSERT(idx == INVALID_INDEX);
            idx = distances.size();
            const auto numBatches = numPdLocs / K + (numPdLocs % K != 0);
            distances.insert(distances.end(), numBatches, DistanceLabel(INFTY));
        }

        void updateDistanceIfSmaller(const unsigned int pdLocId, const int transferRank, const int dist) {
            const auto offsetInBatch = pdLocId % K;
            auto &curDist = labelFor(pdLocId, transferRank)[offsetInBatch];
            curDist = std::min(curDist, dist);
        }

        void updateDistanceBatchIfSmaller(const unsigned int firstPdLocId, const int transferRank,
                                          const DistanceLabel &dist) {
            auto &label = labelFor(firstPdLocId, transferRank);
            label.min(dist);
        }

    private:

        DistanceLabel &labelFor(const unsigned int pdLocId, const unsigned int transferRank) {
            KASSERT(pdLocId < numPdLocs);
            const auto idx = rankToDistanceIdx[transferRank];
            KASSERT(idx != INVALID_INDEX);
            return distances[idx + pdLocId / K];
        }

        const DistanceLabel &labelFor(const unsigned int pdLocId, const unsigned int transferRank) const {
            KASSERT(pdLocId < numPdLocs);
            const auto idx = rankToDistanceIdx[transferRank];
            KASSERT(idx != INVALID_INDEX);
            return distances[idx + pdLocId / K];
        }

        int numPdLocs;

        // Maps each queried rank in the graph to the index of the first batch for this rank in distances.
        std::vector<int> rankToDistanceIdx;

        // Distances are stored as vectors of size K (DistanceLabel).
        // ceil(numPdLocs / K) labels per queried CH rank, sequentially arranged by increasing rankToSeqIdx[location].
        AlignedVector<DistanceLabel> distances;
    };
}