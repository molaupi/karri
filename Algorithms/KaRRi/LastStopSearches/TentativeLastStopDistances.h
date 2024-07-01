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

#include <cassert>
#include <vector>
#include "Tools/Constants.h"
#include "DataStructures/Containers/TimestampedVector.h"

namespace karri {


// Data structure for dynamically tracking distances from last stops to PD locs.
// Allocates entries for distances from a last stop s to all PD locs when one relevant distance from s is found
// for the first time.
    template<typename LabelSetT>
    class TentativeLastStopDistances {

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

    public:

        TentativeLastStopDistances(const size_t fleetSize)
                : startIdxForVeh(fleetSize, INVALID_INDEX),
                  costs(),
                  travelTimes() {}

        void init(const int &numBatches) {
            curNumBatches = numBatches;
            startIdxForVeh.clear();
            costs.clear();
            travelTimes.clear();
        }

        void setCurBatchIdx(const int &batchIdx) {
            curBatchIdx = batchIdx;
        }

        int getCost(const int &vehId, const int &pdLocId) {
            assert(vehId < startIdxForVeh.size());
            const int startIdx = startIdxForVeh[vehId];
            if (startIdx == INVALID_INDEX)
                return INFTY;

            const int batchIdx = pdLocId / K;
            return costs[startIdx + batchIdx][pdLocId % K];
        }

        DistanceLabel getCostsForCurBatch(const int &vehId) {
            assert(vehId < startIdxForVeh.size());
            const int startIdx = startIdxForVeh[vehId];
            if (startIdx == INVALID_INDEX)
                return DistanceLabel(INFTY);
            return costs[startIdx + curBatchIdx];
        }

        int getTravelTime(const int &vehId, const int &pdLocId) {
            assert(vehId < startIdxForVeh.size());
            const int startIdx = startIdxForVeh[vehId];
            if (startIdx == INVALID_INDEX)
                return INFTY;

            const int batchIdx = pdLocId / K;
            return travelTimes[startIdx + batchIdx][pdLocId % K];
        }

        DistanceLabel getTravelTimesForCurBatch(const int &vehId) {
            assert(vehId < startIdxForVeh.size());
            const int startIdx = startIdxForVeh[vehId];
            if (startIdx == INVALID_INDEX)
                return DistanceLabel(INFTY);
            return travelTimes[startIdx + curBatchIdx];
        }

        void
        setDistancesForCurBatchIf(const int &vehId,
                                  const DistanceLabel &costBatch,
                                  const DistanceLabel &travelTimeBatch,
                                  const LabelMask &batchInsertMask) {
            if (!anySet(batchInsertMask))
                return;

            if (startIdxForVeh[vehId] == INVALID_INDEX) {
                startIdxForVeh[vehId] = costs.size();
                costs.insert(costs.end(), curNumBatches, DistanceLabel(INFTY));
                travelTimes.insert(travelTimes.end(), curNumBatches, DistanceLabel(INFTY));
            }

            costs[startIdxForVeh[vehId] + curBatchIdx].setIf(costBatch, batchInsertMask);
            travelTimes[startIdxForVeh[vehId] + curBatchIdx].setIf(travelTimeBatch, batchInsertMask);
        }


    private:

        int curNumBatches;
        TimestampedVector<int> startIdxForVeh;
        std::vector<DistanceLabel> costs; // curNumBatches DistanceLabels per vehicle
        std::vector<DistanceLabel> travelTimes;

        int curBatchIdx;

    };
}