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

#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Containers/Subset.h"
#include "Algorithms/CH/CH.h"
#include "Tools/Constants.h"
#include "TentativeLastStopDistances.h"

namespace karri {


    template<typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename PrunerT,
            typename LabelSetT = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>>
    class LastStopBCHQuery {

    private:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        struct ScanSortedBucket {

        public:

            explicit ScanSortedBucket(LastStopBCHQuery &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContainerT & /*distLabels*/) {

                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (allSet(search.pruner.doesDistanceNotAdmitBestAsgn(distFromV, true)))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (LastStopBucketsEnvT::SORTING == UNSORTED) {
                    auto bucket = search.bucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &vehId = entry.targetId;
                        if (!search.pruner.isVehicleEligible(vehId))
                            continue;

                        const DistanceLabel distViaV = distFromV + DistanceLabel(entry.distToTarget);
                        tryUpdatingDistance(vehId, distViaV);
                    }
                } else if constexpr (LastStopBucketsEnvT::SORTING == ONLY_DIST) {
                    auto bucket = search.bucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &vehId = entry.targetId;
                        const DistanceLabel distFromLastStopToV = entry.distToTarget;
                        const DistanceLabel distViaV = distFromLastStopToV + distFromV;
                        const auto atLeastAsGoodAsCurBest = ~search.pruner.doesDistanceNotAdmitBestAsgn(distViaV, true);
                        if (!anySet(atLeastAsGoodAsCurBest))
                            break;

                        if (!search.pruner.isVehicleEligible(vehId))
                            continue;

                        tryUpdatingDistance(vehId, distViaV);
                    }
                } else {

                    if constexpr (PrunerT::INCLUDE_IDLE_VEHICLES) {
                        auto idleBucket = search.bucketContainer.getIdleBucketOf(v);

                        for (const auto &entry: idleBucket) {
                            ++numEntriesScannedHere;

                            const int &vehId = entry.targetId;
                            assert(search.routeState.numStopsOf(vehId) == 1);
                            const DistanceLabel distFromLastStopToV = entry.distToTarget;
                            const DistanceLabel distViaV = distFromLastStopToV + distFromV;
                            const auto atLeastAsGoodAsCurBest = ~search.pruner.doesDistanceNotAdmitBestAsgn(distViaV, true);
                            if (!anySet(atLeastAsGoodAsCurBest))
                                break;

                            if (!search.pruner.isVehicleEligible(vehId))
                                continue;

                            tryUpdatingDistance(vehId, distViaV);
                        }
                    }

                    auto nonIdleBucket = search.bucketContainer.getNonIdleBucketOf(v);
                    for (const auto& entry : nonIdleBucket) {
                        ++numEntriesScannedHere;
                        const int& vehId = entry.targetId;
                        assert(search.routeState.numStopsOf(vehId) > 1);
                        const DistanceLabel arrTimeAtV = entry.distToTarget;
                        const DistanceLabel arrTimeAtPDLoc = arrTimeAtV + distFromV;
                        const auto atLeastAsGoodAsCurBest = ~search.pruner.doesArrTimeNotAdmitBestAsgn(arrTimeAtPDLoc,distFromV);
                        if (!anySet(atLeastAsGoodAsCurBest))
                            break;

                        if (!search.pruner.isVehicleEligible(vehId))
                            continue;


                        const auto depTimeAtLastStop = search.routeState.schedDepTimesFor(vehId)[search.routeState.numStopsOf(vehId) - 1];
                        const auto distViaV = arrTimeAtPDLoc - depTimeAtLastStop;
                        tryUpdatingDistance(vehId, distViaV);
                    }
                }

                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:

            void tryUpdatingDistance(const int vehId, const DistanceLabel& distToPDLoc) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(search.tentativeDistances.getDistancesForCurBatch(vehId) < distToPDLoc);
                mask &= distToPDLoc < INFTY;
                if (!anySet(mask))
                    return;

                mask &= ~search.pruner.isWorseThanBestKnownVehicleDependent(vehId, distToPDLoc);
                if (anySet(mask)) { // if any search requires updates, update the right ones according to mask
                    search.tentativeDistances.setDistancesForCurBatchIf(vehId, distToPDLoc, mask);
                    search.vehiclesSeen.insert(vehId);
                    search.pruner.updateUpperBoundCost(vehId, distToPDLoc);
                }
            }


            LastStopBCHQuery &search;
        };


        struct StopLastStopBCH {
            explicit StopLastStopBCH(const LastStopBCHQuery &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) const {
                return allSet(search.pruner.doesDistanceNotAdmitBestAsgn(distToV, false));
            }

        private:
            const LastStopBCHQuery &search;

        };


    public:

        LastStopBCHQuery(
                const LastStopBucketsEnvT &lastStopBucketsEnv,
                TentativeLastStopDistances <LabelSetT> &tentativeLastStopDistances,
                const CHEnvT &chEnv,
                const RouteState& routeState,
                Subset &vehiclesSeen,
                PrunerT pruner)
                : upwardSearch(chEnv.template getReverseSearch<ScanSortedBucket, StopLastStopBCH, LabelSetT>(
                ScanSortedBucket(*this), StopLastStopBCH(*this))),
                  pruner(pruner),
                  ch(chEnv.getCH()),
                  bucketContainer(lastStopBucketsEnv.getBuckets()),
                  routeState(routeState),
                  tentativeDistances(tentativeLastStopDistances),
                  vehiclesSeen(vehiclesSeen),
                  numVerticesSettled(0),
                  numEntriesVisited(0) {}

        void run(const std::array<int, K> &sources,
                 const std::array<int, K> offsets = {}) {
            numVerticesSettled = 0;
            numEntriesVisited = 0;
            std::array<int, K> sources_ranks = {};
            std::transform(sources.begin(), sources.end(), sources_ranks.begin(),
                           [&](const int v) { return ch.rank(v); });
            upwardSearch.runWithOffset(sources_ranks, offsets);
        }

        int getNumEdgeRelaxations() const {
            return upwardSearch.getNumEdgeRelaxations();
        }

        int getNumVerticesSettled() const {
            return numVerticesSettled;
        }

        int getNumEntriesScanned() const {
            return numEntriesVisited;
        }

    private:

        typename CHEnvT::template UpwardSearch<ScanSortedBucket, StopLastStopBCH, LabelSetT> upwardSearch;
        PrunerT pruner;

        const CH &ch;
        const typename LastStopBucketsEnvT::BucketContainer &bucketContainer;
        const RouteState& routeState;

        TentativeLastStopDistances <LabelSetT> &tentativeDistances;

        Subset &vehiclesSeen;
        int numVerticesSettled;
        int numEntriesVisited;
    };

}