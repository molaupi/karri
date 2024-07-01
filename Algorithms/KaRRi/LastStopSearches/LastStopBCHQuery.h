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

        using TravelTimes = StampedDistanceLabelContainer<DistanceLabel>;

        struct ScanSortedBucket {

        public:

            explicit ScanSortedBucket(LastStopBCHQuery &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &costFromV, const DistLabelContainerT & /*distLabels*/) {

                const DistanceLabel travelTimeFromV = search.travelTimes[v];

                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (allSet(search.pruner.doesNotAdmitBestAsgn(costFromV, travelTimeFromV, true)))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!LastStopBucketsEnvT::SORTED) {
                    auto bucket = search.bucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &vehId = entry.targetId;
                        if (!search.pruner.isVehicleEligible(vehId))
                            continue;

                        const DistanceLabel costViaV = DistanceLabel(entry.distToTarget) + costFromV;
                        const auto travelTimeViaV = DistanceLabel(entry.travelTimeToTarget) + travelTimeFromV;
                        tryUpdatingDistance(vehId, costViaV, travelTimeViaV);
                    }
                } else {


                    for (const auto &entry: search.bucketContainer.getBucketOf(v)) {
                        ++numEntriesScannedHere;

                        const int &vehId = entry.targetId;
                        const DistanceLabel costFromLastStopToV = entry.distToTarget;
                        const DistanceLabel costViaV = costFromLastStopToV + costFromV;
                        const DistanceLabel travelTimeViaV = DistanceLabel(entry.travelTimeToTarget) + travelTimeFromV;
                        const auto atLeastAsGoodAsCurBest =
                                ~search.pruner.doesNotAdmitBestAsgn(costViaV, travelTimeViaV, true);
                        if (!anySet(atLeastAsGoodAsCurBest))
                            break;

                        if (!search.pruner.isVehicleEligible(vehId))
                            continue;

                        tryUpdatingDistance(vehId, costViaV, travelTimeViaV);
                    }

                }

                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:

            void tryUpdatingDistance(const int vehId,
                                     const DistanceLabel &costToPDLoc, const DistanceLabel &travelTimeToPDLoc) {
                // Update tentative distances to v for any searches where costViaV admits a possible better assignment
                // than the current best and where costViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(search.tentativeDistances.getCostsForCurBatch(vehId) < costToPDLoc);
                mask &= costToPDLoc < INFTY;
                if (!anySet(mask))
                    return;

                mask &= ~search.pruner.isWorseThanBestKnownVehicleDependent(vehId, costToPDLoc, travelTimeToPDLoc);
                if (anySet(mask)) { // if any search requires updates, update the right ones according to mask
                    search.tentativeDistances.setDistancesForCurBatchIf(vehId, costToPDLoc, travelTimeToPDLoc, mask);
                    search.vehiclesSeen.insert(vehId);
                    search.pruner.updateUpperBoundCost(vehId, costToPDLoc, travelTimeToPDLoc);
                }
            }


            LastStopBCHQuery &search;
        };


        struct StopLastStopBCH {
            explicit StopLastStopBCH(LastStopBCHQuery &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &costFromV, const DistLabelContainerT & /*distLabels*/) const {
                return allSet(search.pruner.doesNotAdmitBestAsgn(costFromV, search.travelTimes[v], false));
            }

        private:
            LastStopBCHQuery &search;

        };

        struct UpdateTravelTimeCallback {

            UpdateTravelTimeCallback(const typename CH::SearchGraph &searchGraph,
                                     TravelTimes &travelTimes) : searchGraph(searchGraph), travelTimes(travelTimes) {}

            template<typename LabelMaskT, typename DistanceLabelContainerT>
            void operator()(const int v, const int w, const int e, const LabelMaskT &improved,
                            const DistanceLabelContainerT &) {
                travelTimes[w].setIf(travelTimes[v] + searchGraph.travelTime(e), improved);
            }

            const CH::SearchGraph &searchGraph;
            TravelTimes &travelTimes;
        };


    public:

        LastStopBCHQuery(
                const LastStopBucketsEnvT &lastStopBucketsEnv,
                TentativeLastStopDistances<LabelSetT> &tentativeLastStopDistances,
                const CHEnvT &chEnv,
                const RouteState &routeState,
                Subset &vehiclesSeen,
                PrunerT pruner)
                : upwardSearch(
                chEnv.template getReverseSearch<ScanSortedBucket, StopLastStopBCH, UpdateTravelTimeCallback, LabelSetT>(
                        ScanSortedBucket(*this), StopLastStopBCH(*this),
                        UpdateTravelTimeCallback(chEnv.getCH().downwardGraph(), travelTimes))),
                pruner(pruner),
                ch(chEnv.getCH()),
                bucketContainer(lastStopBucketsEnv.getBuckets()),
                routeState(routeState),
                travelTimes(ch.downwardGraph().numVertices()),
                tentativeDistances(tentativeLastStopDistances),
                vehiclesSeen(vehiclesSeen),
                numVerticesSettled(0),
                numEntriesVisited(0) {}

        void run(const std::array<int, K> &sources,
                 const std::array<int, K> costOffsets = {},
                 const std::array<int, K> travelTimeOffsets = {}) {
            numVerticesSettled = 0;
            numEntriesVisited = 0;
            std::array<int, K> sources_ranks = {};
            std::transform(sources.begin(), sources.end(), sources_ranks.begin(),
                           [&](const int v) { return ch.rank(v); });
            travelTimes.init();
            for (int i = 0; i < K; ++i)
                travelTimes[sources_ranks[i]][i] = travelTimeOffsets[i];

            upwardSearch.runWithOffset(sources_ranks, costOffsets);
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

        typename CHEnvT::template UpwardSearch<ScanSortedBucket, StopLastStopBCH, UpdateTravelTimeCallback, LabelSetT> upwardSearch;
        PrunerT pruner;

        const CH &ch;
        const typename LastStopBucketsEnvT::BucketContainer &bucketContainer;
        const RouteState &routeState;

        TravelTimes travelTimes;
        TentativeLastStopDistances<LabelSetT> &tentativeDistances;

        Subset &vehiclesSeen;
        int numVerticesSettled;
        int numEntriesVisited;
    };

}