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
#include "DataStructures/Containers/Parallel/ThreadSafeSubset.h"
#include "Algorithms/CH/CH.h"
#include "Tools/Constants.h"
#include "TentativeLastStopDistances.h"

#include <tbb/enumerable_thread_specific.h>

namespace karri {
    using namespace tbb;

    template<typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename PrunerT,
            typename LabelSetT = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>>
    class LastStopBCHQuery {

    private:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;
        using ThreadLocalTentativeDistances = typename TentativeLastStopDistances<LabelSetT>::ThreadLocalTentativeLastStopDistances;

        struct UpdateDistancesToPDLocs {

            UpdateDistancesToPDLocs() : curTentative(nullptr) {}

            void operator()(const int vehId, const DistanceLabel &distToPDLoc, const LabelMask &mask) {

                assert(curTentative);
                return curTentative->setDistancesForCurBatchIf(vehId, distToPDLoc, mask);
            }

            DistanceLabel getDistances(const int &vehId) {
                return curTentative->getDistancesForCurBatch(vehId);
            }

            void setCurLocalTentative(ThreadLocalTentativeDistances *const newCurTentative) {
                curTentative = newCurTentative;
            }

        private:
            ThreadLocalTentativeDistances *curTentative;
        };


        struct ScanSortedBucket {

        public:

            explicit ScanSortedBucket(LastStopBCHQuery &search,
                                      PrunerT &pruner,
                                      UpdateDistancesToPDLocs &updateDistances,
                                      int& numVerticesSettled,
                                      int& numEntriesVisited)
                    : search(search),
                      pruner(pruner),
                      updateDistances(updateDistances),
                      numVerticesSettled(numVerticesSettled),
                      numEntriesVisited(numEntriesVisited) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContainerT & /*distLabels*/) {

                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (allSet(pruner.doesDistanceNotAdmitBestAsgn(distFromV, true)))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!LastStopBucketsEnvT::SORTED) {
                    auto bucket = search.bucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &vehId = entry.targetId;
                        if (!pruner.isVehicleEligible(vehId))
                            continue;

                        const DistanceLabel distViaV = distFromV + DistanceLabel(entry.distToTarget);
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
                            const auto atLeastAsGoodAsCurBest = ~pruner.doesDistanceNotAdmitBestAsgn(distViaV, true);
                            if (!anySet(atLeastAsGoodAsCurBest))
                                break;

                            if (!pruner.isVehicleEligible(vehId))
                                continue;

                            tryUpdatingDistance(vehId, distViaV);
                        }
                    }

                    auto nonIdleBucket = search.bucketContainer.getNonIdleBucketOf(v);
                    for (const auto &entry: nonIdleBucket) {
                        ++numEntriesScannedHere;
                        const int &vehId = entry.targetId;
                        assert(search.routeState.numStopsOf(vehId) > 1);
                        const DistanceLabel arrTimeAtV = entry.distToTarget;
                        const DistanceLabel arrTimeAtPDLoc = arrTimeAtV + distFromV;
                        const auto atLeastAsGoodAsCurBest = ~pruner.doesArrTimeNotAdmitBestAsgn(arrTimeAtPDLoc,
                                                                                                distFromV);
                        if (!anySet(atLeastAsGoodAsCurBest))
                            break;

                        if (!pruner.isVehicleEligible(vehId))
                            continue;


                        const auto depTimeAtLastStop = search.routeState.schedDepTimesFor(vehId)[
                                search.routeState.numStopsOf(vehId) - 1];
                        const auto distViaV = arrTimeAtPDLoc - depTimeAtLastStop;
                        tryUpdatingDistance(vehId, distViaV);
                    }
                }

                numEntriesVisited += numEntriesScannedHere;
                ++numVerticesSettled;

                return false;
            }

        private:

            void tryUpdatingDistance(const int vehId, const DistanceLabel &distToPDLoc) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(updateDistances.getDistances(vehId) < distToPDLoc);
                mask &= distToPDLoc < INFTY;
                if (!anySet(mask))
                    return;

                mask &= ~pruner.isWorseThanBestKnownVehicleDependent(vehId, distToPDLoc);
                if (anySet(mask)) { // if any search requires updates, update the right ones according to mask
                    updateDistances(vehId, distToPDLoc, mask);
                    pruner.updateUpperBoundCost(vehId, distToPDLoc);
                }
            }


            LastStopBCHQuery &search;
            PrunerT &pruner;
            UpdateDistancesToPDLocs &updateDistances;
            int& numVerticesSettled;
            int& numEntriesVisited;
        };


        struct StopLastStopBCH {
            explicit StopLastStopBCH(const PrunerT &pruner) : pruner(pruner) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) const {
                return allSet(pruner.doesDistanceNotAdmitBestAsgn(distToV, false));
            }

        private:
            const PrunerT& pruner;

        };

        typedef typename CHEnvT::template UpwardSearch<ScanSortedBucket, StopLastStopBCH, LabelSetT> UpwardSearchType;

    public:

        LastStopBCHQuery(
                const LastStopBucketsEnvT &lastStopBucketsEnv,
                TentativeLastStopDistances<LabelSetT> &tentativeLastStopDistances,
                const CHEnvT &chEnv,
                const RouteState &routeState,
                tbb::enumerable_thread_specific<PrunerT> &pPruners)
                : ch(chEnv.getCH()),
                  bucketContainer(lastStopBucketsEnv.getBuckets()),
                  routeState(routeState),
                  tentativeDistances(tentativeLastStopDistances),
                  pruners(pPruners),
                  updateDistancesToPdLocs(),
                  upwardSearch([&]() {
                      return chEnv.template getReverseSearch<ScanSortedBucket, StopLastStopBCH, LabelSetT>(
                              ScanSortedBucket(*this, pruners.local(), updateDistancesToPdLocs.local(), numVerticesSettled.local(), numEntriesVisited.local()),
                              StopLastStopBCH(pruners.local()));
                  }),
                  numVerticesSettled(0),
                  numEntriesVisited(0) {}

        void run(const std::array<int, K> &sources,
                 const std::array<int, K> offsets = {}) {

            numVerticesSettled.local() = 0;
            numEntriesVisited.local() = 0;

            // Get reference to thread local result structure once and have search work on it.
            auto localTentativeDistances = tentativeDistances.getThreadLocalTentativeDistances();
            localTentativeDistances.initForSearch();
            auto &localUpdateDistances = updateDistancesToPdLocs.local();
            localUpdateDistances.setCurLocalTentative(&localTentativeDistances);

            std::array<int, K> sources_ranks = {};
            std::transform(sources.begin(), sources.end(), sources_ranks.begin(),
                           [&](const int v) { return ch.rank(v); });

            UpwardSearchType &localUpwardSearch = upwardSearch.local();
            localUpwardSearch.runWithOffset(sources_ranks, offsets);
        }

        int getLocalNumEdgeRelaxations() {
            return upwardSearch.local().getNumEdgeRelaxations();
        }

        int getLocalNumVerticesSettled() {
            return numVerticesSettled.local();
        }

        int getLocalNumEntriesScanned() {
            return numEntriesVisited.local();
        }

    private:

        const CH &ch;
        const typename LastStopBucketsEnvT::BucketContainer &bucketContainer;
        const RouteState &routeState;

        TentativeLastStopDistances<LabelSetT> &tentativeDistances;
        enumerable_thread_specific<PrunerT> &pruners;


        enumerable_thread_specific<UpdateDistancesToPDLocs> updateDistancesToPdLocs;
        enumerable_thread_specific<UpwardSearchType> upwardSearch;

        enumerable_thread_specific<int> numVerticesSettled;
        enumerable_thread_specific<int> numEntriesVisited;
    };

}