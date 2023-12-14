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
            typename LastStopBucketsUpdaterT,
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
                if (allSet(search.pruner.isWorseThanUpperBoundCost(distFromV, true)))
                    return true;

                int numEntriesScannedHere = 0;
                assert(search.bucketContainer != nullptr);
                auto bucket = search.bucketContainer->getBucketOf(v);
                for (const auto &entry: bucket) {

                    const int &vehId = entry.targetId;


                    ++numEntriesScannedHere;

                    const DistanceLabel distViaV = distFromV + DistanceLabel(entry.distToTarget);
                    LabelMask atLeastAsGoodAsCurBest = ~search.pruner.isWorseThanUpperBoundCost(distViaV, true);

                    if constexpr (LastStopBucketsUpdaterT::SORTED_BY_DIST) {
                        // Entries are ordered according to entry.distToTarget, i.e. the distance from the last
                        // stop of the vehicle with id entry.targetId to vertex v. For a cost calculation that only depends on
                        // this distance (and other factors that can be considered constant during the bucket scan), the cost
                        // grows monotonously with the bucket entries. Therefore, we can stop scanning the bucket once we find
                        // an entry with cost larger than the best known assignment cost.
                        if (!anySet(atLeastAsGoodAsCurBest)) {
                            // The cost for an assignment after the last stop of vehId via v is certainly greater than the
                            // cost of the best currently known assignment. Therefore, all subsequent entries in the bucket
                            // will also be worse, and we can stop the bucket scan.
                            break;
                        }
                    }

                    // If this vehicle is not eligible, skip it.
                    if (!search.pruner.isVehicleEligible(vehId))
                        continue;


                    // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                    // than the current best and where distViaV is at least as good as the current tentative distance.
                    LabelMask mask = ~(search.tentativeDistances.getDistancesForCurBatch(vehId) < distViaV);
                    mask &= atLeastAsGoodAsCurBest;

                    // Don't update anywhere where distViaV >= INFTY
                    mask &= distViaV < INFTY;

                    if (mask) { // if any search requires updates, update the right ones according to mask
                        search.tentativeDistances.setDistancesForCurBatchIf(vehId, distViaV, mask);
                        search.vehiclesSeen.insert(vehId);
                        search.pruner.updateUpperBoundCost(vehId, distViaV);
                    }
                }

                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;


                return false;
            }

        private:
            LastStopBCHQuery &search;
        };


        struct StopLastStopBCH {
            explicit StopLastStopBCH(const LastStopBCHQuery &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) const {
                return allSet(search.pruner.isWorseThanUpperBoundCost(distToV, false));
            }

        private:
            const LastStopBCHQuery &search;

        };


    public:

        LastStopBCHQuery(TentativeLastStopDistances <LabelSetT> &tentativeLastStopDistances,
                const CHEnvT &chEnv,
                Subset &vehiclesSeen,
                PrunerT pruner)
                : upwardSearch(chEnv.template getReverseSearch<ScanSortedBucket, StopLastStopBCH, LabelSetT>(
                ScanSortedBucket(*this), StopLastStopBCH(*this))),
                  pruner(pruner),
                  ch(chEnv.getCH()),
                  bucketContainer(nullptr),
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

        void exchangeBuckets(const typename LastStopBucketsUpdaterT::BucketContainer &newBuckets) {
            bucketContainer = &newBuckets;
        }

    private:

        typename CHEnvT::template UpwardSearch<ScanSortedBucket, StopLastStopBCH, LabelSetT> upwardSearch;
        PrunerT pruner;

        const CH &ch;
        const typename LastStopBucketsUpdaterT::BucketContainer *bucketContainer;

        TentativeLastStopDistances <LabelSetT> &tentativeDistances;

        Subset &vehiclesSeen;
        int numVerticesSettled;
        int numEntriesVisited;
    };

}