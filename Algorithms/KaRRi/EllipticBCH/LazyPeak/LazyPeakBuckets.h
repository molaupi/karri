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

#include "DataStructures/Graph/Graph.h"
#include "Algorithms/CH/CH.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "DataStructures/Containers/FastResetFlagArray.h"
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/KaRRi/EllipticBCH/BucketEntryWithLeeway.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "DataStructures/Containers/LightweightSubset.h"


namespace karri {

    // TODO: Adapt so lazy buckets don't have to be fully computed anew for every request.

    template<typename BucketContainerT>
    class LazyPeakBuckets {

    public:

        LazyPeakBuckets(const typename CH::SearchGraph &upGraph,
                        const BucketContainerT &downBuckets)
                : upGraph(upGraph),
                  downBuckets(downBuckets),
                  upDownBuckets(upGraph.numVertices()),
                  constructed(upGraph.numVertices()),
                  merger(0, INFTY),
                  bucketMembers(0) {}

        void init(const int numTargets) {
            upDownBuckets.clear();
            constructed.reset();

            if (numTargets > merger.size()) {
                merger.resize(numTargets);
                leeways.resize(numTargets);
                bucketMembers.resize(numTargets);
            }
        }

        BucketContainerT::Bucket getBucketOf(const int rank) {
            constructBucket(rank); // Constructs bucket if not already done
            return upDownBuckets.getBucketOf(rank);
        }

    private:

        void constructBucket(const int rank) {
            if (constructed.isSet(rank))
                return;

            // TODO: Replace recursion with iteration
            // Recurse on upward neighbors
            FORALL_INCIDENT_EDGES(upGraph, rank, e) {
                constructBucket(upGraph.edgeHead(e));
            }

            // Initialize up-down bucket at rank with down bucket at rank
            merger.clear();
            bucketMembers.clear();
            for (const auto &entry: downBuckets.getBucketOf(rank)) {
                merger[entry.targetId] = entry.distToTarget;
                bucketMembers.insert(entry.targetId);
                leeways[entry.targetId] = entry.leeway;
            }

            // Merge up-down buckets of all upward neighbors
            FORALL_INCIDENT_EDGES(upGraph, rank, e) {
                const int offset = upGraph.template get<CH::Weight>(e);
                const int neighbor = upGraph.edgeHead(e);
                for (const auto &entry: upDownBuckets.getBucketOf(neighbor)) {
                    auto &val = merger[entry.targetId];
                    val = std::min(val, offset + entry.distToTarget);
                    bucketMembers.insert(entry.targetId);
                    leeways[entry.targetId] = entry.leeway;
                }
            }

            // Piece information on bucket entries together into BucketEntryWithLeeway
            for (const auto &targetId: bucketMembers) {
                const auto &distToTarget = merger[targetId];
                KASSERT(distToTarget >= 0 && distToTarget < INFTY);
                const auto &leeway = leeways[targetId];
                if (distToTarget > leeway)
                    continue; // No leeway left for this entry
                upDownBuckets.insert(rank, BucketEntryWithLeeway(targetId, distToTarget, leeway));
            }

            constructed.set(rank);
        }

        const typename CH::SearchGraph &upGraph;
        const BucketContainerT &downBuckets;

        BucketContainerT upDownBuckets;
        FastResetFlagArray<uint16_t> constructed;

        TimestampedVector<int, std::vector> merger;
        std::vector<int> leeways;
        LightweightSubset bucketMembers;


    };
}