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
                  bucketMembers(0),
                  stack(),
                  ptoQueue(),
                  inPTOQueue(upGraph.numVertices()) {
            stack.reserve(upGraph.numVertices());
            ptoQueue.reserve(upGraph.numVertices());
        }

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

            // Find out for which vertices in upward search space we need to construct the bucket (ptoQueue).
            computeVerticesToConstruct(rank);

            // For all vertices in queue, construct the bucket:
            for (const auto &v: ptoQueue) {
                KASSERT(!constructed.isSet(v));
                inPTOQueue[v] = false; // Reset for next call
                constructBucketForFinishedParents(v);
                constructed.set(v);
            }
            ptoQueue.clear();
        }

        // Given a source rank, traverses the upward search space and writes the vertices that are reachable from the
        // source and that do not already have a bucket to ptoQueue in post-traversal order.
        void computeVerticesToConstruct(const int source) {
            if (constructed.isSet(source))
                return;
            KASSERT(stack.empty() && ptoQueue.empty());
            stack.emplace_back(source, upGraph.firstEdge(source));
            while (!stack.empty()) {
                auto& pair = stack.back();
                auto& e = pair.second;
                const auto tail = pair.first;
                if (e >= upGraph.lastEdge(tail)) {
                    // All outgoing edges of tail have been processed and all its ancestors are already in ptoQueue.
                    // Add tail to ptoQueue and retreat to its parent, proceeding with next child of parent.
                    ptoQueue.push_back(tail);
                    inPTOQueue[tail] = true;
                    stack.pop_back();
                    ++stack.back().second; // Proceed with next outgoing edge of parent
                    continue;
                }
                const auto head = upGraph.edgeHead(e);
                if (inPTOQueue[head] || constructed.isSet(head)) {
                    // If child is already in ptoQueue or already has a bucket, proceed with next child.
                    ++stack.back().second;
                    continue;
                }
                // Advance to child and first outgoing edge of child
                stack.emplace_back(head, upGraph.firstEdge(head));
            }
        }

        void constructBucketForFinishedParents(const int rank) {
            KASSERT(!constructed.isSet(rank));

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
                KASSERT(constructed.isSet(neighbor));
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
        }

        const typename CH::SearchGraph &upGraph;
        const BucketContainerT &downBuckets;

        BucketContainerT upDownBuckets;
        FastResetFlagArray<> constructed;

        TimestampedVector<int> merger;
        std::vector<int> leeways;
        LightweightSubset bucketMembers;

        std::vector<std::pair<int, int>> stack;
        std::vector<int> ptoQueue;
        BitVector inPTOQueue;


    };
}