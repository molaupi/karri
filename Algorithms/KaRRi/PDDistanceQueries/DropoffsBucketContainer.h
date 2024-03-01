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

#include <algorithm>
#include <cassert>
#include <vector>

#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "DataStructures/Containers/TimestampedVector.h"

#include "Parallel/atomic_wrapper.h"
#include "tbb/enumerable_thread_specific.h"
#include "tbb/concurrent_vector.h"
#include "DataStructures/Containers/ThreadSafeSubset.h"

template<typename DistanceLabelT>
class DropoffsBucketContainer {
public:

    using Bucket = ConstantConcurrentVectorRange<DistanceLabelT>;

    // Constructs a container that can maintain buckets for the specified number of vertices.
    explicit DropoffsBucketContainer(const int numVertices)
            : numSearches(0),
              vertexLocks(numVertices, SpinLock()),
              verticesWithEntries(numVertices),
              offsetForVertex(numVertices, INVALID_INDEX) {
        assert(numVertices >= 0);
    }

    // Returns the bucket of the specified vertex.
    Bucket getBucketOf(const int vertex) {
        assert(vertex >= 0);
        assert(vertex < offsetForVertex.size());

        if (offsetForVertex[vertex] == INVALID_INDEX)
            return Bucket(distances.begin(), distances.begin());

        return Bucket(distances.begin() + offsetForVertex[vertex],
                      distances.begin() + offsetForVertex[vertex] + numSearches);
    }

    void updateDistancesInGlobalVectors(const int batchId,
                                        const std::vector<std::pair<int, DistanceLabelT>> &localSearchSpaceWithDistances) {

        std::vector<std::pair<int, DistanceLabelT> const *> retries;

        // todo: Eliminate all but last occurrence of each vertex in search space before calling this.

        // Attempt 1: Give up on pairs where vertex lock cannot be obtained and mark them as retry.
        for (const auto& pair : localSearchSpaceWithDistances) {
            const auto &vertex = pair.first;
            const auto &dist = pair.second;
            if (!tryAllocateEntriesFor(vertex)) {
                retries.push_back(&pair);
                continue;
            }
            distances[offsetForVertex[vertex] + batchId].min(dist);
        }

        // Attempt 2: Wait for locks for retries from attempt 1.
        for (const auto& pair : retries) {
            const auto &vertex = pair->first;
            const auto &dist = pair->second;
            allocateEntriesFor(vertex);
            distances[offsetForVertex[vertex] + batchId].min(dist);
        }
    }

    // Removes all entries from all buckets.
    void init(const int newNumSearches) {
        numSearches = newNumSearches;
        distances.clear();

        for (const auto &v: verticesWithEntries)
            offsetForVertex[v] = INVALID_INDEX;
        verticesWithEntries.clear();
    }

private:

    // Checks if entry already exists for vertex, otherwise allocates one.
    // If vertex already has entries per ThreadSafeSubset check or lock can be acquired, return true.
    // Otherwise, if lock cannot be acquired, return false.
    bool tryAllocateEntriesFor(const int vertex) {
        if (verticesWithEntries.contains(vertex))
            return true;

        SpinLock &currLock = vertexLocks[vertex];
        if (!currLock.tryLock())
            return false;

        if (offsetForVertex[vertex] != INVALID_INDEX) {
            currLock.unlock();
            return true;
        }

        const auto entriesIt = distances.grow_by(numSearches, DistanceLabelT(INFTY));
        offsetForVertex[vertex] = entriesIt - distances.begin();

        currLock.unlock();
        verticesWithEntries.insert(vertex);
        return true;
    }

    void allocateEntriesFor(const int vertex) {
        if (verticesWithEntries.contains(vertex))
            return;

        SpinLock &currLock = vertexLocks[vertex];
        currLock.lock();

        if (offsetForVertex[vertex] != INVALID_INDEX) {
            currLock.unlock();
            return;
        }

        const auto entriesIt = distances.grow_by(numSearches, DistanceLabelT(INFTY));
        offsetForVertex[vertex] = entriesIt - distances.begin();

        currLock.unlock();
        verticesWithEntries.insert(vertex);
    }

    int numSearches;
    std::vector<SpinLock> vertexLocks;

    karri::ThreadSafeSubset verticesWithEntries;
    std::vector<int> offsetForVertex;
    tbb::concurrent_vector<DistanceLabelT> distances;
};
