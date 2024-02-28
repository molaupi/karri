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

// A bucket container that is meant to be used when the set of sources of labels is known in advance and their search
// spaces in the CH can be expected to be similar. Let k be the number of sources for which this bucket container will
// contain entries. Whenever a bucket entry for a source s is inserted into the bucket of a vertex v for which the
// container has no entries yet, the container allocates all k possible entries for v and assigns dist=Infinity to all
// entries (apart from the inserted one). Then, if another entry is to be inserted for a source s' at v when v already
// has entries, the already existing entry for s is simply updated. Under the assumption that the search spaces of all
// sources are similar, this allows a dynamically growing container that does not waste a lot of memory.
// Moreover, this container then allows random access to the distance label dist(s,v) for any source s and vertex v
// (which in other containers would require iterating through the entire bucket of v).
//
// Requires BucketEntryT to be default constructible and to provide a method e.cmpAndUpdate(const BucketEntryT& e) that
// compares an existing entry e against a given entry e' and updates e accordingly. (This should usually consist of
// comparing the distances in both entries).
template<typename BucketEntryT>
class SharedSearchSpaceBucketContainer {
    static_assert(std::is_default_constructible<BucketEntryT>());
public:

    using Bucket = ConstantConcurrentVectorRange<BucketEntryT>;

    // Represents information that one thread computes during one bucket search. Can be incorporated into
    // global result at end of search.
    class ThreadLocalBuckets {

        friend SharedSearchSpaceBucketContainer;

    public:

        ThreadLocalBuckets(std::vector<std::pair<int, BucketEntryT>> &pairs) :
                pairs(pairs) {}

        void initForSearch() {
            pairs.clear();
        }

        bool insertOrUpdate(const int v, const BucketEntryT &newEntry) {
//            const int targetId = newEntry.targetId;
            assert(newEntry.targetId >= 0);

            // If no entries exist yet for this vertex, add an entry, otherwise compare to existing entry.
            auto it = std::find_if(pairs.begin(), pairs.end(), [&](const auto &pair) { return pair.first == v; });
            if (it == pairs.end()) {
                pairs.push_back({v, newEntry});
                return true;
            }

            auto &entry = it->second;
            assert(entry.targetId == BucketEntryT().targetId || entry.targetId == newEntry.targetId);
            entry.cmpAndUpdate(newEntry);

            return true;
        }

    private:

        std::vector<std::pair<int, BucketEntryT>> &pairs;

    };

    // Constructs a container that can maintain buckets for the specified number of vertices.
    explicit SharedSearchSpaceBucketContainer(const int numVertices)
            : numSearches(0),
              vertexLocks(numVertices, SpinLock()),
              offsetForVertex(numVertices, INVALID_INDEX),
              localResultPairs(),
              seedCounter(0) {
        assert(numVertices >= 0);
    }

    // Returns the bucket of the specified vertex.
    Bucket getBucketOf(const int vertex) {
        assert(vertex >= 0);
        assert(vertex < offsetForVertex.size());

        if (offsetForVertex[vertex] == INVALID_INDEX)
            return Bucket(entries.begin(), entries.begin());

        return Bucket(entries.begin() + offsetForVertex[vertex],
                      entries.begin() + offsetForVertex[vertex] + numSearches);
    }

    // Inserts the given entry into the bucket of the specified vertex.
    // bool insertOrUpdate(const int vertex, const BucketEntryT &newEntry) {
    //     assert(vertex >= 0);
    //     assert(vertex < offsetForVertex.size());
    //     assert(newEntry.targetId >= 0);
    //     assert(newEntry.targetId < numSearches);

    //     // If this is the first time any search inserts an entry at this vertex, allocate entries for all searches at
    //     // this vertex, i.e. construct entries for all possible searches.
    //     if (offsetForVertex[vertex] == INVALID_INDEX) {
    //         offsetForVertex[vertex] = entries.size();
    //         entries.insert(entries.end(), numSearches, BucketEntryT());
    //     }

    //     // If the entries for this vertex already exist, update the according entry
    //     auto &entry = entries[offsetForVertex[vertex] + newEntry.targetId];
    //     assert(entry.targetId == BucketEntryT().targetId || entry.targetId == newEntry.targetId);
    //     entry.cmpAndUpdate(newEntry);

    //     return true;
    // }

    void updateBucketEntriesInGlobalVectors() {

//        const auto &localIndices = indexInBucketEntriesVector.local();
//        const auto &localEntries = bucketEntriesForSingleVertex.local();

        const auto &localPairs = localResultPairs.local();

        const int seed = seedCounter.fetch_add(1, std::memory_order_relaxed);
        const auto permutation = Permutation::getRandomPermutation(localPairs.size(), std::minstd_rand(seed));

        for (const auto &idx: permutation) {
            const auto& pair = localPairs[idx];
            const auto &vertex = pair.first;
            const auto &localEntry = pair.second;
            allocateEntriesFor(vertex);
            assert(entries[offsetForVertex[vertex] + localEntry.targetId].targetId == BucketEntryT().targetId ||
                   entries[offsetForVertex[vertex] + localEntry.targetId].targetId == localEntry.targetId);
            entries[offsetForVertex[vertex] + localEntry.targetId] = localEntry;
        }
    }

    // Removes all entries from all buckets.
    void init(const int newNumSearches) {
        numSearches = newNumSearches;
        entries.clear();

        for (const auto &v: verticesWithEntries)
            offsetForVertex[v] = INVALID_INDEX;
        verticesWithEntries.clear();
    }

    // Each thread gets an instance of a ThreadLocalBuckets at the beginning of a search. This
    // object encapsulates the local result of the thread for that search. This way, the underlying TLS structures
    // are only queried once per search.
    ThreadLocalBuckets getThreadLocalBuckets() {
        return ThreadLocalBuckets(localResultPairs.local());
//        return ThreadLocalBuckets(numSearches, indexInBucketEntriesVector.local(), bucketEntriesForSingleVertex.local());
    }

private:
    void allocateEntriesFor(const int vertex) {
        SpinLock &currLock = vertexLocks[vertex];
        currLock.lock();

        if (offsetForVertex[vertex] != INVALID_INDEX) {
            currLock.unlock();
            return;
        }

        const auto entriesIt = entries.grow_by(numSearches, BucketEntryT());
        offsetForVertex[vertex] = entriesIt - entries.begin();
        verticesWithEntries.push_back(vertex);

        currLock.unlock();
    }

    int numSearches;
    std::vector<SpinLock> vertexLocks;

//    TimestampedVector<int> offsetForVertex;
    tbb::concurrent_vector<int> verticesWithEntries;
    std::vector<int> offsetForVertex;
    tbb::concurrent_vector<BucketEntryT> entries;

    // Thread Local Storage for local bucket entries calculation
//    tbb::enumerable_thread_specific<std::vector<int>> indexInBucketEntriesVector;
//    tbb::enumerable_thread_specific<std::vector<BucketEntryT>> bucketEntriesForSingleVertex;
    tbb::enumerable_thread_specific<std::vector<std::pair<int, BucketEntryT>>> localResultPairs;

    // Atomic used to generate unique seeds for creating random permutations.
    CAtomic<int> seedCounter;
};
