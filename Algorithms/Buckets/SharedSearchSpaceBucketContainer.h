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

    using Bucket = ConstantVectorRange<BucketEntryT>;

    // Constructs a container that can maintain buckets for the specified number of vertices.
    explicit SharedSearchSpaceBucketContainer(const int numVertices)
            : numSearches(0), offsetForVertex(numVertices, INVALID_INDEX), entries(0) {
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
    bool insertOrUpdate(const int vertex, const BucketEntryT &newEntry) {
        assert(vertex >= 0);
        assert(vertex < offsetForVertex.size());
        assert(newEntry.targetId >= 0);
        assert(newEntry.targetId < numSearches);

        // If this is the first time any search inserts an entry at this vertex, allocate entries for all searches at
        // this vertex, i.e. construct entries for all possible searches.
        if (offsetForVertex[vertex] == INVALID_INDEX) {
            offsetForVertex[vertex] = entries.size();
            entries.insert(entries.end(), numSearches, BucketEntryT());
        }

        // If the entries for this vertex already exist, update the according entry
        auto &entry = entries[offsetForVertex[vertex] + newEntry.targetId];
        assert(entry.targetId == BucketEntryT().targetId || entry.targetId == newEntry.targetId);
        entry.cmpAndUpdate(newEntry);

        return true;
    }

    // Removes all entries from all buckets.
    void init(const int newNumSearches) {
        numSearches = newNumSearches;
        offsetForVertex.clear();
        entries.clear();
    }

private:

    int numSearches;
    TimestampedVector<int> offsetForVertex;
    std::vector<BucketEntryT> entries;
};
