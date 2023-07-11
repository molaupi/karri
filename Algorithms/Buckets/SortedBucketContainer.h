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

#include <vector>
#include <cassert>
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"


template<typename BucketEntryT, typename BucketEntryComparatorT>
class SortedBucketContainer {

public:

    using SortedBucket = ConstantVectorRange<BucketEntryT>;

    explicit SortedBucketContainer(const int numVertices)
            : numEntriesVisited(0), comp(), bucketPositions(numVertices) {}


    // Returns the bucket of the specified vertex, sorted in ascending order according to BucketEntryComparatorT.
    SortedBucket getBucketOf(const int v) const {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        const auto &pos = bucketPositions[v];
        return SortedBucket(entries.begin() + pos.start, entries.begin() + pos.end);
    }

    // Inserts the given entry into the bucket of the specified vertex.
    bool insert(const int v, const BucketEntryT &entry) {
        const auto pos = findInsertionPosForEntryInBucket(v, entry);
        stableInsertion(v, pos, entry, bucketPositions, entries);
        return true;
    }

    // Applies the given transformation to all entries in the bucket at vertex.
    // The transformation must be callable with a single entry of type BucketEntryT& as an argument and must return
    // bool stating whether an update was performed.
    // Returns true if any entry was updated and false otherwise.
    template<typename TransformationT>
    bool updateAllEntries(const int vertex, const TransformationT &transform) {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());

        const auto &pos = bucketPositions[vertex];
        assert(std::is_sorted(entries.begin() + pos.start, entries.begin() + pos.end,
                              [&](const auto &e1, const auto &e2) { return comp(e1, e2); }));

        numEntriesVisited = 0;
        bool anyUpdated = false;
        for (auto i = pos.start; i < pos.end; ++i) {
            ++numEntriesVisited;
            const bool transformed = transform(entries[i]);
            anyUpdated |= transformed;
        }
        if (!anyUpdated)
            return false;

        // If any entries were updated, fix sorting of bucket:
        std::sort(entries.begin() + pos.start, entries.begin() + pos.end,
                  [&](const auto &e1, const auto &e2) { return comp(e1, e2); });
        return true;
    }

    // Removes the entry for targetId from the bucket of the specified vertex.
    // Linear search for the targetId in the bucket.
    bool remove(const int v, const int targetId) {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        numEntriesVisited = 0;

        const auto &pos = bucketPositions[v];
        auto i = pos.start;
        while (i < pos.end) {
            ++numEntriesVisited;
            if (entries[i].targetId == targetId) {
                break;
            }
            ++i;
        }
        if (i == pos.end) return false;

        stableRemoval(v, i - pos.start, bucketPositions, entries);
        return true;
    }

    // Removes the given entry from the bucket of the specified vertex.
    // Since buckets are sorted, binary search can be used.
    bool remove(const int v, const BucketEntryT &entry) {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        numEntriesVisited = 0;

        int posInBucket = -1;
        const bool found = findPosOfExistingEntryInBucket(v, entry, posInBucket);
        if (!found)
            return false;

        stableRemoval(v, posInBucket, bucketPositions, entries);
        return true;
    }

    int getNumEntriesVisitedInLastUpdateOrRemove() const {
        return numEntriesVisited;
    }

    // Removes all entries from all buckets.
    void clear() {
        for (auto &bucketPos: bucketPositions)
            bucketPos.end = bucketPos.start;
        std::fill(entries.begin(), entries.end(), BucketEntryT());
    }

private:


    int findInsertionPosForEntryInBucket(const int v, const BucketEntryT &entry) {
        assert(v >= 0);
        assert(v < bucketPositions.size());

        const auto &pos = bucketPositions[v];

        // Check if bucket is currently empty or new entry needs to become first element:
        if (pos.end == pos.start || comp(entry, entries[pos.start]))
            return 0;

        // Check if new entry needs to become last element:
        if (!comp(entry, entries[pos.end - 1]))
            return pos.end - pos.start;

        // Binary search with invariant: !comp(entry, entries[pos.start + l]) && comp(entry, entries[pos.start + r])
        int l = 0, r = pos.end - pos.start - 1;
        while (l < r - 1) {
            assert(!comp(entry, entries[pos.start + l]) && comp(entry, entries[pos.start + r]));
            int m = (l + r) / 2;
            if (comp(entry, entries[pos.start + m])) {
                r = m;
            } else {
                l = m;
            }
        }

        return r;
    }

    // Returns whether e1 and e2 are equivalent wrt comp.
    bool equiv(const BucketEntryT &e1, const BucketEntryT &e2) const {
        return !comp(e1, e2) && !comp(e2, e1);
    }

    // Returns whether position was found. If so, position will be written to idx.
    bool findPosOfExistingEntryInBucket(const int v, const BucketEntryT &entry, int &idx) {
        assert(v >= 0);
        assert(v < bucketPositions.size());

        const auto &pos = bucketPositions[v];

        // Check if bucket is currently empty or is smaller than smallest entry in bucket or larger
        // than largest entry in bucket.
        numEntriesVisited += 2;
        if (pos.end == pos.start || comp(entry, entries[pos.start]) || comp(entries[pos.end - 1], entry))
            return false;

        // Check if entry is equal to last entry wrt to comp. If so, scan all entries that are equal backwards linearly.
        if (equiv(entry, entries[pos.end - 1])) {
            idx = pos.end - pos.start - 1;
            while (idx >= 0 && equiv(entry, entries[pos.start + idx]) &&
                   entry.targetId != entries[pos.start + idx].targetId) {
                ++numEntriesVisited;
                --idx;
            }
            return idx >= 0 && entry.targetId == entries[pos.start + idx].targetId;
        }

        // Binary search with invariant: !comp(entry, entries[pos.start + l]) && comp(entry, entries[pos.start + r])
        int l = 0, r = pos.end - pos.start - 1;
        while (l < r - 1) {
            assert(!comp(entry, entries[pos.start + l]) && comp(entry, entries[pos.start + r]));
            int m = (l + r) / 2;
            ++numEntriesVisited;
            if (comp(entry, entries[pos.start + m])) {
                r = m;
            } else {
                l = m;
            }
        }

        // r is position right of range of entries that are equivalent to entry wrt comp (range may be empty if entry
        // does not exist). Scan through this range backwards linearly:
        idx = r - 1;
        while (idx >= 0 && equiv(entry, entries[pos.start + idx]) &&
               entry.targetId != entries[pos.start + idx].targetId) {
            ++numEntriesVisited;
            --idx;
        }
        return idx >= 0 && entry.targetId == entries[pos.start + idx].targetId;
    }


    int numEntriesVisited;
    BucketEntryComparatorT comp;

    using BucketPosition = ValueBlockPosition;
    std::vector<BucketPosition> bucketPositions;
    std::vector<BucketEntryT> entries;
};