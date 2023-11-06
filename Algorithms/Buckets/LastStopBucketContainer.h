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

/// Stores upward distances from last stops of vehicles to vertices in buckets.
/// Stores entries for idle vehicles and non-idle vehicles separately in order to
/// sort them by distance to the vertex and arrival time at the vertex, respectively.
/// This sorting allows more precise early stopping of the bucket scan in both cases.
template<typename BucketEntryT, typename IdleComparatorT, typename NonIdleComparatorT>
class LastStopBucketContainer {

private:
    struct BucketPosition {
        int start = 0;
        int numIdleEntries = 0;
        int end = 0;
    };

public:

    using Bucket = ConstantVectorRange<BucketEntryT>;

    explicit LastStopBucketContainer(const int numVertices)
            : numEntriesVisited(0), idleComp(), nonIdleComp(), bucketPositions(numVertices) {}


    // Returns the bucket of idle vehicles at the specified vertex, sorted in ascending order according to
    // IdleEntryComparatorT.
    Bucket getIdleBucketOf(const int v) const {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        const auto &pos = bucketPositions[v];
        return Bucket(entries.begin() + pos.start, entries.begin() + pos.start + pos.numIdleEntries);
    }

    // Returns the bucket of non-idle vehicles at the specified vertex, sorted in ascending order according to
    // NonIdleEntryComparatorT.
    Bucket getNonIdleBucketOf(const int v) const {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        const auto &pos = bucketPositions[v];
        return Bucket(entries.begin() + pos.start + pos.numIdleEntries, entries.begin() + pos.end);
    }

    std::pair<Bucket, Bucket> getIdleAndNonIdleBucketOf(const int v) const {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        const auto &pos = bucketPositions[v];
        return {Bucket(entries.begin() + pos.start, entries.begin() + pos.start + pos.numIdleEntries),
                Bucket(entries.begin() + pos.start + pos.numIdleEntries, entries.begin() + pos.end)};

    }

    Bucket getUnsortedBucketOf(const int v) const {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        const auto &pos = bucketPositions[v];
        return Bucket(entries.begin() + pos.start, entries.begin() + pos.end);
    }

    // Inserts the given entry into the bucket of idle vehicles at the specified vertex.
    bool insertIdle(const int v, const BucketEntryT &entry) {
        auto &pos = bucketPositions[v];
        const auto col = searchForInsertionIdx(entry, pos.start, pos.start + pos.numIdleEntries, idleComp) - pos.start;
        stableInsertion(v, col, entry, bucketPositions, entries);
        ++pos.numIdleEntries;
        return true;
    }

    // Inserts the given entry into the bucket of non-idle vehicles at the specified vertex.
    bool insertNonIdle(const int v, const BucketEntryT &entry) {
        const auto &pos = bucketPositions[v];
        const auto col = searchForInsertionIdx(entry, pos.start + pos.numIdleEntries, pos.end, nonIdleComp) - pos.start;
        stableInsertion(v, col, entry, bucketPositions, entries);
        return true;
    }

    // Removes the given entry from the idle bucket of the specified vertex.
    // Since buckets are sorted, binary search can be used.
    bool removeIdle(const int v, const BucketEntryT &entry) {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        numEntriesVisited = 0;

        auto &pos = bucketPositions[v];
        int col = -1;
        const bool found = binarySearchForExistingEntry(entry, col, pos.start, pos.start + pos.numIdleEntries,
                                                        idleComp);
        if (!found)
            return false;

        col -= pos.start;
        stableRemoval(v, col, bucketPositions, entries);
        --pos.numIdleEntries;
        return true;
    }

    // Removes the given entry from the non-idle bucket of the specified vertex.
    // Since buckets are sorted, binary search can be used.
    bool removeNonIdle(const int v, const BucketEntryT &entry) {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        numEntriesVisited = 0;

        const auto &pos = bucketPositions[v];
        int col = -1;
        const bool found =
                binarySearchForExistingEntry(entry, col, pos.start + pos.numIdleEntries, pos.end, nonIdleComp);
        if (!found)
            return false;

        col -= pos.start;
        stableRemoval(v, col, bucketPositions, entries);
        return true;
    }

    // Removes the given entry from the non-idle bucket of the specified vertex.
    // Given only the vehicle id, we use linear search.
    bool removeNonIdle(const int v, const int vehId) {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        numEntriesVisited = 0;

        const auto &pos = bucketPositions[v];
        int col = -1;
        const bool found = linearSearchForExistingEntry(vehId, col, pos.start + pos.numIdleEntries, pos.end);
        if (!found)
            return false;

        col -= pos.start;
        stableRemoval(v, col, bucketPositions, entries);
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

    // Returns index between start and end (inclusive) where entry should be inserted to preserve order according to
    // comp. start, end, and return value are indices in entries vector.
    template<typename CompT>
    int searchForInsertionIdx(const BucketEntryT &entry, const int start, const int end, CompT &comp) {

        // Check if idle bucket is currently empty or new entry needs to become first element:
        if (start == end || comp(entry, entries[start]))
            return start;

        // Check if new entry needs to become last element:
        if (!comp(entry, entries[end - 1]))
            return end;

        // Binary search with invariant: !idleComp(entry, entries[l]) && idleComp(entry, entries[r])
        int l = start;
        int r = end - 1;
        while (l < r - 1) {
            assert(!comp(entry, entries[l]) && comp(entry, entries[r]));
            int m = (l + r) / 2;
            if (comp(entry, entries[m])) {
                r = m;
            } else {
                l = m;
            }
        }

        return r;
    }

    // Returns whether e1 and e2 are equivalent wrt comp.
    template<typename CompT>
    bool equiv(const BucketEntryT &e1, const BucketEntryT &e2, CompT &comp) const {
        return !comp(e1, e2) && !comp(e2, e1);
    }

    // Finds existing entry between indices start and end (inclusive) assuming entries are ordered by comp.
    // If found, idx is set to index of entry and true is returned. Otherwise, false is returned.
    // start, end, and idx are indices in entries vector.
    template<typename CompT>
    bool
    binarySearchForExistingEntry(const BucketEntryT &entry, int &idx, const int start, const int end, CompT &comp) {
        // Check if bucket is currently empty or is smaller than smallest entry in bucket or larger
        // than largest entry in bucket.
        numEntriesVisited += 2;
        if (end == start || comp(entry, entries[start]) || comp(entries[end - 1], entry))
            return false;

        // Check if entry is equal to last entry wrt to comp. If so, scan all entries that are equal backwards linearly.
        if (equiv(entry, entries[end - 1], comp)) {
            idx = end - 1;
            while (idx >= start && equiv(entry, entries[idx], comp) &&
                   entry.targetId != entries[idx].targetId) {
                ++numEntriesVisited;
                --idx;
            }
            return idx >= start && entry.targetId == entries[idx].targetId;
        }

        // Binary search with invariant: !comp(entry, entries[pos.start + l]) && comp(entry, entries[pos.start + r])
        int l = start, r = end - 1;
        while (l < r - 1) {
            assert(!comp(entry, entries[l]) && comp(entry, entries[r]));
            int m = (l + r) / 2;
            ++numEntriesVisited;
            if (comp(entry, entries[m])) {
                r = m;
            } else {
                l = m;
            }
        }

        // r is position right of range of entries that are equivalent to entry wrt comp (range may be empty if entry
        // does not exist). Scan through this range backwards linearly:
        idx = r - 1;
        while (idx >= start && equiv(entry, entries[idx], comp) &&
               entry.targetId != entries[idx].targetId) {
            ++numEntriesVisited;
            --idx;
        }
        return idx >= start && entry.targetId == entries[idx].targetId;
    }


    bool linearSearchForExistingEntry(const int vehId, int &idx, const int start, const int end) {
        for (idx = start; idx < end; ++idx) {
            if (entries[idx].targetId == vehId)
                return true;
        }
        return false;
    }


    int numEntriesVisited;
    IdleComparatorT idleComp;
    NonIdleComparatorT nonIdleComp;

    std::vector<BucketPosition> bucketPositions;
    std::vector<BucketEntryT> entries;
};