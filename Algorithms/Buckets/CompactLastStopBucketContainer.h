/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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
#include <kassert/kassert.hpp>
#include "CompactSortedBucketContainer.h"

/// Stores upward distances from last stops of vehicles to vertices in buckets.
/// Stores entries for idle vehicles and non-idle vehicles separately in order to
/// sort them by distance to the vertex and arrival time at the vertex, respectively.
/// This sorting allows more precise early stopping of the bucket scan in both cases.
/// As opposed to LastStopBucketContainer, buckets are stored in a compact manner in the order of CH rank, without
/// gaps of invalid entries between buckets. Thus, the bucket can be read in a more cache-efficient way in CH
/// upward searches. However, updates require more work. This is okay for batched updates.
template<typename BucketEntryT, typename IdleComparatorT, typename NonIdleComparatorT>
class CompactLastStopBucketContainer {

public:

    using EntryInsertion = compact_batch_ragged2d::Insertion<BucketEntryT>;
    using EntryDeletion = compact_batch_ragged2d::Deletion;

    template<typename T>
    using EntryVectorT = parallel::scalable_aligned_vector<T>;
    using Bucket = ConstantVectorRange<BucketEntryT, EntryVectorT>;

    explicit CompactLastStopBucketContainer(const int numVertices)
            : idleComp(), nonIdleComp(), offset(numVertices + 1, 0), numIdleEntries(numVertices, 0) {}


    // Returns the bucket of idle vehicles at the specified vertex, sorted in ascending order according to
    // IdleEntryComparatorT.
    Bucket getIdleBucketOf(const int v) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        KASSERT(start + numIdleEntries[v] <= offset[v + 1]);
        return Bucket(entries.begin() + start, entries.begin() + start + numIdleEntries[v]);
    }

    // Returns the bucket of non-idle vehicles at the specified vertex, sorted in ascending order according to
    // NonIdleEntryComparatorT.
    Bucket getNonIdleBucketOf(const int v) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        const auto end = offset[v + 1];
        KASSERT(start + numIdleEntries[v] <= end);
        return Bucket(entries.begin() + start + numIdleEntries[v], entries.begin() + end);
    }

    std::pair<Bucket, Bucket> getIdleAndNonIdleBucketOf(const int v) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        const auto end = offset[v + 1];
        KASSERT(start + numIdleEntries[v] <= end);
        return {Bucket(entries.begin() + start, entries.begin() + start + numIdleEntries[v]),
                Bucket(entries.begin() + start + numIdleEntries[v], entries.begin() + end)};

    }

    Bucket getUnsortedBucketOf(const int v) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        const auto end = offset[v + 1];
        return Bucket(entries.begin() + start, entries.begin() + end);
    }

    bool getIdleEntryInsertion(const int v, const BucketEntryT &entry, EntryInsertion &ins) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        KASSERT(start + numIdleEntries[v] <= offset[v + 1]);
        // TODO: change searchForInsertionIdx to return index relative to bucket ?
        const auto col = searchForInsertionIdx(entry, start, start + numIdleEntries[v], idleComp) - start;
        ins = {v, col, entry};
        return true;
    }

    bool getNonIdleEntryInsertion(const int v, const BucketEntryT &entry, EntryInsertion &ins) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        const auto end = offset[v + 1];
        KASSERT(start + numIdleEntries[v] <= end);
        // TODO: change searchForInsertionIdx to return index relative to bucket ?
        const auto col = searchForInsertionIdx(entry, start + numIdleEntries[v], end, nonIdleComp) - start;
        ins = {v, col, entry};
        return true;
    }

    bool
    getIdleEntryDeletion(const int v, const BucketEntryT &entry, EntryDeletion &del) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        KASSERT(start + numIdleEntries[v] <= offset[v + 1]);

        int col = -1;
        const bool found = binarySearchForExistingEntry(entry, col, start, start + numIdleEntries[v], idleComp);
        if (!found)
            return false;

        // TODO: change binarySearchForExistingEntry to return index relative to bucket ?
        col -= start;
        del = {v, col};
        return true;
    }

    bool getNonIdleEntryDeletion(const int v, const BucketEntryT &entry, EntryDeletion &del) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        const auto end = offset[v + 1];
        KASSERT(start + numIdleEntries[v] <= offset[v + 1]);

        int col = -1;
        const bool found = binarySearchForExistingEntry(entry, col, start + numIdleEntries[v], end, idleComp);
        if (!found)
            return false;

        // TODO: change binarySearchForExistingEntry to return index relative to bucket ?
        col -= start;
        del = {v, col};
        return true;
    }

    bool getNonIdleEntryDeletion(const int v, const int vehId, EntryDeletion &del) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        const auto end = offset[v + 1];
        KASSERT(start + numIdleEntries[v] <= offset[v + 1]);

        int col = -1;
        const bool found = linearSearchForExistingEntry(vehId, col, start + numIdleEntries[v], end);
        if (!found)
            return false;

        // TODO: change linearSearchForExistingEntry to return index relative to bucket ?
        col -= start;
        del = {v, col};
        return true;
    }


    void batchedCommitInsertions(std::vector<EntryInsertion> &idleInsertions,
                                 std::vector<EntryInsertion> &nonIdleInsertions) {

        // Sort idle insertions using idleComp.
        std::sort(idleInsertions.begin(), idleInsertions.end(),
                  [&](const EntryInsertion &i1, const EntryInsertion &i2) {
                      return idleComp(i1.value, i2.value);
                  });

        // Sort non-idle insertions using nonIdleComp.
        std::sort(nonIdleInsertions.begin(), nonIdleInsertions.end(),
                  [&](const EntryInsertion &i1, const EntryInsertion &i2) {
                      return nonIdleComp(i1.value, i2.value);
                  });

        // Update number of idle entries for each idle insertion.
        for (const auto &ins: idleInsertions) {
            ++numIdleEntries[ins.row];
        }

        // Append non-idle insertions to idle insertions, then run batched update for all insertions. Stable
        // batched update respects order by idleComp and nonIdleComp as well as the fact that idle entries have to
        // come before non-idle ones.
        idleInsertions.insert(idleInsertions.end(), nonIdleInsertions.begin(), nonIdleInsertions.end());

        // Perform insertions. Respects previous sorting by comp.
        compact_batch_ragged2d::stableBatchedInsertionsSequential(idleInsertions, offset, entries);

        KASSERT(std::all_of(entries.begin(), entries.end(), [&](const auto &e) { return e != BucketEntryT(); }));
        KASSERT(verifyAllBucketsSorted());
    }


    void
    batchedCommitDeletions(std::vector<EntryDeletion> &idleDeletions, std::vector<EntryDeletion> &nonIdleDeletions) {
        KASSERT(entries.size() >= idleDeletions.size() + nonIdleDeletions.size());

        // Update number of idle entries for each idle deletion.
        for (const auto &del: idleDeletions) {
            --numIdleEntries[del.row];
        }

        // Append nonIdleDeletions to idleDeletions and perform batched update for all deletions
        idleDeletions.insert(idleDeletions.end(), nonIdleDeletions.begin(), nonIdleDeletions.end());
        compact_batch_ragged2d::stableBatchedDeletionsSequential(idleDeletions, offset, entries);
        KASSERT(verifyAllBucketsSorted());
    }

    void batchedCommitInsertionsAndDeletions(std::vector<EntryInsertion> &idleInsertions,
                                             std::vector<EntryInsertion> &nonIdleInsertions,
                                             std::vector<EntryDeletion> &idleDeletions,
                                             std::vector<EntryDeletion> &nonIdleDeletions) {
        KASSERT(entries.size() >= idleDeletions.size() + nonIdleDeletions.size());

        // Sort idle insertions using idleComp.
        std::sort(idleInsertions.begin(), idleInsertions.end(),
                  [&](const EntryInsertion &i1, const EntryInsertion &i2) {
                      return idleComp(i1.value, i2.value);
                  });

        // Sort non-idle insertions using nonIdleComp.
        std::sort(nonIdleInsertions.begin(), nonIdleInsertions.end(),
                  [&](const EntryInsertion &i1, const EntryInsertion &i2) {
                      return nonIdleComp(i1.value, i2.value);
                  });

        // Update number of idle entries for each idle insertion.
        for (const auto &ins: idleInsertions) {
            ++numIdleEntries[ins.row];
        }

        // Append non-idle insertions to idle insertions, then run batched update for all insertions. Stable
        // batched update respects order by idleComp and nonIdleComp as well as the fact that idle entries have to
        // come before non-idle ones.
        idleInsertions.insert(idleInsertions.end(), nonIdleInsertions.begin(), nonIdleInsertions.end());

        // Update number of idle entries for each idle deletion.
        for (const auto &del: idleDeletions) {
            --numIdleEntries[del.row];
        }

        // Append nonIdleDeletions to idleDeletions and perform batched update for all deletions
        idleDeletions.insert(idleDeletions.end(), nonIdleDeletions.begin(), nonIdleDeletions.end());

        compact_batch_ragged2d::stableBatchedInsertionsAndDeletionsParallel(idleInsertions, idleDeletions, offset,
                                                                              entries);
        KASSERT(verifyAllBucketsSorted());
    }


    // Removes all entries from all buckets.
    void clear() {
        for (auto &o: offset)
            o = 0;
        entries.clear();
    }

private:

    // Returns index between start and end (inclusive) where entry should be inserted to preserve order according to
    // comp. start, end, and return value are indices in entries vector.
    template<typename CompT>
    int searchForInsertionIdx(const BucketEntryT &entry, const int start, const int end, const CompT &comp) const {

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
            KASSERT(!comp(entry, entries[l]) && comp(entry, entries[r]));
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
    binarySearchForExistingEntry(const BucketEntryT &entry, int &idx, const int start, const int end,
                                 const CompT &comp) const {
        // Check if bucket is currently empty or is smaller than smallest entry in bucket or larger
        // than largest entry in bucket.
        if (end == start || comp(entry, entries[start]) || comp(entries[end - 1], entry))
            return false;

        // Check if entry is equal to last entry wrt to comp. If so, scan all entries that are equal backwards linearly.
        if (equiv(entry, entries[end - 1], comp)) {
            idx = end - 1;
            while (idx >= start && equiv(entry, entries[idx], comp) &&
                   entry.targetId != entries[idx].targetId) {
                --idx;
            }
            return idx >= start && entry.targetId == entries[idx].targetId;
        }

        // Binary search with invariant: !comp(entry, entries[pos.start + l]) && comp(entry, entries[pos.start + r])
        int l = start, r = end - 1;
        while (l < r - 1) {
            KASSERT(!comp(entry, entries[l]) && comp(entry, entries[r]));
            int m = (l + r) / 2;
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
            --idx;
        }
        return idx >= start && entry.targetId == entries[idx].targetId;
    }


    bool linearSearchForExistingEntry(const int vehId, int &idx, const int start, const int end) const {
        for (idx = start; idx < end; ++idx) {
            if (entries[idx].targetId == vehId)
                return true;
        }
        return false;
    }

    bool verifyAllBucketsSorted() const {
        for (int v = 0; v < offset.size() - 1; ++v) {
            const auto start = offset[v];
            const auto end = offset[v + 1];
            const auto numIdle = numIdleEntries[v];
            KASSERT(start + numIdle <= end);
            const bool idleSorted = std::is_sorted(entries.begin() + start, entries.begin() + start + numIdle,
                                                   [&](const auto &e1, const auto &e2) { return idleComp(e1, e2); });
            KASSERT(idleSorted);

            const bool nonIdleSorted = std::is_sorted(entries.begin() + start + numIdle, entries.begin() + end,
                                                      [&](const auto &e1, const auto &e2) {
                                                          return nonIdleComp(e1, e2);
                                                      });
            KASSERT(nonIdleSorted);
            if (!idleSorted || !nonIdleSorted)
                return false;
        }
        return true;
    }

    IdleComparatorT idleComp;
    NonIdleComparatorT nonIdleComp;

    // Buckets are stored without gaps between (as opposed to LastStopBucketContainer).
    // Bucket of vertex v is stored at entries[ offset[v] .. offset[v + 1] ].
    // Each bucket is sorted internally according to comp.
    using Offset = int32_t;
    std::vector<Offset> offset;
    EntryVectorT<BucketEntryT> entries;

    // Bucket for vertex v has numIdleEntries[v] idle entries and offset[v + 1] - offset[v] - numIdleEntries[v] non-idle
    // entries with the idle entries being stored at the beginning of the bucket and the non-idle entries after.
    std::vector<int> numIdleEntries;
};