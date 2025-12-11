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
            : idleComp(), nonIdleComp(), bucketPositions(numVertices) {}


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
        dynamic_ragged2d::stableInsertion(v, col, entry, bucketPositions, entries);
        ++pos.numIdleEntries;
        return true;
    }

    // Inserts the given entry into the bucket of non-idle vehicles at the specified vertex.
    bool insertNonIdle(const int v, const BucketEntryT &entry) {
        const auto &pos = bucketPositions[v];
        const auto col = searchForInsertionIdx(entry, pos.start + pos.numIdleEntries, pos.end, nonIdleComp) - pos.start;
        dynamic_ragged2d::stableInsertion(v, col, entry, bucketPositions, entries);
        return true;
    }

    // Removes the given entry from the idle bucket of the specified vertex.
    // Since buckets are sorted, binary search can be used.
    bool removeIdle(const int v, const BucketEntryT &entry, int64_t &numEntriesVisited) {
        assert(v >= 0);
        assert(v < bucketPositions.size());

        auto &pos = bucketPositions[v];
        int col = -1;
        const bool found = binarySearchForExistingEntry(entry, col, pos.start, pos.start + pos.numIdleEntries,
                                                        idleComp, numEntriesVisited);
        if (!found)
            return false;

        col -= pos.start;
        dynamic_ragged2d::stableRemoval(v, col, bucketPositions, entries);
        --pos.numIdleEntries;
        return true;
    }

    // Removes the given entry from the non-idle bucket of the specified vertex.
    // Since buckets are sorted, binary search can be used.
    bool removeNonIdle(const int v, const BucketEntryT &entry, int64_t &numEntriesVisited) {
        assert(v >= 0);
        assert(v < bucketPositions.size());

        const auto &pos = bucketPositions[v];
        int col = -1;
        const bool found =
                binarySearchForExistingEntry(entry, col, pos.start + pos.numIdleEntries, pos.end, nonIdleComp, numEntriesVisited);
        if (!found)
            return false;

        col -= pos.start;
        dynamic_ragged2d::stableRemoval(v, col, bucketPositions, entries);
        return true;
    }

    // Removes the given entry from the non-idle bucket of the specified vertex.
    // Given only the vehicle id, we use linear search.
    bool removeNonIdle(const int v, const int vehId, int64_t &numEntriesVisited) {
        assert(v >= 0);
        assert(v < bucketPositions.size());

        const auto &pos = bucketPositions[v];
        int col = -1;
        const bool found = linearSearchForExistingEntry(vehId, col, pos.start + pos.numIdleEntries, pos.end, numEntriesVisited);
        if (!found)
            return false;

        col -= pos.start;
        dynamic_ragged2d::stableRemoval(v, col, bucketPositions, entries);
        return true;
    }

    // Batch update interface:

    struct EntryInsertion {
        int row = INVALID_INDEX;
        int col = INVALID_INDEX;
        BucketEntryT value = BucketEntryT();


        constexpr bool operator==(const EntryInsertion &) const noexcept = default;
    };

    struct EntryDeletion {
        int row = INVALID_INDEX;
        int col = INVALID_INDEX;

        constexpr bool operator==(const EntryDeletion &) const noexcept = default;
    };

    bool getIdleEntryInsertion(const int v, const BucketEntryT &entry, EntryInsertion &ins) const {
        KASSERT(v >= 0);
        KASSERT(v < bucketPositions.size());
        const auto start = bucketPositions[v].start;
        KASSERT(start + bucketPositions[v].numIdleEntries <= bucketPositions[v].end);
        // TODO: change searchForInsertionIdx to return index relative to bucket ?
        const auto col = searchForInsertionIdx(entry, start, start + bucketPositions[v].numIdleEntries, idleComp) - start;
        ins = {v, col, entry};
        return true;
    }

    bool getNonIdleEntryInsertion(const int v, const BucketEntryT &entry, EntryInsertion &ins) const {
        KASSERT(v >= 0);
        KASSERT(v < bucketPositions.size());
        const auto start = bucketPositions[v].start;
        const auto end = bucketPositions[v].end;
        KASSERT(start + bucketPositions[v].numIdleEntries <= end);
        // TODO: change searchForInsertionIdx to return index relative to bucket ?
        const auto col = searchForInsertionIdx(entry, start + bucketPositions[v].numIdleEntries, end, nonIdleComp) - start;
        ins = {v, col, entry};
        return true;
    }

    bool
    getIdleEntryDeletion(const int v, const BucketEntryT &entry, EntryDeletion &del) const {
        KASSERT(v >= 0);
        KASSERT(v < bucketPositions.size());
        const auto start = bucketPositions[v].start;
        KASSERT(start + bucketPositions[v].numIdleEntries <= bucketPositions[v].end);
        int64_t numEntriesVisited = 0;

        int col = -1;
        const bool found = binarySearchForExistingEntry(entry, col, start, start + bucketPositions[v].numIdleEntries, idleComp, numEntriesVisited);
        if (!found)
            return false;

        // TODO: change binarySearchForExistingEntry to return index relative to bucket ?
        col -= start;
        del = {v, col};
        return true;
    }

    bool getNonIdleEntryDeletion(const int v, const BucketEntryT &entry, EntryDeletion &del) const {
        KASSERT(v >= 0);
        KASSERT(v < bucketPositions.size());
        const auto start = bucketPositions[v].start;
        const auto end = bucketPositions[v].end;
        KASSERT(start + bucketPositions[v].numIdleEntries <= end);
        int64_t numEntriesVisited = 0;

        int col = -1;
        const bool found = binarySearchForExistingEntry(entry, col, start + bucketPositions[v].numIdleEntries, end, idleComp, numEntriesVisited);
        if (!found)
            return false;

        // TODO: change binarySearchForExistingEntry to return index relative to bucket ?
        col -= start;
        del = {v, col};
        return true;
    }

    bool getNonIdleEntryDeletion(const int v, const int vehId, EntryDeletion &del) const {
        KASSERT(v >= 0);
        KASSERT(v < bucketPositions.size());
        const auto start = bucketPositions[v].start;
        const auto end = bucketPositions[v].end;
        KASSERT(start + bucketPositions[v].numIdleEntries <= end);
        int64_t numEntriesVisited = 0;

        int col = -1;
        const bool found = linearSearchForExistingEntry(vehId, col, start + bucketPositions[v].numIdleEntries, end, numEntriesVisited);
        if (!found)
            return false;

        // TODO: change linearSearchForExistingEntry to return index relative to bucket ?
        col -= start;
        del = {v, col};
        return true;
    }

    template<typename EntryInsertionsVecT, typename EntryDeletionsVecT>
    void batchedCommitInsertionsAndDeletions(EntryInsertionsVecT &insertions,
                                             size_t numIdleInsertions,
//                                             EntryInsertionsVecT &nonIdleInsertions,
                                             EntryDeletionsVecT &deletions,
                                             size_t numIdleDeletions,
                                             karri::LastStopBucketUpdateStats& stats) {
//                                             EntryDeletionsVecT &nonIdleDeletions) {
        KASSERT(entries.size() >= deletions.size());

        // Sort idle insertions using idleComp. Expects that idle insertions are the first numIdleInsertions elements.
        // TODO: Actually only need to sort by idleComp/nonIdleComp if insertions point to the same row and column.
        Timer timer;
        std::sort(insertions.begin(), insertions.begin() + numIdleInsertions,
                  [&](const EntryInsertion &i1, const EntryInsertion &i2) {
                      return idleComp(i1.value, i2.value);
                  });
        stats.sortIdleInsertionsByCompTime = timer.elapsed<std::chrono::nanoseconds>();

        // Sort non-idle insertions using nonIdleComp.
        timer.restart();
        std::sort(insertions.begin() + numIdleInsertions, insertions.end(),
                  [&](const EntryInsertion &i1, const EntryInsertion &i2) {
                      return nonIdleComp(i1.value, i2.value);
                  });
        stats.sortNonIdleInsertionsByCompTime = timer.elapsed<std::chrono::nanoseconds>();

        timer.restart();

        // Update number of idle entries for each idle insertion.
        for (int i = 0; i < numIdleInsertions; ++i) {
            ++bucketPositions[insertions[i].row].numIdleEntries;
        }

        // Update number of idle entries for each idle deletion. Expects that idle deletions are the first numIdleDeletions elements.
        for (int i = 0; i < numIdleDeletions; ++i) {
            --bucketPositions[deletions[i].row].numIdleEntries;
        }
        stats.updateNumIdleEntriesTime = timer.elapsed<std::chrono::nanoseconds>();

        // Sort insertions stable by row and column, then
        static const auto sortInsertionsByRowAndCol = [&](const EntryInsertion &i1, const EntryInsertion &i2) {
            return i1.row < i2.row || (i1.row == i2.row && (i1.col < i2.col));
        };
        std::stable_sort(insertions.begin(), insertions.end(), sortInsertionsByRowAndCol);

        // Sort deletions by row and column
        static const auto sortEntryDeletions = [&](const auto &d1, const auto &d2) {
            return d1.row < d2.row || (d1.row == d2.row && d1.col < d2.col);
        };
        std::sort(deletions.begin(), deletions.end(), sortEntryDeletions);

        // Update column of deletion for any case where there are insertions and deletions in the same row.
        int numInsertionsInRowBefore = 0;
        int firstInsInRow = 0;
        for (int delIdx = 0; delIdx < deletions.size(); ++delIdx) {
            const auto delRow = deletions[delIdx].row;
            while (firstInsInRow < insertions.size() && insertions[firstInsInRow].row < delRow) {
                ++firstInsInRow;
            }
            numInsertionsInRowBefore = 0;
            while (firstInsInRow + numInsertionsInRowBefore < insertions.size() && insertions[firstInsInRow + numInsertionsInRowBefore].row == delRow &&
                   insertions[firstInsInRow + numInsertionsInRowBefore].col <= deletions[delIdx].col) {
                ++numInsertionsInRowBefore;
            }
            deletions[delIdx].col += numInsertionsInRowBefore;
        }

        // TODO: Remove if using new collective method in dynamic_ragged2d.
       // Update column of insertion for any case where there are multiple insertions to the same row.
        numInsertionsInRowBefore = 1;
        for (int i = 1; i < insertions.size(); ++i) {
            if (insertions[i].row == insertions[i - 1].row) {
                insertions[i].col += numInsertionsInRowBefore;
                ++numInsertionsInRowBefore;
            } else {
                numInsertionsInRowBefore = 1;
            }
        }


        // TODO: Should do deletions before insertions to increase chance of being able to grow rows without conflicts.
        // TODO: Parallelize insertions (see SortedBucketContainer).
        //  However, remove always shrinks at back of row, and new grow would always grow at front of row.
        //  This doesn't really make sense so maybe rework remove to shrink at front of row instead?

        // Perform insertions.
        for (const auto &ins: insertions) {
            dynamic_ragged2d::stableInsertion(ins.row, ins.col, ins.value, bucketPositions, entries);
        }

        if (deletions.empty())
            return;



        // Group deletions per row.
        std::vector<int> startOfDeletionsInRow;
        startOfDeletionsInRow.reserve(deletions.size());
        startOfDeletionsInRow.push_back(0);
        for (int i = 1; i < deletions.size(); ++i) {
            if (deletions[i].row != deletions[i - 1].row) {
                startOfDeletionsInRow.push_back(i);
            }
        }
        startOfDeletionsInRow.push_back(deletions.size());

        // Perform deletions in parallel. Deletions in the same row are performed by the same thread.
        tbb::parallel_for(0ul, startOfDeletionsInRow.size() - 1, [&](const size_t i) {
            const auto start = startOfDeletionsInRow[i];
            const auto end = startOfDeletionsInRow[i + 1];
            const auto row = deletions[start].row;
            std::vector<int> deletionCols(end - start);
            for (int j = start; j < end; ++j) {
                deletionCols[j - start] = deletions[j].col;
            }
            dynamic_ragged2d::stableRemovalOfSortedCols(row, deletionCols, bucketPositions, entries);
        });


        KASSERT(verifyAllBucketsSorted());
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
    int searchForInsertionIdx(const BucketEntryT &entry, const int start, const int end, CompT &comp) const {

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
    binarySearchForExistingEntry(const BucketEntryT &entry, int &idx, const int start, const int end, CompT &comp,
        int64_t &numEntriesVisited) const {
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


    bool linearSearchForExistingEntry(const int vehId, int &idx, const int start, const int end, int64_t &numEntriesVisited) const {
        for (idx = start; idx < end; ++idx) {
            ++numEntriesVisited;
            if (entries[idx].targetId == vehId)
                return true;
        }
        return false;
    }

    bool verifyAllBucketsSorted() {
        for (int v = 0; v < bucketPositions.size(); ++v) {
            const auto start = bucketPositions[v].start;
            const auto numIdle = bucketPositions[v].numIdleEntries;
            const auto end = bucketPositions[v].end;
            const bool idleSorted = std::is_sorted(entries.begin() + start, entries.begin() + start + numIdle,
                                               [&](const auto &e1, const auto &e2) { return idleComp(e1, e2); });
            KASSERT(idleSorted);
            if (!idleSorted)
                return false;
            const bool nonIdleSorted = std::is_sorted(entries.begin() + start + numIdle, entries.begin() + end,
                                                   [&](const auto &e1, const auto &e2) {
                                                       return nonIdleComp(e1, e2);
                                                   });
            KASSERT(nonIdleSorted);
            if (!nonIdleSorted)
                return false;
        }
        return true;
    }


    IdleComparatorT idleComp;
    NonIdleComparatorT nonIdleComp;

    std::vector<BucketPosition> bucketPositions;
    std::vector<BucketEntryT> entries;
};