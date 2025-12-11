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
#include <tbb/parallel_for.h>
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"


template<typename BucketEntryT, typename BucketEntryComparatorT>
class SortedBucketContainer {
public:
    using SortedBucket = ConstantVectorRange<BucketEntryT>;

    explicit SortedBucketContainer(const int numVertices)
        : comp(), bucketPositions(numVertices) {
    }

    size_t totalNumEntries() const {
        size_t numEntries = 0;
        for (const auto &pos: bucketPositions) {
            numEntries += pos.end - pos.start;
        }
        return numEntries;
    }

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
        dynamic_ragged2d::stableInsertion(v, pos, entry, bucketPositions, entries);
        return true;
    }

    // Applies the given transformation to all entries in the bucket at vertex.
    // The transformation must be callable with a single entry of type BucketEntryT& as an argument and must return
    // bool stating whether an update was performed.
    // Returns true if any entry was updated and false otherwise.
    // Only affects entries of vertex v. May update entries of v and change their order within the bucket but does
    // not affect anything else. Thus, this is safe to call in parallel over multiple vertices (as long as calls to
    // transform and comp are thread-safe).
    template<typename TransformationT>
    bool updateAllEntries(const int vertex, const TransformationT &transform, int64_t &numEntriesVisited) {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());

        const auto &pos = bucketPositions[vertex];
        assert(std::is_sorted(entries.begin() + pos.start, entries.begin() + pos.end,
            [&](const auto &e1, const auto &e2) { return comp(e1, e2); }));

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
    bool remove(const int v, const int targetId, int64_t &numEntriesVisited) {
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

        dynamic_ragged2d::stableRemoval(v, i - pos.start, bucketPositions, entries);
        return true;
    }

    // Removes the given entry from the bucket of the specified vertex.
    // Since buckets are sorted, binary search can be used.
    bool remove(const int v, const BucketEntryT &entry, int64_t &numEntriesVisited) {
        assert(v >= 0);
        assert(v < bucketPositions.size());
        numEntriesVisited = 0;

        int posInBucket = -1;
        const bool found = findPosOfExistingEntryBinary(v, entry, posInBucket);
        if (!found)
            return false;

        dynamic_ragged2d::stableRemoval(v, posInBucket, bucketPositions, entries);
        return true;
    }

    // Removes all entries from all buckets.
    void clear() {
        for (auto &bucketPos: bucketPositions)
            bucketPos.end = bucketPos.start;
        std::fill(entries.begin(), entries.end(), BucketEntryT());
    }

    // Batched interface:

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

    bool getEntryInsertion(const int v, const BucketEntryT &entry, EntryInsertion &ins) const {
        const auto pos = findInsertionPosForEntryInBucket(v, entry);
        ins = {v, pos, entry};
        return true;
    }

    bool getEntryDeletion(const int v, const int targetId, EntryDeletion &del, int64_t &numEntriesVisited) const {
        int pos = -1;
        const bool found = findPosOfExistingEntryLinear(v, targetId, pos, numEntriesVisited);
        if (!found)
            return false;

        del = {v, pos};
        return true;
    }

    bool
    getEntryDeletion(const int v, const BucketEntryT &entry, EntryDeletion &del, int64_t &numEntriesVisited) const {
        int pos = -1;
        const bool found = findPosOfExistingEntryBinary(v, entry, pos, numEntriesVisited);
        if (!found)
            return false;

        del = {v, pos};
        return true;
    }

    void batchedCommitInsertions(std::vector<EntryInsertion> &insertions) {

        if (insertions.empty())
            return;

        // Sort insertions by bucket-internal order given by comp.
        static const auto sortEntryInsertions = [&](const EntryInsertion &i1, const EntryInsertion &i2) {
            if (i1.row < i2.row)
                return true;
            if (i1.row > i2.row)
                return false;
            if (i1.col < i2.col)
                return true;
            if (i1.col > i2.col)
                return false;
            return comp(i1.value, i2.value);
        };
        std::sort(insertions.begin(), insertions.end(), sortEntryInsertions);

        // Now in dynamic_ragged2d method
        // Update insertion column for any case where there are multiple insertions to the same row:
        int numInsertionsInRowBefore = 1;
        for (int i = 1; i < insertions.size(); ++i) {
            if (insertions[i].row == insertions[i - 1].row) {
                insertions[i].col += numInsertionsInRowBefore;
                ++numInsertionsInRowBefore;
            } else {
                numInsertionsInRowBefore = 1;
            }
        }

        // // TODO: Parallelize insertions. Group insertions by rows, then in parallel for each row: Find out if row has
        // //  enough room to grow backward (to smaller indices) using new checkIfRangeCanGrowBackward() method in dynamic_ragged2d.
        // //  If so, perform all insertions in the row immediately with new collective method in dynamic_ragged2d.
        // //  If not, mark the row as not grown. After all rows processed, collect all not grown rows and perform insertions for them
        // //  sequentially using existing dynamic_ragged2d::stableInsertion method (which resizes the entire entries array if needed).
        // //  Can do one big resize for all not grown rows at once and compute their new ranges in entries, then copy in parallel.
        // //  -
        // //  However, remove always shrinks at back of row, and new grow would always grow at front of row.
        // //  This doesn't really make sense so maybe rework remove to shrink at front of row instead?
        //
        // std::vector<int> startOfInsertionsInRow;
        // startOfInsertionsInRow.reserve(insertions.size());
        // startOfInsertionsInRow.push_back(0);
        // for (int i = 1; i < insertions.size(); ++i) {
        //     if (insertions[i].row != insertions[i - 1].row) {
        //         startOfInsertionsInRow.push_back(i);
        //     }
        // }
        // startOfInsertionsInRow.push_back(insertions.size());
        // std::vector<int> newStartsOfAffectedRows(startOfInsertionsInRow.size() - 1);
        //
        // const auto oldEntriesSize = entries.size();
        //
        // // Find position of affected rows after insertions:
        // for (int i = 0; i < startOfInsertionsInRow.size() - 1; ++i) {
        //     const auto start = startOfInsertionsInRow[i];
        //     const auto end = startOfInsertionsInRow[i + 1];
        //     const auto row = insertions[start].row;
        //     const auto numInsertions = end - start;
        //     newStartsOfAffectedRows[i] = dynamic_ragged2d::getStartOfNewRangeForInsertionsGrowingBackwards(
        //         row, bucketPositions, entries, numInsertions);
        //
        //     if (newStartsOfAffectedRows[i] < oldEntriesSize) {
        //         KASSERT(newStartsOfAffectedRows[i] == dynamic_ragged2d::getStartOfNewRangeForInsertionsGrowingBackwards(
        //         row, bucketPositions, entries, end - start));
        //     }
        // }
        //
        // // Perform insertions in parallel. Insertions in the same row are performed by the same thread.
        // tbb::parallel_for(0ul, startOfInsertionsInRow.size() - 1, [&](const size_t i) {
        //     const auto start = startOfInsertionsInRow[i];
        //     const auto end = startOfInsertionsInRow[i + 1];
        //     const auto row = insertions[start].row;
        //     const auto startOfNewRange = newStartsOfAffectedRows[i];
        //
        //     if (startOfNewRange < oldEntriesSize) {
        //         KASSERT(startOfNewRange == dynamic_ragged2d::getStartOfNewRangeForInsertionsGrowingBackwards(
        //         row, bucketPositions, entries, end - start));
        //     }
        //
        //     // Prepare insertion columns and values.
        //     std::vector<int> insertionCols(end - start);
        //     std::vector<BucketEntryT> insertionVals(end - start);
        //     for (int j = start; j < end; ++j) {
        //         insertionCols[j - start] = insertions[j].col;
        //         insertionVals[j - start] = insertions[j].value;
        //     }
        //
        //     // Perform insertions collectively.
        //     dynamic_ragged2d::stableInsertionOfSortedColsGrowingBackwardWithKnownStartOfNewRange(
        //         row, insertionCols, insertionVals, startOfNewRange, bucketPositions, entries);
        // });


        // Perform insertions.
        for (const auto &ins: insertions) {
            dynamic_ragged2d::stableInsertion(ins.row, ins.col, ins.value, bucketPositions, entries);
        }

        KASSERT(dynamic_ragged2d::verifyConsistency(bucketPositions, entries));
        KASSERT(verifyAllBucketsSorted());
    }


    void batchedCommitDeletions(std::vector<EntryDeletion> &deletions) {
        KASSERT(entries.size() >= deletions.size());
        if (deletions.empty())
            return;

        // Sort deletions by row and get offset per row.
        static const auto sortEntryDeletions = [&](const auto &d1, const auto &d2) {
            return d1.row < d2.row || (d1.row == d2.row && d1.col < d2.col);
        };
        std::sort(deletions.begin(), deletions.end(), sortEntryDeletions);

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

private:
    int findInsertionPosForEntryInBucket(const int v, const BucketEntryT &entry) const {
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

    // Returns whether position was found. If so, position (relative to bucket) will be written to idx.
    // Uses linear search.
    bool findPosOfExistingEntryLinear(const int v, const int targetId, int &idx, int64_t &numEntriesVisited) const {
        KASSERT(v >= 0);
        assert(v < bucketPositions.size());

        const auto &pos = bucketPositions[v];
        const auto start = pos.start;
        const auto numEntries = pos.end - start;

        idx = 0;
        while (idx < numEntries) {
            ++numEntriesVisited;
            if (entries[start + idx].targetId == targetId) {
                break;
            }
            ++idx;
        }
        return idx < numEntries;
    }

    // Returns whether position was found. If so, position will be written to idx.
    bool findPosOfExistingEntryBinary(const int v, const BucketEntryT &entry, int &idx, int64_t &numEntriesVisited) {
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

    bool verifyAllBucketsSorted() {
        for (int v = 0; v < bucketPositions.size(); ++v) {
            const auto start = bucketPositions[v].start;
            const auto end = bucketPositions[v].end;
            const bool sorted = std::is_sorted(entries.begin() + start, entries.begin() + end,
                                               [&](const auto &e1, const auto &e2) { return comp(e1, e2); });
            KASSERT(sorted);
            if (!sorted)
                return false;
        }
        return true;
    }


    BucketEntryComparatorT comp;

    using BucketPosition = dynamic_ragged2d::ValueBlockPosition;

    // Bucket entries for vertex v are stored in entries between indices bucketPositions[v].start (inclusive)
    // and bucketPositions[v].end (exclusive).
    std::vector<BucketPosition> bucketPositions;
    std::vector<BucketEntryT> entries;
};
