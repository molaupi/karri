/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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

// This class maintains a dynamic bucket for each vertex for bucket-based CH searches. We store all
// bucket entries in a single dynamic value array. The entries in the same bucket are stored
// consecutively in memory, in no particular order. In addition, an index array stores the starting
// and ending point of each bucket's value block in the value array.
//
// When we remove an entry from a bucket, we fill the resulting hole in the value array with the
// rightmost value in the bucket's value block, and decrement the block's ending point in the index
// array. Consider an insertion of an entry into a bucket. If the element immediately after the
// bucket's value block is a hole, the new entry fills this hole. Analogously, if the element before
// the value block is a hole, the new entry fills that hole. Otherwise, we move the entire value
// block to the end of the value array, and additionally insert a number of holes after the value
// block (the number is a constant fraction of the block size). Then, there is a hole after the
// block, and we proceed as described above.
template<typename BucketEntryT>
class DynamicBucketContainer {
public:
    using Entry = BucketEntryT;
    using Bucket = ConstantVectorRange<BucketEntryT>;

    // Constructs a container that can maintain buckets for the specified number of vertices.
    explicit DynamicBucketContainer(const int numVertices) : bucketPositions(numVertices) {
        assert(numVertices >= 0);
    }

    // Returns the bucket of the specified vertex.
    Bucket getBucketOf(const int vertex) const {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());
        const auto &pos = bucketPositions[vertex];
        return Bucket(entries.begin() + pos.start, entries.begin() + pos.end);
    }

    Bucket getUnsortedBucketOf(const int vertex) const {
        return getBucketOf(vertex);
    }

    // Inserts the given entry into the bucket of the specified vertex.
    bool insert(const int vertex, const BucketEntryT &entry) {
        dynamic_ragged2d::insertion(vertex, entry, bucketPositions, entries);
        return true;
    }

    // Inserts the given entry into the bucket of the specified vertex at the given index (within the bucket).
    bool stableInsert(const int vertex, const int idx, const BucketEntryT &entry) {
        dynamic_ragged2d::stableInsertion(vertex, idx, entry, bucketPositions, entries);
        return true;
    }

    // Finds an entry for targetId in the bucket of the specified vertex and applies the given transformation to it.
    // The transformation must be callable with a single entry of type BucketEntryT& as an argument.
    // Returns true if an entry is found and replaced and false if no entry with targetId can be found.
    template<typename TransformationT>
    bool update(const int vertex, const int targetId, const TransformationT &transform, int64_t &numEntriesVisited) {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());
        const auto &pos = bucketPositions[vertex];
        for (auto i = pos.start; i < pos.end; ++i) {
            ++numEntriesVisited;
            if (entries[i].targetId == targetId) {
                transform(entries[i]);
                return true;
            }
        }
        return false;
    }

    // Removes the entry for targetId from the bucket of the specified vertex.
    bool remove(const int vertex, const int targetId, int64_t& numEntriesVisited) {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());
        const auto &pos = bucketPositions[vertex];
        for (auto i = pos.start; i < pos.end; ++i) {
            ++numEntriesVisited;
            if (entries[i].targetId == targetId) {
                dynamic_ragged2d::removal(vertex, i - pos.start, bucketPositions, entries);
                return true;
            }
        }
        return false;
    }

    // Removes the given entry from the bucket of the specified vertex.
    bool remove(const int vertex, const BucketEntryT &entry, int64_t &numEntriesVisited) {
        return remove(vertex, entry.targetId, numEntriesVisited);
    }


    // Removes multiple entries at given indices in the bucket. Indices have to be sorted in ascending order.
    template<typename IndicesRangeT>
    void removeSortedIndices(const int vertex, const IndicesRangeT &indicesRange) {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());
        dynamic_ragged2d::removalOfSortedCols(vertex, indicesRange, bucketPositions, entries);
        //        numEntriesVisited = indicesRange.size();
    }

    // Removes multiple entries at given indices in the bucket while keeping the order of remaining elements.
    // Indices have to be sorted in ascending order.
    template<typename IndicesRangeT>
    void stableRemoveSortedIndices(const int vertex, const IndicesRangeT &indicesRange) {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());
        dynamic_ragged2d::stableRemovalOfSortedCols(vertex, indicesRange, bucketPositions, entries);
        //        numEntriesVisited = indicesRange.size();
    }

    void clearBucket(const int vertex) {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());
        auto &bucketPos = bucketPositions[vertex];
        std::fill(entries.begin() + bucketPos.start, entries.begin() + bucketPos.end, BucketEntryT());
        bucketPos.end = bucketPos.start;
    }

    // Removes all entries from all buckets.
    void clear() {
        for (auto &bucketPos: bucketPositions)
            bucketPos.end = bucketPos.start;
        std::fill(entries.begin(), entries.end(), BucketEntryT());
    }

    bool allEmpty() const {
        const bool posEmpty = std::all_of(bucketPositions.begin(), bucketPositions.end(),
                                          [](const BucketPosition &bucketPos) {
                                              return bucketPos.end == bucketPos.start;
                                          });
        if (!posEmpty) return false;

        const auto hole = BucketEntryT();
        return std::all_of(entries.begin(), entries.end(), [hole](const BucketEntryT &entry) { return entry == hole; });
    }

    size_t totalNumEntries() const {
        size_t numEntries = 0;
        for (const auto &pos: bucketPositions) {
            numEntries += pos.end - pos.start;
        }
        return numEntries;
    }


    // Batch update interface:

    struct EntryInsertion {
        int row = INVALID_INDEX;
        BucketEntryT value = BucketEntryT();

        constexpr bool operator==(const EntryInsertion &) const noexcept = default;
    };

    struct EntryDeletion {
        int row = INVALID_INDEX;
        int col = INVALID_INDEX;

        constexpr bool operator==(const EntryDeletion &) const noexcept = default;
    };

    bool getEntryInsertion(const int v, const BucketEntryT &entry, EntryInsertion &ins) const {
        KASSERT(v >= 0);
        KASSERT(v < bucketPositions.size());
        ins = {v, entry};
        return true;
    }

    bool getEntryDeletion(const int v, const int targetId, EntryDeletion &del, int64_t &pNumEntriesVisited) const {
        KASSERT(v >= 0);
        KASSERT(v < bucketPositions.size());
        const auto start = bucketPositions[v].start;
        const auto &pos = bucketPositions[v];
        for (auto i = pos.start; i < pos.end; ++i) {
            ++pNumEntriesVisited;
            if (entries[i].targetId == targetId) {
                del = {v, i - start};
                return true;
            }
        }
        return false;
    }

    bool
    getEntryDeletion(const int v, const BucketEntryT &entry, EntryDeletion &del, int64_t &pNumEntriesVisited) const {
        return getEntryDeletion(v, entry.targetId, del, pNumEntriesVisited);
    }

    template<typename EntryInsertionsVecT, typename EntryDeletionsVecT>
    void batchedCommitInsertionsAndDeletions(EntryInsertionsVecT &insertions,
                                             EntryDeletionsVecT &deletions) {
        batchedCommitInsertions(insertions);
        batchedCommitDeletions(deletions);
    }

    template<typename EntryInsertionsVecT>
    void batchedCommitInsertions(EntryInsertionsVecT &insertions) {
        for (const auto &ins: insertions) {
            dynamic_ragged2d::insertion(ins.row, ins.value, bucketPositions, entries);
        }
    }

    template<typename EntryDeletionsVecT>
    void batchedCommitDeletions(EntryDeletionsVecT &deletions) {
        KASSERT(entries.size() >= deletions.size());
        if (deletions.empty())
            return;

        // Sort deletions by row and column.
        std::sort(deletions.begin(), deletions.end(), [&](const auto &d1, const auto &d2) {
            return d1.row < d2.row || (d1.row == d2.row && d1.col < d2.col);
        });

        // Iterate over deletions. Whenever next deletion is for a different row, perform deletions for current row.
        int currentRow = deletions[0].row;
        std::vector<int> colsToDelete;
        for (const auto &del: deletions) {
            if (del.row != currentRow) {
                dynamic_ragged2d::removalOfSortedCols(currentRow, colsToDelete, bucketPositions, entries);
                colsToDelete.clear();
                currentRow = del.row;
            }
            colsToDelete.push_back(del.col);
        }
        dynamic_ragged2d::removalOfSortedCols(currentRow, colsToDelete, bucketPositions, entries);
    }

private:
    using BucketPosition = dynamic_ragged2d::ValueBlockPosition;

    std::vector<BucketPosition> bucketPositions;
    std::vector<BucketEntryT> entries;
};
