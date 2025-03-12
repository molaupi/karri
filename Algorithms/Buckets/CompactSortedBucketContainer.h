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
#include <kassert/kassert.hpp>
#include "DataStructures/Utilities/Permutation.h"
#include "DataStructures/Utilities/IteratorRange.h"
#include "DataStructures/Utilities/CompactBatchRagged2DArrays.h"
#include "Parallel/scalable_vector.h"


template<typename BucketEntryT, typename BucketEntryComparatorT>
class CompactSortedBucketContainer {

public:

    template<typename T>
    using EntryVectorT = parallel::scalable_aligned_vector<T>;
    using SortedBucket = ConstantVectorRange<BucketEntryT, EntryVectorT>;

    using EntryInsertion = compact_batch_ragged2d::Insertion<BucketEntryT>;
    using EntryDeletion = compact_batch_ragged2d::Deletion;

//    struct EntryInsertion {
//        int vertex = INVALID_VERTEX;
//        int indexInEntries = INVALID_INDEX;
//        BucketEntryT entry = {};
//    };
//
//    struct EntryDeletion {
//        int vertex = INVALID_VERTEX;
//        int indexInEntries = INVALID_INDEX;
//    };


    explicit CompactSortedBucketContainer(const int numVertices)
            : comp(),
              offset(numVertices + 1, 0),
              entries() {}

    size_t totalNumEntries() const {
        return entries.size();
    }

    // Returns the bucket of the specified vertex, sorted in ascending order according to BucketEntryComparatorT.
    SortedBucket getBucketOf(const int v) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);
        const auto start = offset[v];
        const auto end = offset[v + 1];
        return SortedBucket(entries.begin() + start, entries.begin() + end);
    }

//    // Inserts the given entry into the bucket of the specified vertex.
//    bool insert(const int v, const BucketEntryT &entry) {
//        const auto pos = findInsertionPosForEntry(v, entry);
//
//        // Insert with linear time
//        entries.insert(entries.begin() + pos, entry);
//        for (int i = v + 1; i < offset.size(); ++i)
//            ++offset[i];
//
//        return true;
//    }

    bool getEntryInsertion(const int v, const BucketEntryT &entry, EntryInsertion &ins) const {
        const auto pos = findInsertionPosForEntry(v, entry);
        ins = {v, pos, entry};
        return true;
    }

    // Applies the given transformation to all entries in the bucket at vertex.
    // The transformation must be callable with a single entry of type BucketEntryT& as an argument and must return
    // bool stating whether an update was performed.
    // Returns true if any entry was updated and false otherwise.
    template<typename TransformationT>
    bool updateAllEntries(const int v, const TransformationT &transform, int64_t &numEntriesVisited) {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);

        const auto start = offset[v];
        const auto end = offset[v + 1];

        KASSERT(std::is_sorted(entries.begin() + start, entries.begin() + end,
                               [&](const auto &e1, const auto &e2) { return comp(e1, e2); }));

        bool anyUpdated = false;
        for (auto i = start; i < end; ++i) {
            ++numEntriesVisited;
            const bool transformed = transform(entries[i]);
            anyUpdated |= transformed;
        }
        if (!anyUpdated)
            return false;

        // If any entries were updated, fix sorting of bucket:
        std::sort(entries.begin() + start, entries.begin() + end,
                  [&](const auto &e1, const auto &e2) { return comp(e1, e2); });
        return true;
    }

//    // Removes the entry for targetId from the bucket of the specified vertex.
//    // Linear search for the targetId in the bucket.
//    bool remove(const int v, const int targetId, int64_t &numEntriesVisited) {
//        KASSERT(v >= 0);
//        KASSERT(v < offset.size() - 1);
//
//        int pos = -1;
//        const bool found = findPosOfExistingEntryLinear(v, targetId, pos, numEntriesVisited);
//        if (!found)
//            return false;
//
//        // Remove with linear time
//        entries.erase(entries.begin() + pos);
//        for (int j = v + 1; j < offset.size(); ++j)
//            --offset[j];
//
//        return true;
//    }

    bool getEntryDeletion(const int v, const int targetId, EntryDeletion &del, int64_t &numEntriesVisited) const {
        int pos = -1;
        const bool found = findPosOfExistingEntryLinear(v, targetId, pos, numEntriesVisited);
        if (!found)
            return false;

        del = {v, pos};
        return true;
    }

//    // Removes the given entry from the bucket of the specified vertex.
//    // Since buckets are sorted, binary search can be used.
//    bool remove(const int v, const BucketEntryT &entry, int64_t &numEntriesVisited) {
//        KASSERT(v >= 0);
//        KASSERT(v < offset.size() - 1);
//
//        int pos = -1;
//        const bool found = findPosOfExistingEntryBinary(v, entry, pos, numEntriesVisited);
//        if (!found)
//            return false;
//
//        // Remove with linear time
//        entries.erase(entries.begin() + pos);
//        for (int j = v + 1; j < offset.size(); ++j)
//            --offset[j];
//
//        return true;
//    }

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

        // Sort insertions by bucket-internal order given by comp.
        static const auto sortEntryInsertions = [&](const EntryInsertion &i1, const EntryInsertion &i2) {
            return comp(i1.value, i2.value);
        };
        std::sort(insertions.begin(), insertions.end(), sortEntryInsertions);

        // Perform insertions. Respects previous sorting by comp.
        compact_batch_ragged2d::stableBatchedInsertions(insertions, offset, entries);

         KASSERT(std::all_of(entries.begin(), entries.end(), [&](const auto &e) { return e != BucketEntryT(); }));
         KASSERT(verifyAllBucketsSorted());
    }


    void batchedCommitDeletions(std::vector<EntryDeletion> &deletions) {
        KASSERT(entries.size() >= deletions.size());

        compact_batch_ragged2d::stableBatchedDeletions(deletions, offset, entries);
        KASSERT(verifyAllBucketsSorted());
    }

//
//    void batchedCommitInsertionsAndDeletions(std::vector<EntryInsertion> &insertions,
//                                             std::vector<EntryDeletion> &deletions) {
//        KASSERT(entries.size() >= deletions.size());
//
//        // Order insertions, deletions by index in entries affected.
//        // Secondary order for insertions: bucket-internal comparator comp.
//        static const auto sortEntryInsertions = [&](const EntryInsertion &i1, const EntryInsertion &i2) {
//            return i1.indexInEntries < i2.indexInEntries ||
//                   (i1.indexInEntries == i2.indexInEntries &&
//                    (i1.vertex < i2.vertex || (i1.vertex == i2.vertex && comp(i1.entry, i2.entry))));
//        };
//        std::sort(insertions.begin(), insertions.end(), sortEntryInsertions);
//        static const auto sortEntryDeletions = [&](const auto &d1, const auto &d2) {
//            return d1.indexInEntries < d2.indexInEntries;
//        };
//        std::sort(deletions.begin(), deletions.end(), sortEntryDeletions);
//        KASSERT(noDuplicateDeletions(deletions));
//
//        // Accumulate changes at indices and number of entries per bucket.
//        std::vector<int> entryIdxChange(entries.size() + 1, 0);
//        std::vector<int> bucketSizeChange(offset.size(), 0);
//        for (const auto &ins: insertions) {
//            ++bucketSizeChange[ins.vertex];
//            ++entryIdxChange[ins.indexInEntries];
//        }
//        for (const auto &del: deletions) {
//            --bucketSizeChange[del.vertex];
//            --entryIdxChange[del.indexInEntries + 1];
//        }
//
//        // Prefix sum over changes at indices gives index offset of existing entries after updates.
//        std::inclusive_scan(entryIdxChange.begin(), entryIdxChange.end(), entryIdxChange.begin());
//
//        // Prefix sum over changes to bucket sizes gives changes to bucket start offsets after updates.
//        std::exclusive_scan(bucketSizeChange.begin(), bucketSizeChange.end(), bucketSizeChange.begin(), 0);
//
//        // Prefix sum over deletions and insertions gives indices of new entries.
//        // Store in indexInEntries field of insertion objects.
//        int insIdx = 0, delIdx = 0;
//        int delta = 0;
//        while (insIdx < insertions.size()) {
//            if (delIdx == deletions.size() || insertions[insIdx].indexInEntries <= deletions[delIdx].indexInEntries) {
//                insertions[insIdx].indexInEntries += delta; // Update insertion index for new entry
//                ++delta; // Register one insertion for offset of new entries
//                ++insIdx;
//                continue;
//            }
//            KASSERT(delIdx < deletions.size() && deletions[delIdx].indexInEntries < insertions[insIdx].indexInEntries);
//            --delta; // Register one deletion for offset of new entries
//            ++delIdx;
//        }
//
//
//
//        // Mark deleted entries by overwriting with invalid entry (only required for assertions right now).
//        for (const auto &del: deletions) {
//            entries[del.indexInEntries] = BucketEntryT();
//        }
//
//        const auto newNumEntries = entries.size() + insertions.size() - deletions.size();
//        std::vector<BucketEntryT> newEntries(newNumEntries + 1);
//
//        // Move existing entries to new entries vector. Deleted entries will be moved and later overwritten.
//        // TODO: for parallelization, make sure not to move non-deleted entry first and then overwrite with deleted
//        //  entry later by different thread.
//        for (int i = 0; i < entryIdxChange.size() - 1; ++i) {
//            KASSERT(i + entryIdxChange[i] < newEntries.size());
//            KASSERT(newEntries[i + entryIdxChange[i]] == BucketEntryT());
//            newEntries[i + entryIdxChange[i]] = entries[i];
//        }
//
//        // Write new entries to new entries vector.
//        for (const auto &ins: insertions) {
//            KASSERT(ins.indexInEntries < newEntries.size());
//            KASSERT(newEntries[ins.indexInEntries] == BucketEntryT());
//            newEntries[ins.indexInEntries] = ins.entry;
//        }
//
//        KASSERT(newEntries.back() == BucketEntryT());
//        newEntries.pop_back();
//        KASSERT(std::all_of(newEntries.begin(), newEntries.end(), [&](const auto &e) { return e != BucketEntryT(); }));
//        entries.swap(newEntries);
//
//        // Update bucket start offsets.
//        for (int j = 0; j < offset.size(); ++j)
//            offset[j] += bucketSizeChange[j];
//
//    }

    // Removes all entries from all buckets.
    void clear() {
        for (auto &i: offset)
            i = 0;
        entries.clear();
    }

private:

    static bool noDuplicateDeletions(const std::vector<EntryDeletion> &deletions) {
        if (deletions.empty())
            return true;
        for (auto it = deletions.begin(); it < deletions.end() - 1; ++it)
            if (it->row == (it + 1)->row && it->col == (it + 1)->col)
                return false;
        return true;
    }


    // Returns position (relative to bucket) to which entry should be inserted.
    int findInsertionPosForEntry(const int v, const BucketEntryT &entry) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);

        const auto start = offset[v];
        const auto numEntries = offset[v + 1] - start;

        // Check if bucket is currently empty or new entry needs to become first element:
        if (numEntries == 0 || comp(entry, entries[start]))
            return 0;

        // Check if new entry needs to become last element:
        if (!comp(entry, entries[start + numEntries - 1]))
            return numEntries;

        // Binary search with invariant: !comp(entry, entries[start + l]) && comp(entry, entries[start + r])
        int l = 0, r = numEntries - 1;
        while (l < r - 1) {
            KASSERT(!comp(entry, entries[start + l]) && comp(entry, entries[start + r]));
            int m = (l + r) / 2;
            if (comp(entry, entries[start + m])) {
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
        KASSERT(v < offset.size() - 1);

        const auto start = offset[v];
        const auto numEntries = offset[v + 1] - start;

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

    // Returns whether entry was found. If so, position (relative to bucket) will be written to idx.
    // Uses binary search.
    bool
    findPosOfExistingEntryBinary(const int v, const BucketEntryT &entry, int &idx, int64_t &numEntriesVisited) const {
        KASSERT(v >= 0);
        KASSERT(v < offset.size() - 1);

        const auto start = offset[v];
        const auto numEntries = offset[v + 1] - start;

        // Check if bucket is currently empty or is smaller than smallest entry in bucket or larger
        // than largest entry in bucket.
        numEntriesVisited += 2;
        if (numEntries == 0 || comp(entry, entries[start]) || comp(entries[start + numEntries - 1], entry))
            return false;

        // Check if entry is equal to last entry wrt to comp. If so, scan all entries that are equal backwards linearly.
        if (equiv(entry, entries[start + numEntries - 1])) {
            idx = numEntries - 1;
            while (idx >= 0 && equiv(entry, entries[start + idx]) &&
                   entry.targetId != entries[start + idx].targetId) {
                ++numEntriesVisited;
                --idx;
            }
            return idx >= 0 && entry.targetId == entries[start + idx].targetId;
        }

        // Binary search with invariant: !comp(entry, entries[start + l]) && comp(entry, entries[start + r])
        int l = 0, r = numEntries - 1;
        while (l < r - 1) {
            KASSERT(!comp(entry, entries[start + l]) && comp(entry, entries[start + r]));
            int m = (l + r) / 2;
            ++numEntriesVisited;
            if (comp(entry, entries[start + m])) {
                r = m;
            } else {
                l = m;
            }
        }

        // r is position right of range of entries that are equivalent to entry wrt comp (range may be empty if entry
        // does not exist). Scan through this range backwards linearly:
        idx = r - 1;
        while (idx >= 0 && equiv(entry, entries[start + idx]) &&
               entry.targetId != entries[start + idx].targetId) {
            ++numEntriesVisited;
            --idx;
        }
        return idx >= 0 && entry.targetId == entries[start + idx].targetId;
    }


    bool verifyAllBucketsSorted() {
        for (int v = 0; v < offset.size() - 1; ++v) {
            const auto start = offset[v];
            const auto end = offset[v + 1];
            const bool sorted = std::is_sorted(entries.begin() + start, entries.begin() + end,
                                               [&](const auto &e1, const auto &e2) { return comp(e1, e2); });
            KASSERT(sorted);
            if (!sorted)
                return false;
        }
        return true;
    }

    BucketEntryComparatorT comp;

    // Buckets are stored without gaps between (as opposed to SortedBucketContainer).
    // Bucket of vertex v is stored at entries[ offset[v] .. offset[v + 1] ].
    // Each bucket is sorted internally according to comp.
    using Offset = uint32_t;
    std::vector<Offset> offset;
    EntryVectorT<BucketEntryT> entries;

};