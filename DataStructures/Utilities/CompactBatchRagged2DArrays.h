/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler
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
#include <cstdint>
#include <iterator>
#include <limits>
#include <vector>
#include <numeric>
#include <kassert/kassert.hpp>
#include "Tools/Constants.h"

// This file contains various functions for manipulating dynamic ragged two-dimensional arrays. In a
// ragged 2D array, different rows have different lengths. We store all values in a single dynamic
// value array. The rows are stored consecutively in memory. Within each row, the values are stored
// consecutively in memory, in the order in which they appear in the row.
//
// In order to maintain this flat representation, every insertion or deletion of an entry requires
// up to O(n) copy operations where n is the total number of values in the 2D-array.
// Thus, this data structure offers functionality for batched insertions and deletions. Given a batch
// of k insertions or deletions, only O(k + n) copy operations are necessary instead of O(k * n).
//
// The operations in this file expect a value array and an index array as input. The value array
// stores all values as described above. The index array stores the index of the first entry of
// every row in the value array. I.e. given value array V and index array I, the values of row r
// are stored in V[I[r] : I[r + 1]].
//
// Insertions and deletions are always stable, i.e. they maintain the order of existing elements.
// Stable deletions specify a row and an index of the value within the row that should be removed.
// Stable insertions specify a row and an index within the row where the value should be inserted.
// In a batch of stable insertions, multiple insertions can specify the same index to
// be inserted to. In this case, the values are inserted in the order that they were passed in,
// i.e. a batch of stable insertions is stable with respect to both the row and the input order.

namespace compact_batch_ragged2d {

    template<typename T>
    struct Insertion {
        int row = INVALID_INDEX;
        int col = INVALID_INDEX;
        T value = T();


        constexpr bool operator==(const Insertion&) const noexcept = default;
    };

    // Returns the indices in the value arrays of the newly inserted values.
    // TODO: ExtraValueArrays?
    template<typename T, typename IndexArray, typename ValueArray>
    std::vector<int> stableBatchedInsertions(std::vector<Insertion<T>> &insertions, IndexArray &indexArray, ValueArray &valueArray) {

        if (insertions.empty())
            return {};

        // Order insertions by row and column. Sort stable to maintain input order of insertions with same row and column.
        static const auto sortInsertions = [&](const Insertion<T> &i1, const Insertion<T> &i2) {
            return i1.row < i2.row || (i1.row == i2.row && (i1.col < i2.col));
        };
        std::stable_sort(insertions.begin(), insertions.end(), sortInsertions);

        // Accumulate changes at indices and number of entries per bucket.
        std::vector<int> entryIdxChange(valueArray.size() + 1, 0);
        std::vector<int> rowSizeChange(indexArray.size(), 0);
        for (const auto &ins: insertions) {
            ++rowSizeChange[ins.row];
            ++entryIdxChange[indexArray[ins.row] + ins.col];
        }

        // TODO: Using prefix sums here is better for future parallel approach but sequential approach could do this
        //  without need for additional vector bucketSizeChange (and possibly without entryIdxChange, too).

        // Prefix sum over changes at indices gives index offset of existing entries after updates.
        std::inclusive_scan(entryIdxChange.begin(), entryIdxChange.end(), entryIdxChange.begin());

        // Prefix sum over changes to row sizes gives changes to row start offsets after updates.
        std::exclusive_scan(rowSizeChange.begin(), rowSizeChange.end(), rowSizeChange.begin(), 0);

        // For each insertion, compute index in value array, resolving multiple insertions to same index.
        std::vector<int> insertIndices(insertions.size());
        for (int i = 0; i < insertions.size(); ++i)
            insertIndices[i] = indexArray[insertions[i].row] + insertions[i].col + i;

        const auto newNumEntries = valueArray.size() + insertions.size();
        valueArray.resize(newNumEntries);

        // Move existing entries to new indices from back to front so we do not overwrite values.
        for (int i = static_cast<int>(entryIdxChange.size()) - 2; i >= 0; --i)
            valueArray[i + entryIdxChange[i]] = valueArray[i];

        // Write new entries.
        for (int i = 0; i < insertions.size(); ++i) {
            KASSERT(insertIndices[i] < valueArray.size());
            valueArray[insertIndices[i]] = insertions[i].value;
        }

        // Update row start offsets.
        for (int j = 0; j < indexArray.size(); ++j)
            indexArray[j] += rowSizeChange[j];

        return insertIndices;
    }


    struct Deletion {
            int row = INVALID_INDEX;
            int col = INVALID_INDEX;

            constexpr bool operator==(const Deletion&) const noexcept = default;
        };


    // TODO: ExtraValueArrays?
    template<typename IndexArray, typename ValueArray>
    void stableBatchedDeletions(std::vector<Deletion> &deletions, IndexArray &indexArray, ValueArray &valueArray) {
        KASSERT(valueArray.size() >= deletions.size());

        if (deletions.empty())
            return;

        // Order deletions by row and col.
        static const auto sortEntryDeletions = [&](const auto &d1, const auto &d2) {
            return d1.row < d2.row || (d1.row == d2.row && d1.col < d2.col);
        };
        std::sort(deletions.begin(), deletions.end(), sortEntryDeletions);

        // Accumulate changes at indices and number of entries per bucket.
        std::vector<int> entryIdxChange(valueArray.size() + 1, 0);
        std::vector<int> rowSizeChange(indexArray.size(), 0);
        for (const auto &del: deletions) {
            --rowSizeChange[del.row];
            --entryIdxChange[indexArray[del.row] + del.col + 1];
        }

        // TODO: Using prefix sums here is better for future parallel approach but sequential approach could do this
        //  without need for additional vectors entryIdxChange and bucketSizeChange.

        // Prefix sum over changes at indices gives index offset of existing entries after updates.
        std::inclusive_scan(entryIdxChange.begin(), entryIdxChange.end(), entryIdxChange.begin());

        // Prefix sum over changes to row sizes gives changes to row start offsets after updates.
        std::exclusive_scan(rowSizeChange.begin(), rowSizeChange.end(), rowSizeChange.begin(), 0);

        const auto newNumEntries = valueArray.size() - deletions.size();

        // Move existing entries to new indices from front to back so we do not overwrite values.
        // Deleted entries will be moved but then overwritten.
        for (int i = 0; i < entryIdxChange.size() - 1; ++i)
            valueArray[i + entryIdxChange[i]] = valueArray[i];

        valueArray.resize(newNumEntries);

        // Update bucket start offsets.
        for (int j = 0; j < indexArray.size(); ++j)
            indexArray[j] += rowSizeChange[j];
    }



    // Returns the indices in the value array at which the new values were inserted.
    // TODO: ExtraValueArrays?
    template<typename T, typename IndexArray, typename ValueArray>
    std::vector<int> stableBatchedInsertionsAndDeletions(std::vector<Insertion<T>> &insertions,
                                             std::vector<Deletion> &deletions,
                                             IndexArray& indexArray, ValueArray& valueArray) {
        KASSERT(valueArray.size() >= deletions.size());

        // If either insertions or deletions are empty, we can handle the other one directly.
        if (insertions.empty() && deletions.empty())
            return {};
        if (insertions.empty()) {
            stableBatchedDeletions(deletions, indexArray, valueArray);
            return {};
        }
        if (deletions.empty()) {
            return stableBatchedInsertions(insertions, indexArray, valueArray);
        }

        // Order insertions by row and column. Sort stable to maintain input order of insertions with same row and column.
        static const auto sortInsertions = [&](const Insertion<T> &i1, const Insertion<T> &i2) {
            return i1.row < i2.row || (i1.row == i2.row && (i1.col < i2.col));
        };
        std::stable_sort(insertions.begin(), insertions.end(), sortInsertions);

        // Order deletions by row and col.
        static const auto sortEntryDeletions = [&](const auto &d1, const auto &d2) {
            return d1.row < d2.row || (d1.row == d2.row && d1.col < d2.col);
        };
        std::sort(deletions.begin(), deletions.end(), sortEntryDeletions);

        // Accumulate changes at indices and number of entries per bucket.
        std::vector<int> entryIdxChange(valueArray.size() + 1, 0);
        std::vector<int> rowSizeChange(indexArray.size(), 0);
        for (const auto &ins: insertions) {
            ++rowSizeChange[ins.row];
            ++entryIdxChange[indexArray[ins.row] + ins.col];
        }
        for (const auto &del: deletions) {
            --rowSizeChange[del.row];
            --entryIdxChange[indexArray[del.row] + del.col + 1];
        }

        // Prefix sum over changes at indices gives index offset of existing entries after updates.
        std::inclusive_scan(entryIdxChange.begin(), entryIdxChange.end(), entryIdxChange.begin());

        // Prefix sum over changes to row sizes gives changes to row start offsets after updates.
        std::exclusive_scan(rowSizeChange.begin(), rowSizeChange.end(), rowSizeChange.begin(), 0);

        // Prefix sum over deletions and insertions gives indices of new entries.
        int insIdx = 0, delIdx = 0;
        int delta = 0;
        std::vector<int> insertIndices(insertions.size());
        while (insIdx < insertions.size()) {
            if (delIdx == deletions.size() || indexArray[insertions[insIdx].row] + insertions[insIdx].col <= indexArray[deletions[delIdx].row] + deletions[delIdx].col) {
                insertIndices[insIdx] = indexArray[insertions[insIdx].row] + insertions[insIdx].col + delta;
                ++delta; // Register one insertion for offset of new entries
                ++insIdx;
                continue;
            }
            KASSERT(delIdx < deletions.size() && indexArray[deletions[delIdx].row] + deletions[delIdx].col < indexArray[insertions[insIdx].row] + insertions[insIdx].col);
            --delta; // Register one deletion for offset of new entries
            ++delIdx;
        }

        // Mark deleted entries by overwriting with invalid value (only required for assertions right now).
        for (const auto &del: deletions) {
            const auto indexInEntries = indexArray[del.row] + del.col;
            valueArray[indexInEntries] = T();
        }

        const auto newNumEntries = valueArray.size() + insertions.size() - deletions.size();
        ValueArray newValueArray(newNumEntries + 1);

        // Move existing entries to new entries vector. Deleted entries will be moved and later overwritten.
        // TODO: for parallelization, make sure not to move non-deleted entry first and then overwrite with deleted
        //  entry later by different thread.
        for (int i = 0; i < entryIdxChange.size() - 1; ++i) {
            KASSERT(i + entryIdxChange[i] < newValueArray.size());
            KASSERT(newValueArray[i + entryIdxChange[i]] == T());
            newValueArray[i + entryIdxChange[i]] = valueArray[i];
        }

        // Write new entries to new entries vector.
        for (int i = 0; i < insertions.size(); ++i) {
            KASSERT(insertIndices[i] < newValueArray.size());
            KASSERT(newValueArray[insertIndices[i]] == T());
            newValueArray[insertIndices[i]] = insertions[i].value;
        }

        KASSERT(newValueArray.back() == T());
        newValueArray.pop_back();
        KASSERT(std::all_of(newValueArray.begin(), newValueArray.end(), [&](const auto &e) { return e != T(); }));
        valueArray.swap(newValueArray);

        // Update bucket start offsets.
        for (int j = 0; j < indexArray.size(); ++j)
            indexArray[j] += rowSizeChange[j];

        return insertIndices;
    }

} // namespace dynamic_ragged2d