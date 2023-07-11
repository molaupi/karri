/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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
#include <cstdint>
#include <iterator>
#include <limits>
#include <vector>
#include "Tools/Constants.h"
#include "IteratorRange.h"

// This file contains various functions for manipulating dynamic ragged two-dimensional arrays. In a
// ragged 2D array, different rows have different lengths. We store all values in a single dynamic
// value array. The values in the same row are stored consecutively in memory, in the order in which
// they appear in the row. In addition, an index array stores the starting and ending point of each
// row's value block in the value array. Note that multiple ragged arrays (value arrays) can share a
// single index array.
//
// The insertion or removal of a value can be stable or not. An insertion or removal that is not
// stable does not keep the relative order within rows. When we remove a value from a row, we fill
// the resulting hole in the value array with the rightmost value in the row's value block, and
// decrement the block's ending point in the index array. Consider an insertion of a value into a
// row. If the element immediately after the row's value block is a hole, the new value fills this
// hole. Analogously, if the element before the value block is a hole, the new value fills that
// hole. Otherwise, we move the entire value block to the end of the value array, and additionally
// insert a number of holes after the value block (the number is a constant fraction of the block
// size). Then, there is a hole after the block, and we proceed as described above.
//
// In contrast, a stable insertion or removal keeps the relative order within rows. When we remove a
// value from a row, we move the resulting hole in the value array to the end of the row's value
// block, and decrement the block's ending point in the index array. Consider an insertion of a
// value into a row. If the element immediately after the row's value block is a hole, we insert the
// new value into the value block and move the values after the insertion point one position to the
// right. Analogously, if the element before the value block is a hole, we move the values before
// the insertion point one position to the left. Otherwise, we proceed as described above (moving
// the entire value block to the end of the value array).

struct ValueBlockPosition {

    int32_t start;
    int32_t end;
};

// Returns the index in the value arrays of the newly inserted value.
template<typename T, typename IndexArray, typename ValueArray, typename... ExtraValueArrays>
inline int insertion(
        const int row, const T &value,
        IndexArray &indexArray, ValueArray &valueArray, ExtraValueArrays &... extraValueArrays) {
    assert(row >= 0);
    assert(row < indexArray.size());
    auto &start = indexArray[row].start;
    auto &end = indexArray[row].end;
    const auto hole = std::numeric_limits<typename ValueArray::value_type>::max();
    int valIdx;

    if (end != valueArray.size() && valueArray[end] == hole) {
        // If the element immediately after the row's block is a hole, the new value fills this hole.
        ++end;
        valIdx = end - 1;
    } else if (start != 0 && valueArray[start - 1] == hole) {
        // Analogously, if the element before the block is a hole, the new value fills that hole.
        --start;
        valIdx = start;
    } else {
        // Otherwise, we move the entire block to the end of the value arrays.
        static constexpr auto CAPACITY_GROWTH_FACTOR = 2.0;
        const auto blockSize = end - start;
        const int cap = std::max(CAPACITY_GROWTH_FACTOR * blockSize, blockSize + 1.0);

        if (end == valueArray.size()) {
            valueArray.insert(valueArray.end(), cap - blockSize, hole);
            (extraValueArrays.insert(
                    extraValueArrays.end(), cap - blockSize, typename ExtraValueArrays::value_type()), ...);
            ++end;
        } else {
            valueArray.insert(valueArray.end(), cap, hole);
            std::copy(valueArray.begin() + start, valueArray.begin() + end, valueArray.end() - cap);
            std::fill(valueArray.begin() + start, valueArray.begin() + end, hole);
            (extraValueArrays.insert(
                    extraValueArrays.end(), cap, typename ExtraValueArrays::value_type()), ...);
            (std::copy(
                    extraValueArrays.begin() + start, extraValueArrays.begin() + end,
                    extraValueArrays.end() - cap), ...);
            start = valueArray.size() - cap;
            end = start + blockSize + 1;
        }

        valIdx = end - 1;
    }

    valueArray[valIdx] = value;
    return valIdx;
}

template<typename IndexArray, typename ValueArray, typename... ExtraValueArrays>
inline void removal(
        const int row, const int col,
        IndexArray &indexArray, ValueArray &valueArray, ExtraValueArrays &... extraValueArrays) {
    assert(row >= 0);
    assert(row < indexArray.size());
    assert(col >= 0);
    assert(col < indexArray[row].end - indexArray[row].start);
    auto &start = indexArray[row].start;
    auto &end = indexArray[row].end;
    --end;
    valueArray[start + col] = valueArray[end];
    valueArray[end] = std::numeric_limits<typename ValueArray::value_type>::max();
    ((extraValueArrays[start + col] = extraValueArrays[end]), ...);
}

// Removes entries given as a range of column indices, sorted in ascending order.
template<typename ColsRangeT, typename IndexArray, typename ValueArray, typename... ExtraValueArrays>
inline void removalOfSortedCols(
        const int row, const ColsRangeT &cols,
        IndexArray &indexArray, ValueArray &valueArray, ExtraValueArrays &... extraValueArrays) {
    assert(std::is_sorted(cols.begin(), cols.end()));
    // Iterate in reverse
    for (auto colIt = cols.rbegin(); colIt != cols.rend(); ++colIt) {
        removal(row, *colIt, indexArray, valueArray, extraValueArrays...);
    }
}

// Removes all columns at a given row. No need to explicitly change extra value arrays since the validity of values in
// those is defined only via indexArray.
template<typename IndexArray, typename ValueArray>
inline void removalOfAllCols(
        const int row, IndexArray &indexArray, ValueArray &valueArray) {
    assert(row >= 0);
    assert(row < indexArray.size());
    const auto &start = indexArray[row].start;
    auto &end = indexArray[row].end;
    while (end > start) {
        --end;
        valueArray[end] = std::numeric_limits<typename ValueArray::value_type>::max();
    }
}

// Returns the index in the value arrays of the newly inserted value.
template<typename T, typename IndexArray, typename ValueArray, typename... ExtraValueArrays>
inline int stableInsertion(
        const int row, const int col, const T &value,
        IndexArray &indexArray, ValueArray &valueArray, ExtraValueArrays &... extraValueArrays) {
    assert(row >= 0);
    assert(row < indexArray.size());
    assert(col >= 0);
    assert(col <= indexArray[row].end - indexArray[row].start);
    const auto valIdx = insertion(row, value, indexArray, valueArray, extraValueArrays...);
    const auto start = indexArray[row].start;
    const auto end = indexArray[row].end;
    if (valIdx == start) {
        std::copy(
                valueArray.begin() + start + 1, valueArray.begin() + start + col + 1,
                valueArray.begin() + start);
        (std::copy(
                extraValueArrays.begin() + start + 1, extraValueArrays.begin() + start + col + 1,
                extraValueArrays.begin() + start), ...);
    } else {
        std::copy_backward(
                valueArray.begin() + start + col, valueArray.begin() + end - 1, valueArray.begin() + end);
        (std::copy_backward(
                extraValueArrays.begin() + start + col, extraValueArrays.begin() + end - 1,
                extraValueArrays.begin() + end), ...);
    }
    valueArray[start + col] = value;
    return start + col;
}

template<typename IndexArray, typename ValueArray, typename... ExtraValueArrays>
inline void stableRemoval(
        const int row, const int col,
        IndexArray &indexArray, ValueArray &valueArray, ExtraValueArrays &... extraValueArrays) {
    assert(row >= 0);
    assert(row < indexArray.size());
    assert(col >= 0);
    assert(col < indexArray[row].end - indexArray[row].start);
    auto &start = indexArray[row].start;
    auto &end = indexArray[row].end;
    std::copy(
            valueArray.begin() + start + col + 1, valueArray.begin() + end,
            valueArray.begin() + start + col);
    (std::copy(
            extraValueArrays.begin() + start + col + 1, extraValueArrays.begin() + end,
            extraValueArrays.begin() + start + col), ...);
    --end;
    valueArray[end] = std::numeric_limits<typename ValueArray::value_type>::max();
}

// Removes entries given as a range of column indices, sorted in ascending order.
template<typename ColsRangeT, typename IndexArray, typename ValueArray, typename... ExtraValueArrays>
inline void stableRemovalOfSortedCols(
        const int row, const ColsRangeT &cols,
        IndexArray &indexArray, ValueArray &valueArray, ExtraValueArrays &... extraValueArrays) {
    assert(std::is_sorted(cols.begin(), cols.end()));
    assert(row >= 0);
    assert(row < indexArray.size());
    const auto &start = indexArray[row].start;
    auto &end = indexArray[row].end;

    int i = 0;
    for (auto it = cols.begin(); it != cols.end();) {
        auto col = *it;
        assert(col >= i);
        assert(col < end - start);

        ++it;
        const auto nextCol = it == cols.end() ? end - start : *it;

        std::copy(valueArray.begin() + start + col + 1, valueArray.begin() + start + nextCol,
                  valueArray.begin() + start + col - i);
        (std::copy(
                extraValueArrays.begin() + start + col + 1, extraValueArrays.begin() + start + nextCol,
                extraValueArrays.begin() + start + col - i), ...);
        ++i;
    }
    std::fill(valueArray.begin() + end - i, valueArray.begin() + end,
              std::numeric_limits<typename ValueArray::value_type>::max());
    end -= i;
}
