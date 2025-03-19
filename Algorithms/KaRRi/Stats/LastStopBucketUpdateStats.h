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

#include <cstdint>
#include "DataStructures/Utilities/CompactBatchRagged2DArrays.h"

namespace karri {

    struct LastStopBucketUpdateStats {

        int64_t findInsertionsAndDeletionsTime = 0;
        int64_t numInsertions = 0;
        int64_t numDeletions = 0;
        int64_t accumulateThreadLocalInsertionsTime = 0;
        int64_t accumulateThreadLocalDeletionsTime = 0;
        int64_t sortIdleInsertionsByCompTime = 0;
        int64_t sortNonIdleInsertionsByCompTime = 0;
        int64_t updateNumIdleEntriesTime = 0;
        compact_batch_ragged2d::BatchedInsertionsAndDeletionsStats arrayUpdateStats = {};

        static std::string addPrefixToEachLoggerCol(const auto &prefix, char const* const colsCharArray) {
            std::stringstream in(colsCharArray);
            std::stringstream out;
            std::string item;

            while (getline (in, item, ',')) {
                out << prefix << item << ",";
            }

            return out.str();
        }


        int64_t getTotalTime() const {
            return findInsertionsAndDeletionsTime + accumulateThreadLocalInsertionsTime +
                    sortIdleInsertionsByCompTime + sortNonIdleInsertionsByCompTime + updateNumIdleEntriesTime +
                   accumulateThreadLocalDeletionsTime + arrayUpdateStats.getTotalTime();
        }


        static std::string getLoggerCols() {
            std::stringstream ss;
            ss << "find_insertions_and_deletions_time,"
               << "num_insertions,"
               << "num_deletions,"
               << "accumulate_thread_local_insertions_time,"
               << "accumulate_thread_local_deletions_time,"
               << "sort_idle_insertions_by_comp_time,"
                << "sort_non_idle_insertions_by_comp_time,"
                << "update_num_idle_entries_time,"
               << addPrefixToEachLoggerCol("2darray.", compact_batch_ragged2d::BatchedInsertionsAndDeletionsStats::LOGGER_COLS)
               << "total_time";
            return ss.str();
        }

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << findInsertionsAndDeletionsTime << "," << numInsertions << "," << numDeletions << ","
                << accumulateThreadLocalInsertionsTime << "," << accumulateThreadLocalDeletionsTime << ","
                << sortIdleInsertionsByCompTime << "," << sortNonIdleInsertionsByCompTime << ","
                << updateNumIdleEntriesTime << ","
                << arrayUpdateStats.getLoggerRow() << "," << getTotalTime();
            return ss.str();
        }


    };

} // namespace karri

