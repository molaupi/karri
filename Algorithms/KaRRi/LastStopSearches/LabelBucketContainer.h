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

#include "DataStructures/Utilities/DynamicRagged2DArrays.h"

namespace karri {


// Stores one bucket per vertex containing labels propagated by a search. Every bucket consists of a range of
// closed labels (have been settled) at the beginning and open labels (have not been settled yet) at the end. The search
// is supposed to keep the whole sorted in ascending order of cost. The search always settles the open label with the
// lowest cost next which is why all closed labels always come earlier in this order than any open label. The order of open
// labels is subject to cost updates for individual labels (i.e. each bucket is used as a cost PQ in its own right).
    template<typename BucketEntryT>
    class LabelBucketContainer {

    private:
        struct BucketPosition {
            int start = 0;
            int numClosed = 0;
            int end = 0;
        };
    public:

        using Label = BucketEntryT;

        struct Bucket {
            using Section = ConstantVectorRange<BucketEntryT>;
            using ConstIt = typename ConstantVectorRange<BucketEntryT>::Iterator;

            Section closed() const {
                return {entries.begin() + bucketPos.start, entries.begin() + bucketPos.start + bucketPos.numClosed};
            }

            Section open() const {
                return {entries.begin() + bucketPos.start + bucketPos.numClosed, entries.begin() + bucketPos.end};
            }

        private:
            friend LabelBucketContainer;

            Bucket(const BucketPosition &bucketPos, const std::vector<BucketEntryT> &entries)
                    : bucketPos(bucketPos), entries(entries) {
                assert(bucketPos.end >= bucketPos.start && bucketPos.end - bucketPos.start >= bucketPos.numClosed);
            }

            const BucketPosition &bucketPos;
            const std::vector<BucketEntryT> &entries;
        };

        LabelBucketContainer(const int numVertices)
                : bucketPositions(numVertices, {0, 0, 0}), entries() {
            assert(numVertices >= 0);
        }

        // Returns the bucket of the specified vertex.
        Bucket getBucketOf(const int vertex) {
            assert(vertex >= 0);
            assert(vertex < bucketPositions.size());
            return Bucket(bucketPositions[vertex], entries);
        }

        // Inserts the given entry into the bucket of the specified vertex at the given index in the bucket.
        void stableInsertOpenLabel(const int vertex, const int index, const BucketEntryT &entry) {
            const int numClosed = bucketPositions[vertex].numClosed;
            assert(bucketPositions[vertex].end - bucketPositions[vertex].start >= numClosed);
            stableInsertion(vertex, index + numClosed, entry, bucketPositions, entries);
        }

        // Stable removal of the entries at the specified indices. Range of indices has to be sorted. Indices mean the
        // position in the range of open labels of the bucket, not the whole bucket.
        template<typename IndexRangeT>
        void stableRemoveOpenLabelsAtIndices(const int vertex, const IndexRangeT &indices) {
            assert(vertex >= 0);
            assert(vertex < bucketPositions.size());
            assert(std::all_of(indices.begin(), indices.end(),
                               [&](const int idx) { return idx >= 0 && idx < getBucketOf(vertex).open().size(); }));
            const int numClosed = bucketPositions[vertex].numClosed;
            assert(bucketPositions[vertex].end - bucketPositions[vertex].start >= numClosed);
            auto indicesWithOffset = std::vector<int>(indices.begin(), indices.end());
            std::for_each(indicesWithOffset.begin(), indicesWithOffset.end(),
                          [numClosed](int &idx) { idx += numClosed; });

            stableRemovalOfSortedCols(vertex, indicesWithOffset, bucketPositions, entries);
            assert(bucketPositions[vertex].end >= bucketPositions[vertex].start &&
                   bucketPositions[vertex].end - bucketPositions[vertex].start >= bucketPositions[vertex].numClosed);
        }

        // Returns the label in the bucket of vertex with the minimum cost.
        const BucketEntryT &minOpenLabel(const int vertex) {
            assert(vertex >= 0);
            assert(vertex < bucketPositions.size());
            auto &bucketPos = bucketPositions[vertex];
            assert(bucketPos.start + bucketPos.numClosed < bucketPos.end);
            return entries[bucketPos.start + bucketPos.numClosed];
        }

        // Closes the open label with the best cost at vertex and returns it.
        BucketEntryT closeMinOpenLabel(const int vertex) {
            assert(vertex >= 0);
            assert(vertex < bucketPositions.size());
            auto &bucketPos = bucketPositions[vertex];
            assert(bucketPos.start + bucketPos.numClosed < bucketPos.end);
            const auto minOpenLabel = entries[bucketPos.start + bucketPos.numClosed];
            ++bucketPos.numClosed;
            return minOpenLabel;
        }

        void clearBucket(const int vertex) {
            assert(vertex >= 0);
            assert(vertex < bucketPositions.size());
            removalOfAllCols(vertex, bucketPositions, entries);
            bucketPositions[vertex].numClosed = 0;
        }

        void clear() {
            bucketPositions.clear();
            entries.clear();
        }

        // Checks whether all buckets are empty. This may be expensive!
        bool allEmpty() const {
            const auto hole = BucketEntryT();
            return bucketPositions.allInvalid() &&
                   std::all_of(entries.begin(), entries.end(),
                               [hole](const BucketEntryT &entry) { return entry == hole; });
        }

    private:

        TimestampedVector<BucketPosition> bucketPositions;
        std::vector<BucketEntryT> entries;

    };

}