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

#include <array>
#include <cassert>
#include <type_traits>

#include <vectorclass.h>

#include "DataStructures/Labels/ParentInfo.h"
#include "Tools/TemplateProgramming.h"

// A set of consistent distance and parent labels for Dijkstra's algorithm. The template arguments
// specify the number of shortest paths computed simultaneously and the kind of parent information
// that should be collected.
template<int logSearches, ParentInfo parentInfo>
struct SimdLabelSet {
    static_assert(logSearches >= 2, "The SIMD label set requires at least 4 simultaneous searches.");

public:
    // The number of simultaneous shortest-path computations.
    static constexpr int logK = logSearches;
    static constexpr int K = 1 << logK;

    // Vectors of multiple data elements for use with SIMD instructions.
    using BooleanVector = std::conditional_t<logK == 2, Vec4ib, Vec8ib>;
    using IntegerVector = std::conditional_t<logK == 2, Vec4i, Vec8i>;

    // Flags indicating whether parent vertices and/or parent edges should be collected.
    static constexpr bool KEEP_PARENT_VERTICES = parentInfo != ParentInfo::NO_PARENT_INFO;
    static constexpr bool KEEP_PARENT_EDGES = parentInfo == ParentInfo::FULL_PARENT_INFO;

private:
    static constexpr int VECTOR_SIZE = logK == 2 ? 4 : 8; // The number of data elements per vector.
    static constexpr int NUM_VECTORS = K / VECTOR_SIZE; // The number of vectors per label.

    // Arrays of vectors that store all k values of a packed label.
    using BooleanLabel = std::array<BooleanVector, NUM_VECTORS>;
    using IntegerLabel = std::array<IntegerVector, NUM_VECTORS>;

public:
    // A mask that marks a subset of components in a packed distance label. For example, the result
    // of a less-than comparison between two multiple-source distance labels a and b is a mask that
    // indicates for which components i it holds that a[i] < b[i].
    class LabelMask {
    public:
        // Constructs an uninitialized mask.
        LabelMask() = default;

        // Constructs a mask with all k components set to val. Converting constructor.
        LabelMask(const bool val) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                isMarked[i] = val;
        }

        // Takes the logical AND of this and the specified mask.
        LabelMask &operator&=(const LabelMask &rhs) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                isMarked[i] &= rhs.isMarked[i];
            return *this;
        }

        friend LabelMask operator&(const LabelMask &mask1, const LabelMask &mask2) {
            LabelMask res;
            for (int i = 0; i < NUM_VECTORS; ++i)
                res.isMarked[i] = mask1.isMarked[i] & mask2.isMarked[i];
            return res;
        }

        // Takes the logical AND of this and the specified mask.
        LabelMask &operator|=(const LabelMask &rhs) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                isMarked[i] |= rhs.isMarked[i];
            return *this;
        }

        friend LabelMask operator|(const LabelMask &mask1, const LabelMask& mask2) {
            LabelMask res;
            for (int i = 0; i < NUM_VECTORS; ++i) {
                res.isMarked[i] = mask1.isMarked[i] | mask2.isMarked[i];
            }
            return res;
        }

        // Returns the logical NOT of a mask.
        friend LabelMask operator~(const LabelMask &mask) {
            LabelMask res;
            for (int i = 0; i < NUM_VECTORS; ++i)
                res.isMarked[i] = ~mask.isMarked[i];
            return res;
        }

        // Returns the i-th value in this mask.
        bool operator[](const int i) const {
            assert(i >= 0);
            assert(i < K);
            return isMarked[i / VECTOR_SIZE][i % VECTOR_SIZE];
        }

        // Returns the i-th value in this mask.
        bool operator[](const int i) {
            assert(i >= 0);
            assert(i < K);
            return isMarked[i / VECTOR_SIZE][i % VECTOR_SIZE];
        }

//        // Returns true if this mask marks at least one component.
//        operator bool() const {
//            BooleanVector tmp = isMarked[0];
//            for (int i = 1; i < NUM_VECTORS; ++i)
//                tmp |= isMarked[i];
//            return horizontal_or(tmp);
//        }

        friend bool anySet(const LabelMask &mask) {
            BooleanVector tmp = mask.isMarked[0];
            for (int i = 1; i < NUM_VECTORS; ++i)
                tmp |= mask.isMarked[i];
            return horizontal_or(tmp);
        }

        friend bool allSet(const LabelMask &mask) {
            return !anySet(~mask);
        }

        BooleanLabel isMarked; // Flags indicating for each component if it is marked.
    };

    // A packed distance label for a vertex, storing k distance values. Each value maintains the
    // tentative distance from a different simultaneous source.
    class DistanceLabel {
    public:

        static constexpr int K = SimdLabelSet::K;

        // A class simulating the behavior of references to a single distance value in a packed label.
        class Reference {
            // Only DistanceLabel is allowed to construct references to a single distance value.
            friend class DistanceLabel;

        public:
            // Sets the distance value to val.
            Reference &operator=(const int val) {
                block.insert(localIndex, val);
                return *this;
            }

            // Returns the distance value.
            operator int() const {
                return block.extract(localIndex);
            }

        private:
            // Constructs a reference to the i-th distance value in the specified block.
            Reference(IntegerVector &block, const int i) : block(block), localIndex(i) {}

            IntegerVector &block; // The block containing the distance value.
            const int localIndex; // The index of the distance value in the block above.
        };

        // Constructs an uninitialized distance label.
        DistanceLabel() = default;

        // Constructs a distance label with all k values set to val. Converting constructor.
        DistanceLabel(const int val) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                values[i] = val;
        }

        // Returns a reference to the i-th distance value in this label.
        Reference operator[](const int i) {
            assert(i >= 0);
            assert(i < K);
            return Reference(values[i / VECTOR_SIZE], i % VECTOR_SIZE);
        }

        // Returns the i-th distance value in this label.
        int operator[](const int i) const {
            assert(i >= 0);
            assert(i < K);
            return values[i / VECTOR_SIZE].extract(i % VECTOR_SIZE);
        }

        DistanceLabel& operator+=(const DistanceLabel &rhs) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                values[i] += rhs.values[i];
            return *this;
        }

        DistanceLabel& operator-=(const DistanceLabel &rhs) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                values[i] -= rhs.values[i];
            return *this;
        }

        // Returns the packed sum of lhs and rhs.
        friend DistanceLabel operator+(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            DistanceLabel sum;
            for (int i = 0; i < NUM_VECTORS; ++i)
                sum.values[i] = lhs.values[i] + rhs.values[i];
            return sum;
        }

        // Packed minus.
        friend DistanceLabel operator-(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            DistanceLabel diff;
            for (int i = 0; i < NUM_VECTORS; ++i)
                diff.values[i] = lhs.values[i] - rhs.values[i];
            return diff;
        }

        // Returns a mask that indicates for which components i it holds that lhs[i] < rhs[i].
        friend LabelMask operator<(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            LabelMask mask;
            for (int i = 0; i < NUM_VECTORS; ++i)
                mask.isMarked[i] = lhs.values[i] < rhs.values[i];
            return mask;
        }

        // Returns a mask that indicates for which components i it holds that lhs[i] <= rhs[i].
        friend LabelMask operator<=(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            return ~(rhs < lhs);
        }

        // Returns a mask that indicates for which components i it holds that lhs[i] > rhs[i].
        friend LabelMask operator>(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            return rhs < lhs;
        }

        // Returns a mask that indicates for which components i it holds that lhs[i] >= rhs[i].
        friend LabelMask operator>=(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            return rhs <= lhs;
        }

        // Returns a mask that indicates for which components i it holds that lhs[i] == rhs[i].
        friend LabelMask operator==(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            LabelMask mask;
            for (int i = 0; i < NUM_VECTORS; ++i)
                mask.isMarked[i] = lhs.values[i] == rhs.values[i];
            return mask;
        }

        // Given a label l and a mask m, this returns a label l' s.t. l'[i] = l[i] if mask[i] = true and l'[i] = 0 if mask[i] = false
        friend DistanceLabel operator&(const DistanceLabel &l, const LabelMask &m) {
            DistanceLabel res;
            for (int i = 0; i < NUM_VECTORS; ++i)
                res[i] = l.values[i] & m.isMarked[i];
            return res;
        }

        // Returns the priority of this label.
        int getKey() const {
            IntegerVector packedMin = values[0];
            for (int i = 1; i < NUM_VECTORS; ++i)
                packedMin = ::min(packedMin, values[i]);
            return horizontal_min(packedMin);
        }

        // Returns the horizontal minimum amongst the packed distance values in this label.
        int horizontalMin() const {
            IntegerVector packedMin = values[0];
            for (int i = 1; i < NUM_VECTORS; ++i)
                packedMin = ::min(packedMin, values[i]);
            return horizontal_min(packedMin);
        }

        // Returns the horizontal maximum amongst the packed distance values in this label.
        int horizontalMax() const {
            IntegerVector packedMax = values[0];
            for (int i = 1; i < NUM_VECTORS; ++i)
                packedMax = ::max(packedMax, values[i]);
            return horizontal_max(packedMax);
        }

        // Take the packed minimum of this and the specified label.
        void min(const DistanceLabel &other) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                values[i] = ::min(values[i], other.values[i]);
        }

        // Take the packed maximum of this and the specified label.
        void max(const DistanceLabel &other) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                values[i] = ::max(values[i], other.values[i]);
        }

        // Multiply all entries in this label with a scalar factor
        void multiplyWithScalar(const int s) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                values[i] = s * values[i];
        }

        friend DistanceLabel multiplyDistanceLabelAndScalar(const DistanceLabel &l, const int s) {
            DistanceLabel res;
            for (int i = 0; i < NUM_VECTORS; ++i)
                res.values[i] = s * l.values[i];
            return res;
        }

        // Sets this label at all slots i where mask[i] = true.
        void setIf(const DistanceLabel &other, const LabelMask &mask) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                values[i] = select(mask.isMarked[i], other.values[i], values[i]);
        }

        // Select between two operands. Corresponds to this pseudocode:
        // for (int i = 0; i < K; i++) result[i] = mask[i] ? l1[i] : l2[i];
        friend DistanceLabel select(const LabelMask &mask, const DistanceLabel &l1, const DistanceLabel &l2) {
            DistanceLabel result;
            for (int i = 0; i < NUM_VECTORS; ++i)
                result.values[i] = select(mask.isMarked[i], l1.values[i], l2.values[i]);
            return result;
        }

    private:
        IntegerLabel values; // The k distance values, one for each simultaneous source.
    };

private:
    // A packed label for a vertex, storing k parent edges.
    class ParentEdge {

    public:
        // Returns the parent edge on the shortest path from the i-th source.
        int edge(const int i) const {
            assert(i >= 0);
            assert(i < K);
            return edges[i / VECTOR_SIZE].extract(i % VECTOR_SIZE);
        }

        // Sets the parent edge to e on all shortest paths specified by mask.
        void setEdge(const int e, const LabelMask &mask) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                edges[i] = select(mask.isMarked[i], e, edges[i]);
        }

    private:
        IntegerLabel edges; // The k parent edges, one for each simultaneous source.
    };

public:
    // A packed label for a vertex, storing k parent vertices and possibly k parent edges.
    class ParentLabel : public std::conditional_t<KEEP_PARENT_EDGES, ParentEdge, EmptyClass> {
    public:

        ParentLabel() = default;

        ParentLabel(const int val) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                vertices[i] = val;

            if (ParentEdge * e = dynamic_cast<ParentEdge *>(this)) {
                e->setEdge(val, true);
            }
        }

        // Returns the parent vertex on the shortest path from the i-th source.
        int vertex(const int i) const {
            assert(i >= 0);
            assert(i < K);
            return vertices[i / VECTOR_SIZE].extract(i % VECTOR_SIZE);
        }

        // Sets the parent vertex to u on all shortest paths specified by mask.
        void setVertex(const int u, const LabelMask &mask) {
            for (int i = 0; i < NUM_VECTORS; ++i)
                vertices[i] = select(mask.isMarked[i], u, vertices[i]);
        }

    private:
        IntegerLabel vertices; // The k parent vertices, one for each simultaneous source.
    };
};
