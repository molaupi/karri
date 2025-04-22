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
#include <array>
#include <cassert>
#include <type_traits>

#include "DataStructures/Labels/ParentInfo.h"
#include "Tools/TemplateProgramming.h"

// A set of consistent distance and parent labels for Dijkstra's algorithm. The template arguments
// specify the number of shortest paths computed simultaneously and the kind of parent information
// that should be collected.
template<int logSearches, ParentInfo parentInfo>
struct BasicLabelSet {
public:
    // The number of simultaneous shortest-path computations.
    static constexpr int logK = logSearches;
    static constexpr int K = 1 << logK;

    // Flags indicating whether parent vertices and/or parent edges should be collected.
    static constexpr bool KEEP_PARENT_VERTICES = parentInfo != ParentInfo::NO_PARENT_INFO;
    static constexpr bool KEEP_PARENT_EDGES = parentInfo == ParentInfo::FULL_PARENT_INFO;

    // A mask that marks a subset of components in a packed distance label. For example, the result
    // of a less-than comparison between two multiple-source distance labels a and b is a mask that
    // indicates for which components i it holds that a[i] < b[i].
    class LabelMask {
    public:
        // Constructs an uninitialized mask.
        LabelMask() = default;

        // Constructs a mask with all k components set to val. Converting constructor.
        LabelMask(const bool val) {
            std::fill(isMarked.begin(), isMarked.end(), val);
        }

        // Takes the logical AND of this and the specified mask.
        LabelMask &operator&=(const LabelMask &rhs) {
            for (int i = 0; i < K; ++i)
                isMarked[i] &= rhs.isMarked[i];
            return *this;
        }

        friend LabelMask operator&(const LabelMask &mask1, const LabelMask &mask2) {
            LabelMask res;
            for (int i = 0; i < K; ++i)
                res.isMarked[i] = mask1.isMarked[i] & mask2.isMarked[i];
            return res;
        }

        // Takes the logical AND of this and the specified mask.
        LabelMask &operator|=(const LabelMask &rhs) {
            for (int i = 0; i < K; ++i)
                isMarked[i] |= rhs.isMarked[i];
            return *this;
        }

        friend LabelMask operator|(const LabelMask &mask1, const LabelMask& mask2) {
            LabelMask res;
            for (int i = 0; i < K; ++i) {
                res.isMarked[i] = mask1.isMarked[i] | mask2.isMarked[i];
            }
            return res;
        }

        // Returns the logical NOT of this mask.
        LabelMask operator~() const {
            LabelMask res = *this;
            for (int i = 0; i < K; ++i)
                res.isMarked[i] = !res.isMarked[i];
            return res;
        }

        // Returns the i-th value in this mask.
        bool operator[](const int i) const {
            assert(i >= 0);
            assert(i < K);
            return isMarked[i];
        }

        // Returns the i-th value in this mask.
        bool operator[](const int i) {
            assert(i >= 0);
            assert(i < K);
            return isMarked[i];
        }

//        // Returns true if this mask marks at least one component.
//        operator bool() const {
//            bool res = isMarked[0];
//            for (int i = 1; i < K; ++i)
//                res |= isMarked[i];
//            return res;
//        }

        friend bool anySet(const LabelMask &mask) {
            bool res = mask.isMarked[0];
            for (int i = 1; i < K; ++i)
                res |= mask.isMarked[i];
            return res;
        }

        friend bool allSet(const LabelMask &mask) {
            return !anySet(~mask);
        }

        std::array<bool, K> isMarked; // Flags indicating for each component if it is marked.
    };

    // A packed distance label for a vertex, storing k distance values. Each value maintains the
    // tentative distance from a different simultaneous source.
    class DistanceLabel {
    public:

        static constexpr int K = BasicLabelSet::K;

        // Constructs an uninitialized distance label.
        DistanceLabel() = default;

        // Constructs a distance label with all k values set to val. Converting constructor.
        constexpr DistanceLabel(const int val) {
            std::fill(values.begin(), values.end(), val);
        }

        // Returns a reference to the i-th distance value in this label.
        int &operator[](const int i) {
            assert(i >= 0);
            assert(i < K);
            return values[i];
        }

        // Returns the i-th distance value in this label.
        int operator[](const int i) const {
            assert(i >= 0);
            assert(i < K);
            return values[i];
        }

        DistanceLabel& operator+=(const DistanceLabel &rhs) {
            for (int i = 0; i < K; ++i)
                values[i] += rhs.values[i];
            return *this;
        }

        DistanceLabel& operator-=(const DistanceLabel &rhs) {
            for (int i = 0; i < K; ++i)
                values[i] -= rhs.values[i];
            return *this;
        }

        // Returns the packed sum of lhs and rhs.
        friend DistanceLabel operator+(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            DistanceLabel sum;
            for (int i = 0; i < K; ++i)
                sum.values[i] = lhs.values[i] + rhs.values[i];
            return sum;
        }

        // Packed minus.
        friend DistanceLabel operator-(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            DistanceLabel diff;
            for (int i = 0; i < K; ++i)
                diff.values[i] = lhs.values[i] - rhs.values[i];
            return diff;
        }

        // Returns a mask that indicates for which components i it holds that lhs[i] < rhs[i].
        friend LabelMask operator<(const DistanceLabel &lhs, const DistanceLabel &rhs) {
            LabelMask mask;
            for (int i = 0; i < K; ++i)
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
            for (int i = 0; i < K; ++i)
                mask.isMarked[i] = lhs.values[i] == rhs.values[i];
            return mask;
        }

        // Given a label l and a mask m, this returns a label l' s.t. l'[i] = l[i] if mask[i] = true and l'[i] = 0 if mask[i] = false
        friend DistanceLabel operator&(const DistanceLabel &l, const LabelMask &m) {
            DistanceLabel res;
            for (int i = 0; i < K; ++i)
                res[i] = m.isMarked[i] ? l[i] : 0;
            return res;
        }

        // Returns the priority of this label.
        int getKey() const {
            int min = values[0];
            for (int i = 1; i < K; ++i)
                min = std::min(min, values[i]);
            return min;
        }

        // Returns the horizontal minimum amongst the packed distance values in this label.
        int horizontalMin() const {
            int min = values[0];
            for (int i = 1; i < K; ++i)
                min = std::min(min, values[i]);
            return min;
        }

        // Returns the horizontal maximum amongst the packed distance values in this label.
        int horizontalMax() const {
            int max = values[0];
            for (int i = 1; i < K; ++i)
                max = std::max(max, values[i]);
            return max;
        }

        // Take the packed minimum of this and the specified label.
        void min(const DistanceLabel &other) {
            for (int i = 0; i < K; ++i)
                values[i] = std::min(values[i], other.values[i]);
        }

        // Take the packed maximum of this and the specified label.
        void max(const DistanceLabel &other) {
            for (int i = 0; i < K; ++i)
                values[i] = std::max(values[i], other.values[i]);
        }

        // Multiply all entries in this label with a scalar factor
        void multiplyWithScalar(const int s) {
            for (int i = 0; i < K; ++i)
                values[i] = s * values[i];
        }

        friend DistanceLabel multiplyDistanceLabelAndScalar(const DistanceLabel &l, const int s) {
            DistanceLabel res;
            for (int i = 0; i < K; ++i)
                res.values[i] = s * l.values[i];
            return res;
        }

        // Sets this label at all slots i where mask[i] = true.
        void setIf(const DistanceLabel &other, const LabelMask &mask) {
            for (int i = 0; i < K; ++i)
                values[i] = mask[i] ? other.values[i] : values[i];
        }

        // Select between two operands. Corresponds to this pseudocode:
        // for (int i = 0; i < K; i++) result[i] = mask[i] ? l1[i] : l2[i];
        friend DistanceLabel select(const LabelMask &mask, const DistanceLabel &l1, const DistanceLabel &l2) {
            DistanceLabel result;
            for (int i = 0; i < K; ++i)
                result[i] = mask[i] ? l1[i] : l2[i];
            return result;
        }

    private:
        std::array<int, K> values; // The k distance values, one for each simultaneous source.
    };

private:
    // A packed label for a vertex, storing k parent edges.
    class ParentEdge {

    public:

        ParentEdge() {
            edges.fill(INVALID_ID);
        }

        // Returns the parent edge on the shortest path from the i-th source.
        int edge(const int i) const {
            assert(i >= 0);
            assert(i < K);
            return edges[i];
        }

        // Sets the parent edge to e on all shortest paths specified by mask.
        void setEdge(const int e, const LabelMask &mask) {
            for (int i = 0; i < K; ++i)
                edges[i] = mask.isMarked[i] ? e : edges[i];
        }

    private:
        std::array<int, K> edges; // The k parent edges, one for each simultaneous source.
    };

public:
    // A packed label for a vertex, storing k parent vertices and possibly k parent edges.
    class ParentLabel : public std::conditional_t<KEEP_PARENT_EDGES, ParentEdge, EmptyClass> {
    public:

        ParentLabel() = default;

        ParentLabel(const int val) {
            vertices.fill(val);

            if (ParentEdge *e = dynamic_cast<ParentEdge *>(this)) {
                e->setEdge(val, true);
            }
        }

        // Returns the parent vertex on the shortest path from the i-th source.
        int vertex(const int i) const {
            assert(i >= 0);
            assert(i < K);
            return vertices[i];
        }

        // Sets the parent vertex to u on all shortest paths specified by mask.
        void setVertex(const int u, const LabelMask &mask) {
            for (int i = 0; i < K; ++i)
                vertices[i] = mask.isMarked[i] ? u : vertices[i];
        }

    private:
        std::array<int, K> vertices; // The k parent vertices, one for each simultaneous source.
    };
};
