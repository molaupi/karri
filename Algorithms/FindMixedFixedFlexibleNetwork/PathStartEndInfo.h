/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "DataStructures/Utilities/IteratorRange.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include <ranges>

namespace mixfix {


    class PathStartEndInfo {

    public:
        explicit PathStartEndInfo(const int numVertices)
                : beginningsIndex(numVertices, {0, 0}),
                  pathBeginnings(),
                  beginningsWalkingDists(),
                  endsIndex(numVertices, {0, 0}),
                  pathEnds(),
                  endsWalkingDists() {}

        explicit PathStartEndInfo(std::ifstream &in) {
            bio::read(in, beginningsIndex);
            bio::read(in, pathBeginnings);
            bio::read(in, beginningsEdgeIdx);
            bio::read(in, beginningsWalkingDists);
            bio::read(in, endsIndex);
            bio::read(in, pathEnds);
            bio::read(in, endsEdgeRevIdx);
            bio::read(in, endsWalkingDists);

            LIGHT_KASSERT(getSmallestVertexIdWithUnsortedRange() == INVALID_VERTEX,
                          "Read path start info with unsorted beginnings / ends ranges.");

            // Make sure ID mappings all point to themselves for initial paths
            int maxPathId = -1;
            for (const auto &i: pathBeginnings)
                maxPathId = std::max(maxPathId, i);
            for (const auto &i: pathEnds)
                maxPathId = std::max(maxPathId, i);

            mapToBeginningActiveDescendent.reserve(maxPathId + 1);
            mapToBeginningAncestor.reserve(maxPathId + 1);
            mapToEndActiveDescendent.reserve(maxPathId + 1);
            mapToEndAncestor.reserve(maxPathId + 1);
            for (int i = 0; i <= maxPathId; ++i) {
                mapToBeginningActiveDescendent.push_back(i);
                mapToBeginningAncestor.push_back(i);
                mapToEndActiveDescendent.push_back(i);
                mapToEndAncestor.push_back(i);
            }
        }

        // Call defrag() first!
        void writeTo(std::ofstream &out) {
            defrag();
            LIGHT_KASSERT(getSmallestVertexIdWithUnsortedRange() == INVALID_VERTEX);
            bio::write(out, beginningsIndex);
            bio::write(out, pathBeginnings);
            bio::write(out, beginningsEdgeIdx);
            bio::write(out, beginningsWalkingDists);
            bio::write(out, endsIndex);
            bio::write(out, pathEnds);
            bio::write(out, endsEdgeRevIdx);
            bio::write(out, endsWalkingDists);
        }

        // Returns IDs of paths that may start at vertex v.
        auto getPathsPossiblyBeginningAt(const int v) const {
            KASSERT(v >= 0);
            KASSERT(v < beginningsIndex.size());
            const auto start = beginningsIndex[v].start;
            const auto end = beginningsIndex[v].end;
            return std::ranges::subrange(pathBeginnings.begin() + start, pathBeginnings.begin() + end) |
                   std::views::transform([&](const int id) {
                       return mapToBeginningActiveDescendent[id];
                   });
        }

        bool hasPathBeginningAt(const int v, const int pathId) const {
            KASSERT(v >= 0);
            KASSERT(v < beginningsIndex.size());
            const auto start = beginningsIndex[v].start;
            const auto end = beginningsIndex[v].end;

            // We may store the paths ancestor ID (may be equal to path ID) so transform path ID first:
            const int ancestorId = mapToBeginningAncestor[pathId];

            const int insertIdx = findInsertIndexInSortedRangeBinary(ancestorId, ConstantVectorRange<int>(
                    pathBeginnings.begin() + start, pathBeginnings.begin() + end));
            const bool result = insertIdx > 0 && pathBeginnings[start + insertIdx - 1] == ancestorId;

            KASSERT(linearRangeContains(pathId, getPathsPossiblyBeginningAt(v)) == result);
            return result;
        }

        // Returns indices of edges in path to the according path beginnings.
        ConstantVectorRange<int> getEdgeIndicesOfPathBeginningsAt(const int v) const {
            KASSERT(v >= 0);
            KASSERT(v < beginningsIndex.size());
            const auto start = beginningsIndex[v].start;
            const auto end = beginningsIndex[v].end;
            return {beginningsEdgeIdx.begin() + start, beginningsEdgeIdx.begin() + end};
        }

        // Returns walking distance for passenger if according path beginning at this vertex is used.
        ConstantVectorRange<int> getWalkingDistsToPathBeginningsAt(const int v) const {
            KASSERT(v >= 0);
            KASSERT(v < beginningsIndex.size());
            const auto start = beginningsIndex[v].start;
            const auto end = beginningsIndex[v].end;
            return {beginningsWalkingDists.begin() + start, beginningsWalkingDists.begin() + end};
        }

        // Returns IDs of paths that may end at vertex v.
        auto getPathsPossiblyEndingAt(const int v) const {
            KASSERT(v >= 0);
            KASSERT(v < endsIndex.size());
            const auto start = endsIndex[v].start;
            const auto end = endsIndex[v].end;
            return std::ranges::subrange(pathEnds.begin() + start, pathEnds.begin() + end) |
                   std::views::transform([&](const int id) {
                       return mapToEndActiveDescendent[id];
                   });
        }

        bool hasPathEndAt(const int v, const int pathId) const {
            KASSERT(v >= 0);
            KASSERT(v < endsIndex.size());
            const auto start = endsIndex[v].start;
            const auto end = endsIndex[v].end;

            // We may store the paths ancestor ID (may be equal to path ID) so transform path ID first:
            const int ancestorId = mapToEndAncestor[pathId];

            const int insertIdx = findInsertIndexInSortedRangeBinary(ancestorId, ConstantVectorRange<int>(
                    pathEnds.begin() + start, pathEnds.begin() + end));
            const bool result = insertIdx > 0 && pathEnds[start + insertIdx - 1] == ancestorId;
            KASSERT(linearRangeContains(pathId, getPathsPossiblyEndingAt(v)) == result);
            return result;
        }

        // Returns reverse indices (from the back of the path) of edges in path to the according path ends.
        ConstantVectorRange<int> getRevEdgeIndicesOfPathEndsAt(const int v) const {
            KASSERT(v >= 0);
            KASSERT(v < endsIndex.size());
            const auto start = endsIndex[v].start;
            const auto end = endsIndex[v].end;
            return {endsEdgeRevIdx.begin() + start, endsEdgeRevIdx.begin() + end};
        }

        // Returns walking distance for passenger if according path end at this vertex is used.
        ConstantVectorRange<int> getWalkingDistsFromPathEndsAt(const int v) const {
            assert(v >= 0);
            assert(v < endsIndex.size());
            const auto start = endsIndex[v].start;
            const auto end = endsIndex[v].end;
            return {endsWalkingDists.begin() + start, endsWalkingDists.begin() + end};
        }

        void addPathBeginningAtVertex(const int pathId, const int idxInPath, const int walkingDist, const int v) {
            assert(v >= 0);
            assert(v < beginningsIndex.size());
            const auto start = beginningsIndex[v].start;
            const auto end = beginningsIndex[v].end;

            // Path IDs are stored sorted by ancestor ID. Newly added paths are their own ancestors, so there is no
            // need to transform the path ID.
            const int insertIdx = findInsertIndexInSortedRangeBinary(pathId, ConstantVectorRange<int>(
                    pathBeginnings.begin() + start, pathBeginnings.begin() + end));

            // Do not add duplicates
            if (insertIdx > 0 && pathBeginnings[start + insertIdx - 1] == pathId)
                return;

            const int valueArrIdx = stableInsertion(v, insertIdx, pathId, beginningsIndex, pathBeginnings,
                                                    beginningsEdgeIdx, beginningsWalkingDists);
            beginningsEdgeIdx[valueArrIdx] = idxInPath;
            beginningsWalkingDists[valueArrIdx] = walkingDist;

            KASSERT(beginningsIndex[v].start >= 0 && beginningsIndex[v].end <= pathBeginnings.size());
            KASSERT(mapToBeginningActiveDescendent.size() == mapToBeginningAncestor.size() &&
                    mapToBeginningActiveDescendent.size() == mapToEndActiveDescendent.size() &&
                    mapToBeginningActiveDescendent.size() == mapToEndAncestor.size());
            if (pathId + 1 > mapToBeginningActiveDescendent.size()) {
                mapToBeginningActiveDescendent.resize(pathId + 1, INVALID_ID);
                mapToBeginningAncestor.resize(pathId + 1, INVALID_ID);
                mapToEndActiveDescendent.resize(pathId + 1, INVALID_ID);
                mapToEndAncestor.resize(pathId + 1, INVALID_ID);
            }

            mapToBeginningActiveDescendent[pathId] = pathId;
            mapToBeginningAncestor[pathId] = pathId;
        }

        // Parameter revIdxInPath specifies the index of the edge e in the path for which head(e) == v counted from the
        // back of the path!
        void addPathEndAtVertex(const int pathId, const int revIdxInPath, const int walkingDist, const int v) {
            assert(v >= 0);
            assert(v < endsIndex.size());
            const auto start = endsIndex[v].start;
            const auto end = endsIndex[v].end;

            // Path IDs are stored sorted by ancestor ID. Newly added paths are their own ancestors, so there is no
            // need to transform the path ID.
            const int insertIdx = findInsertIndexInSortedRangeBinary(pathId, ConstantVectorRange<int>(
                    pathEnds.begin() + start, pathEnds.begin() + end));

            // Do not add duplicates
            if (insertIdx > 0 && pathEnds[start + insertIdx - 1] == pathId)
                return;

            const int valueArrayIdx = stableInsertion(v, insertIdx, pathId, endsIndex, pathEnds,
                                                      endsEdgeRevIdx, endsWalkingDists);
            endsEdgeRevIdx[valueArrayIdx]= revIdxInPath;
            endsWalkingDists[valueArrayIdx] = walkingDist;

            KASSERT(endsIndex[v].start >= 0 && endsIndex[v].end <= pathEnds.size());
            KASSERT(mapToBeginningActiveDescendent.size() == mapToBeginningAncestor.size() &&
                    mapToBeginningActiveDescendent.size() == mapToEndActiveDescendent.size() &&
                    mapToBeginningActiveDescendent.size() == mapToEndAncestor.size());
            if (pathId + 1 > mapToBeginningActiveDescendent.size()) {
                mapToBeginningActiveDescendent.resize(pathId + 1, INVALID_ID);
                mapToBeginningAncestor.resize(pathId + 1, INVALID_ID);
                mapToEndActiveDescendent.resize(pathId + 1, INVALID_ID);
                mapToEndAncestor.resize(pathId + 1, INVALID_ID);
            }

            mapToEndActiveDescendent[pathId] = pathId;
            mapToEndAncestor[pathId] = pathId;
        }


        // If path with ID i is sliced and a subpath [0,k) is inserted as a new path with a new ID j, the possible
        // beginning vertices of path j are the same as the old path i. Thus, we do not replace all occurrences of
        // i in pathBeginnings with j but instead only memorize the mapping.
        // The mapping is applied whenever the beginning IDs are queried.
        void notifyNewPathIdForBeginnings(const int oldPathId, const int newPathId) {
            KASSERT(oldPathId < mapToBeginningActiveDescendent.size());
            KASSERT(mapToBeginningActiveDescendent.size() == mapToBeginningAncestor.size() &&
                    mapToBeginningActiveDescendent.size() == mapToEndActiveDescendent.size() &&
                    mapToBeginningActiveDescendent.size() == mapToEndAncestor.size());
            if (newPathId + 1 > mapToBeginningActiveDescendent.size()) {
                mapToBeginningActiveDescendent.resize(newPathId + 1, INVALID_ID);
                mapToBeginningAncestor.resize(newPathId + 1, INVALID_ID);
                mapToEndActiveDescendent.resize(newPathId + 1, INVALID_ID);
                mapToEndAncestor.resize(newPathId + 1, INVALID_ID);
            }

            KASSERT(mapToBeginningActiveDescendent[oldPathId] == oldPathId ||
                    mapToBeginningActiveDescendent[oldPathId] == INVALID_ID);

            const int ancestorId = mapToBeginningAncestor[oldPathId];
            KASSERT(mapToBeginningActiveDescendent[ancestorId] == oldPathId);
            mapToBeginningAncestor[newPathId] = ancestorId;
            mapToBeginningActiveDescendent[ancestorId] = newPathId;
            mapToBeginningActiveDescendent[newPathId] = INVALID_ID;
        }

        // If path with ID i is sliced and a subpath [k, path.size()) is inserted as a new path with a new ID j, the
        // possible end vertices of path j are the same as the old path i. Thus, we do not replace all occurrences of
        // i in pathEnds with j but instead only memorize the mapping.
        // The mapping is applied whenever the end IDs are queried.
        void notifyNewPathIdForEnds(const int oldPathId, const int newPathId) {
            KASSERT(oldPathId < mapToEndActiveDescendent.size());
            KASSERT(mapToBeginningActiveDescendent.size() == mapToBeginningAncestor.size() &&
                    mapToBeginningActiveDescendent.size() == mapToEndActiveDescendent.size() &&
                    mapToBeginningActiveDescendent.size() == mapToEndAncestor.size());
            if (newPathId + 1 > mapToBeginningActiveDescendent.size()) {
                mapToBeginningActiveDescendent.resize(newPathId + 1, INVALID_ID);
                mapToBeginningAncestor.resize(newPathId + 1, INVALID_ID);
                mapToEndActiveDescendent.resize(newPathId + 1, INVALID_ID);
                mapToEndAncestor.resize(newPathId + 1, INVALID_ID);
            }
            KASSERT(mapToEndActiveDescendent[oldPathId] == oldPathId ||
                    mapToEndActiveDescendent[oldPathId] == INVALID_ID);

            const int ancestorId = mapToEndAncestor[oldPathId];
            KASSERT(mapToEndActiveDescendent[ancestorId] == oldPathId);
            mapToEndAncestor[newPathId] = ancestorId;
            mapToEndActiveDescendent[ancestorId] = newPathId;
            mapToEndActiveDescendent[newPathId] = INVALID_ID;
        }

        // De-fragment 2D-arrays
        void defrag() {
            defragImpl(beginningsIndex, pathBeginnings, beginningsEdgeIdx, beginningsWalkingDists);
            defragImpl(endsIndex, pathEnds, endsEdgeRevIdx, endsWalkingDists);
        }

    private:

        // Assumes range is sorted according to std::less.
        template<typename T, typename SortedRangeT>
        static int findInsertIndexInSortedRangeBinary(const T &x, const SortedRangeT &range) {
            KASSERT(std::is_sorted(range.begin(), range.end(), std::less<>()));
            // Check if range is currently empty or new entry needs to become first element:
            if (range.end() == range.begin() || x < range[0])
                return 0;

            // Check if new entry needs to become last element:
            if (!(x < range[range.size() - 1]))
                return range.size();

            // Binary search with invariant: !(x < range[l]) && x < range[r]
            int l = 0, r = range.size() - 1;
            while (l < r - 1) {
                KASSERT(!(x < range[l]) && x < range[r]);
                int m = (l + r) / 2;
                if (x < range[m]) {
                    r = m;
                } else {
                    l = m;
                }
            }

            return r;
        }

        template<typename T, typename RangeT>
        static bool linearRangeContains(const T& x, const RangeT& range) {
            return contains(range.begin(), range.end(), x);
        }

        void defragImpl(std::vector<ValueBlockPosition> &index,
                        std::vector<int> &pathIds,
                        std::vector<int> &edgeIdx,
                        std::vector<int> &walkingDists) {
            int totalPossiblePaths = 0;
            for (const auto &pos: index)
                totalPossiblePaths += pos.end - pos.start;

            std::vector<ValueBlockPosition> newIndex(index.size());
            std::vector<int> newPathIds;
            std::vector<int> newEdgeIdx;
            std::vector<int> newWalkingDists;
            newPathIds.reserve(totalPossiblePaths);
            newWalkingDists.reserve(totalPossiblePaths);

            for (int v = 0; v < index.size(); ++v) {
                newIndex[v].start = newPathIds.size();
                newPathIds.insert(newPathIds.end(), pathIds.begin() + index[v].start,
                                  pathIds.begin() + index[v].end);
                newEdgeIdx.insert(newEdgeIdx.end(), edgeIdx.begin() + index[v].start,
                                  edgeIdx.begin() + index[v].end);
                newWalkingDists.insert(newWalkingDists.end(), walkingDists.begin() + index[v].start,
                                       walkingDists.begin() + index[v].end);
                newIndex[v].end = newPathIds.size();
            }
            index = std::move(newIndex);
            pathIds = std::move(newPathIds);
            edgeIdx = std::move(newEdgeIdx);
            walkingDists = std::move(newWalkingDists);
        }

        // Returns first smallest vertex ID for which either beginnings or ends are not sorted or INVALID_VERTEX if
        // all are sorted.
        int getSmallestVertexIdWithUnsortedRange() {
            KASSERT(beginningsIndex.size() == endsIndex.size());
            for (int v = 0; v < beginningsIndex.size(); ++v) {
                if (!verifyBeginningsSorted(v) || !verifyEndsSorted(v))
                    return v;
            }
            return INVALID_VERTEX;
        }

        bool verifyBeginningsSorted(const int v) const {
            return std::is_sorted(pathBeginnings.begin() + beginningsIndex[v].start,
                                  pathBeginnings.begin() + beginningsIndex[v].end);
        }

        bool verifyEndsSorted(const int v) const {
            return std::is_sorted(pathEnds.begin() + endsIndex[v].start, pathEnds.begin() + endsIndex[v].end);
        }

        // We use two DynamicRagged2DArray to store the subset of paths that may begin/end at each vertex.
        // For both 2D-arrays, there is a single index array that for each vertex stores a range of indices where
        // the entries for that vertex are stored in the according value arrays.

        // Path Beginnings Index Array: For each vertex (of the vehicle graph) v, the according entries in the
        // beginnings value arrays lie in the index interval [beginningsIndex[v].start, beginningsIndex[v].end).
        std::vector<ValueBlockPosition> beginningsIndex;

        // IDs of paths that may start at vertex. If walking is allowed, a path may start at any possible pickup vertex.
        std::vector<int> pathBeginnings;

        // x := beginningsEdgeIdx[beginningsIndex[v].start + i] describes the index of the edge after v in the path p with
        // ID pathBeginnings[beginningsIndex[v].start + i], i.e. inputGraph.edgeTail(p[x]) == v.
        std::vector<int> beginningsEdgeIdx;

        // Walking distance for passenger if path beginning at this vertex is used.
        std::vector<int> beginningsWalkingDists;

        // Path Ends Index Array: For each vertex (of the vehicle graph) v, the according entries in the ends value
        // arrays lie in the index interval [endsIndex[v].start, endsIndex[v].end).
        std::vector<ValueBlockPosition> endsIndex;

        // IDs of paths that may end at vertex. If walking is allowed, a path may end at any possible dropoff vertex.
        std::vector<int> pathEnds;

        // x := endsEdgeRevIdx[endsIndex[v].start + i] describes the index of the edge before v in the path p with
        // ID pathEnds[endsIndex[v].start + i] _counted from the back (Rev)_, i.e. inputGraph.edgeHead(p[p.size() - 1 - x]) == v.
        // Reverse indices are used as they do not change when slicing out subpaths at the end of a path.
        std::vector<int> endsEdgeRevIdx;

        // Walking distance for passenger if path end at this vertex is used.
        std::vector<int> endsWalkingDists;


        // If a path is sliced into subpaths, the beginnings/ends are the same as the original path's for the subpaths
        // at the front and back of the original path, respectively. Instead of replacing the path IDs in
        // pathBeginnings and pathEnds, we only store a mapping and apply the mapping upon query.
        std::vector<int> mapToBeginningActiveDescendent;
        std::vector<int> mapToBeginningAncestor;
        std::vector<int> mapToEndActiveDescendent;
        std::vector<int> mapToEndAncestor;

    };
}

