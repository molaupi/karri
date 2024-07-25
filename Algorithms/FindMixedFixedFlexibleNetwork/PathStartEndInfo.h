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

namespace mixfix {


    class PathStartEndInfo {

    public:
        PathStartEndInfo(const int numVertices)
                : beginningsIndex(numVertices, {0, 0}),
                  pathBeginnings(),
                  beginningsWalkingDists(),
                  endsIndex(numVertices, {0, 0}),
                  pathEnds(),
                  endsWalkingDists() {}

        PathStartEndInfo(std::ifstream &in) {
            bio::read(in, beginningsIndex);
            bio::read(in, pathBeginnings);
            bio::read(in, beginningsWalkingDists);
            bio::read(in, endsIndex);
            bio::read(in, pathEnds);
            bio::read(in, endsWalkingDists);

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
            bio::write(out, beginningsIndex);
            bio::write(out, pathBeginnings);
            bio::write(out, beginningsWalkingDists);
            bio::write(out, endsIndex);
            bio::write(out, pathEnds);
            bio::write(out, endsWalkingDists);
        }


        // Returns IDs of paths that may start at vertex v.
        std::vector<int> getPathsPossiblyBeginningAt(const int v) const {
            assert(v >= 0);
            assert(v < beginningsIndex.size());
            const auto start = beginningsIndex[v].start;
            const auto end = beginningsIndex[v].end;
            // TODO: This should use a transform view using ranges if C++20 (or using the range-v3 library)
            std::vector<int> result(pathBeginnings.begin() + start, pathBeginnings.begin() + end);
            std::transform(result.cbegin(), result.cend(), result.begin(), [&](const int id) {
                return mapToBeginningActiveDescendent[id];
            });
            return result;
        }

        // Returns walking distance for passenger if according path beginning at this vertex is used.
        ConstantVectorRange<int> getWalkingDistsToPathBeginningsAt(const int v) const {
            assert(v >= 0);
            assert(v < beginningsIndex.size());
            const auto start = beginningsIndex[v].start;
            const auto end = beginningsIndex[v].end;
            return {beginningsWalkingDists.begin() + start, beginningsWalkingDists.begin() + end};
        }

        // Returns IDs of paths that may end at vertex v.
        std::vector<int> getPathsPossiblyEndingAt(const int v) const {
            assert(v >= 0);
            assert(v < endsIndex.size());
            const auto start = endsIndex[v].start;
            const auto end = endsIndex[v].end;
            // TODO: This should use a transform view using ranges if C++20 (or using the range-v3 library)
            std::vector<int> result(pathEnds.begin() + start, pathEnds.begin() + end);
            std::transform(result.cbegin(), result.cend(), result.begin(), [&](const int id) {
                return mapToEndActiveDescendent[id];
            });
            return result;
        }

        // Returns walking distance for passenger if according path end at this vertex is used.
        ConstantVectorRange<int> getWalkingDistsFromPathEndsAt(const int v) const {
            assert(v >= 0);
            assert(v < endsIndex.size());
            const auto start = endsIndex[v].start;
            const auto end = endsIndex[v].end;
            return {endsWalkingDists.begin() + start, endsWalkingDists.begin() + end};
        }

        void addPathBeginningAtVertex(const int pathId, const int walkingDist, const int v) {
            assert(v >= 0);
            assert(v < beginningsIndex.size());
            const auto start = beginningsIndex[v].start;
            const auto end = beginningsIndex[v].end;

            // TODO: sort value arrays?
            // Do not add duplicates
            if (contains(pathBeginnings.begin() + start, pathBeginnings.begin() + end, pathId))
                return;

            const int idx = insertion(v, pathId, beginningsIndex, pathBeginnings, beginningsWalkingDists);
            beginningsWalkingDists[idx] = walkingDist;

            KASSERT(beginningsIndex[v].start >= 0 && beginningsIndex[v].end <= pathBeginnings.size());

            mapToBeginningActiveDescendent[pathId] = pathId;
            mapToBeginningAncestor[pathId] = pathId;
        }

        void addPathEndAtVertex(const int pathId, const int walkingDist, const int v) {
            assert(v >= 0);
            assert(v < endsIndex.size());
            const auto start = endsIndex[v].start;
            const auto end = endsIndex[v].end;

            // TODO: sort value arrays?
            // Do not add duplicates
            if (contains(pathEnds.begin() + start, pathEnds.begin() + end, pathId))
                return;

            const int idx = insertion(v, pathId, endsIndex, pathEnds, endsWalkingDists);
            endsWalkingDists[idx] = walkingDist;

            KASSERT(endsIndex[v].start >= 0 && endsIndex[v].end <= pathEnds.size());

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
            defragImpl(beginningsIndex, pathBeginnings, beginningsWalkingDists);
            defragImpl(endsIndex, pathEnds, endsWalkingDists);
        }

    private:

        void defragImpl(std::vector<ValueBlockPosition> &index,
                        std::vector<int> &pathIds,
                        std::vector<int> &walkingDists) {
            int totalPossiblePaths = 0;
            for (const auto &pos: index)
                totalPossiblePaths += pos.end - pos.start;

            std::vector<ValueBlockPosition> newIndex(index.size());
            std::vector<int> newPathIds;
            std::vector<int> newWalkingDists;
            newPathIds.reserve(totalPossiblePaths);
            newWalkingDists.reserve(totalPossiblePaths);

            for (int e = 0; e < index.size(); ++e) {
                newIndex[e].start = newPathIds.size();
                newPathIds.insert(newPathIds.end(), pathIds.begin() + index[e].start,
                                  pathIds.begin() + index[e].end);
                newWalkingDists.insert(newWalkingDists.end(), walkingDists.begin() + index[e].start,
                                       walkingDists.begin() + index[e].end);
                newIndex[e].end = newPathIds.size();
            }
            index = std::move(newIndex);
            pathIds = std::move(newPathIds);
            walkingDists = std::move(newWalkingDists);
        }

        // We use two DynamicRagged2DArray to store the subset of paths that may begin/end at each vertex.
        // For both 2D-arrays, there is a single index array that for each vertex stores a range of indices where
        // the entries for that vertex are stored in the according value arrays.

        // Path Beginnings Index Array: For each vertex (of the vehicle graph) v, the according entries in the
        // beginnings value arrays lie in the index interval [beginningsIndex[v].start, beginningsIndex[v].end).
        std::vector<ValueBlockPosition> beginningsIndex;

        // IDs of paths that may start at vertex. If walking is allowed, a path may start at any possible pickup vertex.
        std::vector<int> pathBeginnings;

        // Walking distance for passenger if path beginning at this vertex is used.
        std::vector<int> beginningsWalkingDists;

        // Path Ends Index Array: For each vertex (of the vehicle graph) v, the according entries in the ends value
        // arrays lie in the index interval [endsIndex[v].start, endsIndex[v].end).
        std::vector<ValueBlockPosition> endsIndex;

        // IDs of paths that may end at vertex. If walking is allowed, a path may end at any possible dropoff vertex.
        std::vector<int> pathEnds;

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

