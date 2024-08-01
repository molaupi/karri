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

#include "Tools/Constants.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"
#include <vector>

namespace mixfix {

    class PreliminaryPaths {
        using EdgeIt = typename std::vector<int>::const_iterator;
        using RevEdgeIt = typename std::vector<int>::const_reverse_iterator;

    public:

        struct Path {

            EdgeIt begin() const { return allPathEdges->begin() + startOffset; }

            EdgeIt end() const { return allPathEdges->begin() + endOffset; }

            EdgeIt cbegin() const { return allPathEdges->cbegin() + startOffset; }

            EdgeIt cend() const { return allPathEdges->cbegin() + endOffset; }

            RevEdgeIt rbegin() const {
                return allPathEdges->rbegin()+ (allPathEdges->size() - endOffset);
            }

            RevEdgeIt rend() const {
                return allPathEdges->rbegin() + (allPathEdges->size() - startOffset);
            }

            const int &operator[](const int idx) const {
                KASSERT(idx < size());
                return (*allPathEdges)[startOffset + idx];
            }

            const int &front() const {
                return (*allPathEdges)[startOffset];
            }

            const int &back() const {
                return (*allPathEdges)[endOffset - 1];
            }

            size_t size() const {
                return static_cast<size_t>(endOffset - startOffset);
            }

            const int &getPathId() const {
                return pathId;
            }

            const int &getAncestorId() const {
                return ancestorPathId;
            }

        private:

            friend PreliminaryPaths;

            Path(const int pathId, const int ancestorPathId, const int startOffset, const int endOffset,
                 std::vector<int> const *allPathEdges)
                    : pathId(pathId), ancestorPathId(ancestorPathId), startOffset(startOffset), endOffset(endOffset),
                      allPathEdges(allPathEdges) {
                KASSERT(endOffset > startOffset);
            }

            int pathId = INVALID_ID;
            int ancestorPathId = INVALID_ID; // ID of initial path that this is a subpath of
            int startOffset;
            int endOffset;
            std::vector<int> const *allPathEdges;
        };

    private:

        using Paths = std::vector<Path>;
        using PathIt = typename Paths::const_iterator;

    public:

        PreliminaryPaths() {}

        void init(const int numInitialPaths) {
            pathIdToIdx.resize(numInitialPaths);
            std::fill(pathIdToIdx.begin(), pathIdToIdx.end(), INVALID_INDEX);
            paths.clear();
        }

        void addInitialPath(const int pathId, std::vector<int> &&edges) {
            const int startOffset = allPathEdges.size();
            const int endOffset = startOffset + edges.size();
            allPathEdges.insert(allPathEdges.end(), edges.begin(), edges.end());
            pathIdToIdx[pathId] = paths.size();
            // Initial paths are their own ancestors.
            paths.push_back(Path(pathId, pathId, startOffset, endOffset, &allPathEdges));
        }

        int getMaxPathId() const {
            return pathIdToIdx.size() - 1;
        }

        int numPaths() const {
            return paths.size();
        }

        bool empty() const {
            return paths.empty();
        }

        bool hasPathFor(const int pathId) const {
            KASSERT(pathId >= 0 && pathId < pathIdToIdx.size());
            const int &idx = pathIdToIdx[pathId];
            KASSERT(idx < static_cast<int>(paths.size()));
            return idx != INVALID_INDEX;
        }

        const Path &getPathFor(const int pathId) const {
            KASSERT(pathId >= 0 && pathId < pathIdToIdx.size());
            KASSERT(hasPathFor(pathId),
                    "No path with ID " << pathId << ".", kassert::assert::light);
            return paths[pathIdToIdx[pathId]];
        }

        void removePath(const int pathId) {
            KASSERT(pathId >= 0 && pathId < pathIdToIdx.size());
            KASSERT(hasPathFor(pathId),
                    "No path with ID " << pathId << ".", kassert::assert::light);
            const int idx = pathIdToIdx[pathId];
            pathIdToIdx[pathId] = INVALID_INDEX;
            if (idx == static_cast<int>(paths.size() - 1)) {
                paths.pop_back();
                return;
            }

            paths[idx] = paths.back();
            paths.pop_back();
            pathIdToIdx[paths[idx].getPathId()] = idx;
        }

        // Slice out subpath with edge indices [start, end) from middle of path.
        // Removes path and inserts up to two new paths representing the remaining subpaths [0, start) and
        // [end, path.size()).
        // Returns IDs of new subpath at beginning and end of old path (INVALID_ID if no subpath introduced).
        std::pair<int, int> sliceOutSubpath(const int pathId, const int start, const int end) {
            KASSERT(pathId >= 0 && pathId < pathIdToIdx.size());
            KASSERT(hasPathFor(pathId),
                    "No path with ID " << pathId << ".", kassert::assert::light);

            const auto oldPath = getPathFor(pathId);
            KASSERT(0 <= start && start < end);
            KASSERT(end <= oldPath.size());

            removePath(pathId);
            int begId = INVALID_ID;
            int endId = INVALID_ID;
            // Add subpath at beginning (if any remaining)
            if (start > 0) {
                begId = pathIdToIdx.size();
                pathIdToIdx.push_back(paths.size());
                paths.push_back(Path(begId, oldPath.ancestorPathId, oldPath.startOffset, oldPath.startOffset + start,
                                     &allPathEdges));
            }

            // Add subpath at end (if any remaining)
            if (end < oldPath.size()) {
                endId = pathIdToIdx.size();
                pathIdToIdx.push_back(paths.size());
                paths.push_back(Path(endId, oldPath.ancestorPathId, oldPath.startOffset + end, oldPath.endOffset,
                                     &allPathEdges));
            }

            return {begId, endId};
        }

        PathIt begin() const {
            return paths.cbegin();
        }

        PathIt end() const {
            return paths.cend();
        }

    private:

        std::vector<int> pathIdToIdx;
        std::vector<Path> paths;
        std::vector<int> allPathEdges;


    };


} // end namespace