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

class OverlappingPaths {

public:

    struct Overlap {

        Overlap() : start(INVALID_INDEX), end(INVALID_INDEX), reachesBeginningOfPath(false), reachesEndOfPath(false) {}

        Overlap(const int start, const int end, const bool reachesBeginningOfPath, const bool reachesEndOfPath)
                : start(start), end(end), reachesBeginningOfPath(reachesBeginningOfPath),
                  reachesEndOfPath(reachesEndOfPath) {}

        void reset() {
            start = INVALID_INDEX;
            end = INVALID_INDEX;
            reachesBeginningOfPath = false;
            reachesEndOfPath = false;
        }

        int start; // smallest known index in path where path overlaps with line
        int end; // (one past) largest known index in path where path overlaps with line

        bool reachesBeginningOfPath;
        bool reachesEndOfPath;
    };

public:

    void init(const int maxPathId) {
        if (maxPathId >= overlaps.size())
            overlaps.resize(maxPathId + 1, Overlap());
    }

    void initializeOverlap(const int pathId, const int start, const int end, const bool reachesBeginningOfPath,
                           const bool reachesEndOfPath) {
        KASSERT(pathId >= 0 && pathId < overlaps.size());
        overlaps[pathId].start = start;
        overlaps[pathId].end = end;
        overlaps[pathId].reachesBeginningOfPath = reachesBeginningOfPath;
        overlaps[pathId].reachesEndOfPath = reachesEndOfPath;
    }

    bool hasKnownOverlap(const int pathId) const {
        KASSERT(pathId >= 0 && pathId < overlaps.size());
        return overlaps[pathId].start != INVALID_INDEX;
    }

    const Overlap &getOverlapFor(const int pathId) const {
        KASSERT(pathId >= 0 && pathId < overlaps.size());
        KASSERT(hasKnownOverlap(pathId));
        return overlaps[pathId];
    }

    Overlap &getOverlapFor(const int pathId) {
        KASSERT(pathId >= 0 && pathId < overlaps.size());
        return overlaps[pathId];
    }


private:

    // Every path has at most one overlap across all lines (i.e. during the entire execution time) as the first
    // overlap causes a path to be split into subpaths that are considered to be self-contained paths with separate IDs.
    // Thus, we store at most one Overlap per path ID.
    std::vector<Overlap> overlaps;


};