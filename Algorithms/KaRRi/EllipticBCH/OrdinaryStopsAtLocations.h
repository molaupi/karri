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

#include <vector>
#include "DataStructures/Utilities/IteratorRange.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"

namespace karri {

    class OrdinaryStopsAtLocations {

    public:

        OrdinaryStopsAtLocations(const int numLocations) : pos(numLocations, {0, 0}) {}

        // loc is expected to be an edge ID
        void addStopAtLocation(const int stopId, const int loc) {
            dynamic_ragged2d::insertion(loc, stopId, pos, stopIds);
        }

        void removeStopAtLocation(const int stopId, const int loc) {
            int idx = 0;
            const auto start = pos[loc].start;
            const auto end = pos[loc].end;
            int i = 0;
            for (; i < end - start; ++i) {
                if (stopIds[start + i] == stopId) {
                    idx = i;
                    break;
                }
            }
            if (i == end - start)
                return; // stopId not found
            dynamic_ragged2d::removal(loc, idx, pos, stopIds);
        }

        ConstantVectorRange<int> getStopsAtLocation(const int loc) const {
            return {stopIds.begin() + pos[loc].start,
                    stopIds.begin() + pos[loc].end};
        }


    private:

        std::vector<dynamic_ragged2d::ValueBlockPosition> pos;
        std::vector<int> stopIds;

    };

}