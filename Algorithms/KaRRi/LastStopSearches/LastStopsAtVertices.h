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

#include <cassert>
#include <vector>
#include "Tools/Workarounds.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "DataStructures/Utilities/IteratorRange.h"

namespace karri {


    class LastStopsAtVertices {

        using VehiclesOrderedByLastStop = std::vector<int>;
        using VehiclesOrderedByLastStopIt = VehiclesOrderedByLastStop::const_iterator;

    public:

        LastStopsAtVertices(const int numVertices, const int fleetSize)
                : numVertices(numVertices), fleetSize(fleetSize), firstLastStopAtVertex(numVertices),
                  vehiclesOrderedByLastStop() {
            unused(this->numVertices, this->fleetSize); // only used for sanity checks in asserts
        }

        bool isAnyLastStopAtVertex(const int vertex) const {
            assert(vertex >= 0 && vertex < numVertices);
            return firstLastStopAtVertex[vertex].end != firstLastStopAtVertex[vertex].start;
        }

        int numLastStopsAtVertex(const int vertex) const {
            assert(vertex >= 0 && vertex < numVertices);
            const auto range = firstLastStopAtVertex[vertex];
            return range.end - range.start;
        }

        ConstantVectorRange<int> vehiclesWithLastStopAt(const int vertex) const {
            assert(vertex >= 0 && vertex < numVertices);
            const auto range = firstLastStopAtVertex[vertex];
            return {vehiclesOrderedByLastStop.begin() + range.start,
                    vehiclesOrderedByLastStop.begin() + range.end};
        }

        // Inserts the last stop of vehicle with given ID at vertex in the data structure.
        void insertLastStopAt(const int vertex, const int vehId) {
            assert(vertex >= 0 && vertex < numVertices);
            assert(vehId >= 0 && vehId < fleetSize);
            insertion(vertex, vehId, firstLastStopAtVertex, vehiclesOrderedByLastStop);
        }

        // Removes the last stop of vehicle with given ID at vertex from the data structure.
        void removeLastStopAt(const int vertex, const int vehId) {
            assert(vertex >= 0 && vertex < numVertices);
            assert(vehId >= 0 && vehId < fleetSize);
            const auto start = firstLastStopAtVertex[vertex].start;
            const auto end = firstLastStopAtVertex[vertex].end;
            assert(end > start);
            for (auto i = start; i < end; ++i)
                if (vehiclesOrderedByLastStop[i] == vehId) {
                    removal(vertex, i - start, firstLastStopAtVertex, vehiclesOrderedByLastStop);
                    break;
                }
            assert(firstLastStopAtVertex[vertex].end - firstLastStopAtVertex[vertex].start == end - start - 1);
        }

    private:

        const int numVertices;
        const int fleetSize;

        std::vector<ValueBlockPosition> firstLastStopAtVertex;
        std::vector<int> vehiclesOrderedByLastStop;
    };
}