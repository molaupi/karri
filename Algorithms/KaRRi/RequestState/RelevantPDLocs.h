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

#include "Tools/Simd/AlignedVector.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

#include "DataStructures/Containers/Parallel/ThreadSafeSubset.h"
#include "tbb/concurrent_vector.h"
#include "RelevantPDLoc.h"


namespace karri {


    struct RelevantPDLocs {

        template<typename, typename, typename> friend
        class RelevantPDLocsReorderer;

        using RelevantPDLocVector = AlignedVector<RelevantPDLoc>;

    public:

        using It = typename RelevantPDLocVector::const_iterator;
        using RevIt = typename RelevantPDLocVector::const_reverse_iterator;

        explicit RelevantPDLocs(const int fleetSize)
                : fleetSize(fleetSize),
                  startOfRelevantPDLocs(fleetSize + 1),
                  relevantSpots(),
                  vehiclesWithRelevantSpots(fleetSize) {}

        const Subset &getVehiclesWithRelevantPDLocs() const {
            return vehiclesWithRelevantSpots;
        }

        bool hasRelevantSpotsFor(const int vehId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            return startOfRelevantPDLocs[vehId] != startOfRelevantPDLocs[vehId + 1];
        }

        IteratorRange<It> relevantSpotsFor(const int vehId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            return {relevantSpots.begin() + startOfRelevantPDLocs[vehId],
                    relevantSpots.begin() + startOfRelevantPDLocs[vehId + 1]};
        }

        IteratorRange<RevIt> relevantSpotsForInReverseOrder(const int vehId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            const int rstart = relevantSpots.size() - startOfRelevantPDLocs[vehId + 1];
            const int rend = relevantSpots.size() - startOfRelevantPDLocs[vehId];
            return {relevantSpots.rbegin() + rstart, relevantSpots.rbegin() + rend};
        }

    private:

        int fleetSize;
        std::vector<int> startOfRelevantPDLocs;
        RelevantPDLocVector relevantSpots;
        Subset vehiclesWithRelevantSpots;

    };
}