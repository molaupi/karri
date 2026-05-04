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

#include "Tools/Simd/AlignedVector.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"



namespace karri {

    // Forward declarations for friend
    namespace Transfers {
        template<typename, typename, typename, typename>
        class TransfersDropoffALSBCHStrategy;
    }

    // Information on all vehicles that can perform a dropoff of the current request after their last stop.
    // For each such vehicle, contains all dropoffs and the respective distance from the last stop to the dropoff.
    // Interface is structured similarly to RelevantPDLocs which is used for ordinary pickup and dropoff insertions.
    // Used in OrdinaryTransferFinder for the dropoff vehicles that can potentially perform transfers before the
    // last stop and a dropoff after.
    struct RelevantDropoffsAfterLastStop {

        // All TransferDropoffALSStrategies are friends and can construct this information.

        template<typename, typename, typename, typename>
        friend
        class Transfers::TransfersDropoffALSBCHStrategy;

        struct RelevantDropoff {
            unsigned int dropoffId = INVALID_ID;
            int distToDropoff = INFTY;
        };

        using RelevantDropoffVector = std::vector<RelevantDropoff>;

    public:

        using It = typename RelevantDropoffVector::const_iterator;
        using RevIt = typename RelevantDropoffVector::const_reverse_iterator;

        RelevantDropoffsAfterLastStop(const int fleetSize)
                : fleetSize(fleetSize),
                  relevantSpots(),
                  vehiclesWithRelevantSpots() {}

        const std::vector<int> &getVehiclesWithRelevantPDLocs() const {
            return vehiclesWithRelevantSpots;
        }

        bool hasRelevantSpotsFor(const int vehId) const {
            KASSERT(vehId >= 0 && vehId < fleetSize);
            return vehicleToPdLocs.contains(vehId);
//            return startOfRelevantPDLocs[vehId] != startOfRelevantPDLocs[vehId + 1];
        }

        IteratorRange<It> relevantSpotsFor(const int vehId) const {
            KASSERT(vehId >= 0 && vehId < fleetSize);
            if (!hasRelevantSpotsFor(vehId))
                return {relevantSpots.end(), relevantSpots.end()};
            const auto range = vehicleToPdLocs.at(vehId);
            return {relevantSpots.begin() + range.start,
                    relevantSpots.begin() + range.end};
        }

        IteratorRange<RevIt> relevantSpotsForInReverseOrder(const int vehId) const {
            KASSERT(vehId >= 0 && vehId < fleetSize);
            if (!hasRelevantSpotsFor(vehId))
                return {relevantSpots.rend(), relevantSpots.rend()};
            const auto range = vehicleToPdLocs.at(vehId);
            const int rstart = relevantSpots.size() - range.end;
            const int rend = relevantSpots.size() - range.start;
            return {relevantSpots.rbegin() + rstart, relevantSpots.rbegin() + rend};
        }

    private:

        const int fleetSize;
        RelevantDropoffVector relevantSpots;

        struct RelevantPDLocsRange {
            int start = INVALID_INDEX;
            int end = INVALID_INDEX;
        };
        std::unordered_map<int, RelevantPDLocsRange> vehicleToPdLocs;
        std::vector<int> vehiclesWithRelevantSpots;

    };
}