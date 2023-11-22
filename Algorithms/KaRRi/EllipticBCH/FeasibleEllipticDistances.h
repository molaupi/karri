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

#include <type_traits>
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "Tools/Simd/AlignedVector.h"
#include "DataStructures/Containers/Subset.h"
#include "DataStructures/Containers/ThreadSafeSubset.h"

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

#include <atomic>

#define UNUSED(x) (void)(x)

namespace karri {


    template<typename LabelSetT>
    class FeasibleEllipticDistances {


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        using DistsVector = AlignedVector<DistanceLabel>;
        using MeetingVerticesVector = AlignedVector<DistanceLabel>;

    public:

        explicit FeasibleEllipticDistances(const int fleetSize, const RouteState &routeState)
                : routeState(routeState),
                  maxStopId(routeState.getMaxStopId()),
                //   startOfRangeInValueArray(fleetSize),
                  vehiclesWithRelevantPDLocs(fleetSize),
                  minDistToPDLoc(fleetSize),
                  minDistFromPDLocToNextStop(fleetSize) {}

        template<typename PDLocsAtExistingStopsT, typename InputGraphT>
        void init(const int newNumPDLocs, const PDLocsAtExistingStopsT &pdLocsAtExistingStops,
                  const InputGraphT &inputGraph) {
            numLabelsPerStop = newNumPDLocs / K + (newNumPDLocs % K != 0);

            distToRelevantPDLocs.clear();
            distFromRelevantPDLocsToNextStop.clear();
            meetingVerticesToRelevantPDLocs.clear();
            meetingVerticesFromRelevantPDLocsToNextStop.clear();
            distToRelevantPDLocs.clear();
            distFromRelevantPDLocsToNextStop.clear();
            meetingVerticesToRelevantPDLocs.clear();
            meetingVerticesFromRelevantPDLocsToNextStop.clear();

            // Static Allocation for all distance vectors
            distToRelevantPDLocs.resize(numLabelsPerStop * (maxStopId + 1), DistanceLabel(INFTY));
            distFromRelevantPDLocsToNextStop.resize(numLabelsPerStop * (maxStopId + 1), DistanceLabel(INFTY));
            meetingVerticesToRelevantPDLocs.resize(numLabelsPerStop * (maxStopId + 1), DistanceLabel(INVALID_VERTEX));
            meetingVerticesFromRelevantPDLocsToNextStop.resize(numLabelsPerStop * (maxStopId + 1), DistanceLabel(INVALID_VERTEX));

            // resize array for min distances to PD locations
            if (minDistToPDLoc.size() < maxStopId + 1) {
                minDistToPDLoc.clear();
                minDistToPDLoc = std::vector<std::atomic_int>(maxStopId + 1);
            }

            // resize array for min distances from PD locations
            if (minDistFromPDLocToNextStop.size() < maxStopId + 1) {
                minDistFromPDLocToNextStop.clear();
                minDistFromPDLocToNextStop = std::vector<std::atomic_int>(maxStopId + 1);
            }

            // fill both arrays with INFTY 
            for (int j = 0; j <= maxStopId; j++) {
                minDistToPDLoc[j].store(INFTY);
                minDistFromPDLocToNextStop[j].store(INFTY);
            }

            // for (int i = 0; i < numLabelsPerStop * (maxStopId + 1); i++) {
            //     distToRelevantPDLocs[i] = DistanceLabel(INFTY);
            //     distFromRelevantPDLocsToNextStop[i] = DistanceLabel(INFTY);
            //     meetingVerticesToRelevantPDLocs[i] = DistanceLabel(INVALID_VERTEX);
            //     meetingVerticesFromRelevantPDLocsToNextStop[i] = DistanceLabel(INVALID_VERTEX);
            // }

            // if (maxStopId >= startOfRangeInValueArray.size()) {
            //     startOfRangeInValueArray.resize(maxStopId + 1);
            //     minDistToPDLoc.resize(maxStopId + 1);
            //     minDistFromPDLocToNextStop.resize(maxStopId + 1);
            // }

            // for (auto &idx: startOfRangeInValueArray)
            //     idx = INVALID_INDEX;
            vehiclesWithRelevantPDLocs.clear();

            // Pre-allocate entries for PD locs at existing stops. The distance 0 may otherwise not be found by the
            // BCH searches. Also, this way, the distance for such a PD loc never has to be updated, and we already
            // allocate the entry array for this stop, which is good since it will likely also be reachable by other PD locs.
            for (const auto &pdLocAtExistingStop: pdLocsAtExistingStops) {
                const auto &vehId = pdLocAtExistingStop.vehId;
                assert(pdLocAtExistingStop.stopIndex < routeState.numStopsOf(vehId));
                const auto &stopId = routeState.stopIdsFor(vehId)[pdLocAtExistingStop.stopIndex];
                const auto &stopVertex = inputGraph.edgeHead(
                        routeState.stopLocationsFor(vehId)[pdLocAtExistingStop.stopIndex]);
                // allocateEntriesFor(stopId);
                vehiclesWithRelevantPDLocs.insert(routeState.vehicleIdOf(stopId));

                DistanceLabel zeroLabel = INFTY;
                zeroLabel[pdLocAtExistingStop.pdId % K] = 0;
                const auto firstIdInBatch = (pdLocAtExistingStop.pdId / K) * K;
                updateDistanceFromStopToPDLoc(stopId, firstIdInBatch, zeroLabel, stopVertex);

                const auto lengthOfLegStartingHere = time_utils::calcLengthOfLegStartingAt(
                        pdLocAtExistingStop.stopIndex, pdLocAtExistingStop.vehId, routeState);
                DistanceLabel lengthOfLegLabel = INFTY;
                lengthOfLegLabel[pdLocAtExistingStop.pdId % K] = lengthOfLegStartingHere;
                updateDistanceFromPDLocToNextStop(stopId, firstIdInBatch, lengthOfLegLabel, stopVertex);
            }
        }

        // Allocate entries for the given stop if none exist already.
        // would not occur in case of static allocation.
        void preallocateEntriesFor(const int stopId) {
            UNUSED(stopId);
            // if (!hasPotentiallyRelevantPDLocs(stopId))
            //     allocateEntriesFor(stopId);
        }

        // Updates the distance from stop to the PD loc. Distance is written if there are
        // entries for the stop already or dynamic allocation of entries is allowed.
        // Returns mask indicating where the distance has been improved (all false if we don't know the stop and dynamic
        // allocation is not allowed).
        LabelMask updateDistanceFromStopToPDLoc(const int stopId, const unsigned int firstPDLocId,
                                                const DistanceLabel newDistToPDLoc, const int meetingVertex) {
            assert(stopId >= 0 && stopId <= maxStopId);
            assert(firstPDLocId < numLabelsPerStop * K);
            assert(firstPDLocId % K == 0);
            assert(newDistToPDLoc.horizontalMin() >= 0 && newDistToPDLoc.horizontalMin() < INFTY);

            // If no entries exist yet for this stop, perform the allocation. 
            // would not occur in case of static allocation.

            // if (startOfRangeInValueArray[stopId] == INVALID_INDEX) {
            //     allocateEntriesFor(stopId);
            // }

            // Write values for new entry and set pointer from PD loc to the entries
            const auto idx = (stopId * numLabelsPerStop) + (firstPDLocId / K);
            const LabelMask improved = newDistToPDLoc < distToRelevantPDLocs[idx];
            distToRelevantPDLocs[idx].setIf(newDistToPDLoc, improved);
            meetingVerticesToRelevantPDLocs[idx].setIf(meetingVertex, improved);

            const int minNewDistToPDLoc = newDistToPDLoc.horizontalMin();

            auto& minToPDLocAtomic = minDistToPDLoc[stopId];
            int expectedMinForStop = minToPDLocAtomic.load(std::memory_order_relaxed);
            while(expectedMinForStop > minNewDistToPDLoc && !minToPDLocAtomic.compare_exchange_strong(expectedMinForStop, minNewDistToPDLoc, std::memory_order_relaxed));

            return improved;
        }

        // Updates the distance from the PD loc to the stop that follows stopId. Distance is written only if entries
        // for the stop exist already.
        // Returns mask indicating where the distance has been improved (all false if we don't know the stop).
        LabelMask updateDistanceFromPDLocToNextStop(const int stopId, const int firstPDLocId,
                                                    const DistanceLabel newDistFromPDLocToNextStop,
                                                    const int meetingVertex) {

            // We assume the from-searches are run after the to-searches. If the stop does not have entries yet, it was
            // considered irrelevant for the to-searches (regardless of whether we allow dynamic allocation or not).
            // Therefore, this stop cannot be relevant on both sides which means we can skip it here.
            const auto idx = (stopId * numLabelsPerStop) + (firstPDLocId / K);
            if (allSet(distToRelevantPDLocs[idx] == DistanceLabel(INFTY)))
                return LabelMask(false);

            const LabelMask improved = newDistFromPDLocToNextStop < distFromRelevantPDLocsToNextStop[idx];
            distFromRelevantPDLocsToNextStop[idx].setIf(newDistFromPDLocToNextStop, improved);
            meetingVerticesFromRelevantPDLocsToNextStop[idx].setIf(meetingVertex, improved);

            const int minNewDistFromPDLocToNextStop = newDistFromPDLocToNextStop.horizontalMin();

            auto& minFromPDLocAtomic = minDistFromPDLocToNextStop[stopId];
            int expectedMinForStop = minFromPDLocAtomic.load(std::memory_order_relaxed);
            while(expectedMinForStop > minNewDistFromPDLocToNextStop && !minFromPDLocAtomic.compare_exchange_strong(expectedMinForStop, minNewDistFromPDLocToNextStop, std::memory_order_relaxed));

            return improved;
        }

        bool hasPotentiallyRelevantPDLocs(const int stopId) const {
            assert(stopId <= maxStopId);
            const auto startIdxForStop = stopId * numLabelsPerStop;
            const auto endIdxForStop = (stopId + 1) * numLabelsPerStop;
            // returns true if any of the distances to or from this stop are set
            for (int idx = startIdxForStop; idx < endIdxForStop; ++idx) {
                if (!allSet(distToRelevantPDLocs[idx] == DistanceLabel(INFTY)) || !allSet(distFromRelevantPDLocsToNextStop[idx] == DistanceLabel(INFTY)))
                    return true;
            }

            return false;
        }

        // Represents a block of DistanceLabels of size n that contains distances or meeting vertices for n * K PD locs.
        // Allows random access to individual label in the block given a PD loc id.
        // Used to hide intrinsics of DistanceLabels to caller.
        class PerPDLocFacade {

            using It = typename DistsVector::const_iterator;

        public:

            int operator[](const unsigned int pdLocId) const {
                assert(pdLocId / K < numLabelsPerStop);
                return labelBegin[pdLocId / K][pdLocId % K];
            }

        private:
            friend FeasibleEllipticDistances;

            PerPDLocFacade(const It labelBegin, const int numLabelsPerStop) : labelBegin(labelBegin),
                                                                              numLabelsPerStop(
                                                                                      numLabelsPerStop) {}

            const It labelBegin;
            const int numLabelsPerStop;
        };


        PerPDLocFacade distancesToRelevantPDLocsFor(const int stopId) const {
            assert(stopId <= maxStopId);
            const auto start = stopId * numLabelsPerStop;
            assert(distToRelevantPDLocs.begin() + start + numLabelsPerStop <= distToRelevantPDLocs.end());
            return {distToRelevantPDLocs.begin() + start, numLabelsPerStop};
        }

        int minDistToRelevantPDLocsFor(const int stopId) const {
            assert(stopId <= maxStopId);
            // assert(startOfRangeInValueArray[stopId] != INVALID_INDEX);
            return minDistToPDLoc[stopId];
        }

        PerPDLocFacade meetingVerticesToRelevantPDLocsFor(const int stopId) const {
            assert(stopId <= maxStopId);
            // assert(startOfRangeInValueArray[stopId] != INVALID_INDEX);
            const auto start = stopId * numLabelsPerStop;
            assert(meetingVerticesToRelevantPDLocs.begin() + start + numLabelsPerStop <=
                   meetingVerticesToRelevantPDLocs.end());
            return {meetingVerticesToRelevantPDLocs.begin() + start, numLabelsPerStop};
        }

        PerPDLocFacade distancesFromRelevantPDLocsToNextStopOf(const int stopId) const {
            assert(stopId <= maxStopId);
            // assert(startOfRangeInValueArray[stopId] != INVALID_INDEX);
            const auto start = stopId * numLabelsPerStop;
            assert(distFromRelevantPDLocsToNextStop.begin() + start + numLabelsPerStop <=
                   distFromRelevantPDLocsToNextStop.end());
            return {distFromRelevantPDLocsToNextStop.begin() + start, numLabelsPerStop};
        }

        int minDistFromPDLocToNextStopOf(const int stopId) const {
            assert(stopId <= maxStopId);
            // assert(startOfRangeInValueArray[stopId] != INVALID_INDEX);
            return minDistFromPDLocToNextStop[stopId];
        }

        PerPDLocFacade meetingVerticesFromRelevantPDLocsToNextStopOf(const int stopId) const {
            assert(stopId <= maxStopId);
            // assert(startOfRangeInValueArray[stopId] != INVALID_INDEX);
            const auto start = stopId * numLabelsPerStop;
            assert(meetingVerticesFromRelevantPDLocsToNextStop.begin() + start + numLabelsPerStop <=
                   meetingVerticesFromRelevantPDLocsToNextStop.end());
            return {meetingVerticesFromRelevantPDLocsToNextStop.begin() + start, numLabelsPerStop};
        }

        const ThreadSafeSubset &getVehiclesWithRelevantPDLocs() const {
            return vehiclesWithRelevantPDLocs;
        }

    private:
        // Dynamic Allocation
        // void allocateEntriesFor(const int stopId) {
        //     assert(startOfRangeInValueArray[stopId] == INVALID_INDEX);
        //     const auto curNumLabels = distToRelevantPDLocs.size();
        //     startOfRangeInValueArray[stopId] = curNumLabels;
        //     vehiclesWithRelevantPDLocs.insert(routeState.vehicleIdOf(stopId));
        //     distToRelevantPDLocs.insert(distToRelevantPDLocs.end(), numLabelsPerStop, DistanceLabel(INFTY));
        //     distFromRelevantPDLocsToNextStop.insert(distFromRelevantPDLocsToNextStop.end(),
        //                                             numLabelsPerStop, DistanceLabel(INFTY));
        //     meetingVerticesToRelevantPDLocs.insert(meetingVerticesToRelevantPDLocs.end(), numLabelsPerStop,
        //                                            DistanceLabel(INVALID_VERTEX));
        //     meetingVerticesFromRelevantPDLocsToNextStop.insert(
        //             meetingVerticesFromRelevantPDLocsToNextStop.end(),
        //             numLabelsPerStop, DistanceLabel(INVALID_VERTEX));
      
        //     minDistToPDLoc[stopId] = INFTY;
        //     minDistFromPDLocToNextStop[stopId] = INFTY;
        // }

        const RouteState &routeState;

        int numLabelsPerStop{};
        const int &maxStopId;

        // Points from a stop id to the start of the entries in the value arrays for PD locs that are relevant
        // for this stop. Not used in case of static allocation
        // std::vector<int> startOfRangeInValueArray;

        // Value arrays.
        DistsVector distToRelevantPDLocs;
        DistsVector distFromRelevantPDLocsToNextStop;
        MeetingVerticesVector meetingVerticesToRelevantPDLocs;
        MeetingVerticesVector meetingVerticesFromRelevantPDLocsToNextStop;

        ThreadSafeSubset vehiclesWithRelevantPDLocs;

        std::vector<std::atomic_int> minDistToPDLoc;
        std::vector<std::atomic_int> minDistFromPDLocToNextStop;

    };

}