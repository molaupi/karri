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

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Tools/Timer.h"

namespace karri {
    template<typename LabelSetT, bool StoreMeetingVertices = false>
    class StaticFeasibleEllipticDistances {
        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        using DistancesVector = AlignedVector<DistanceLabel>;
        using MeetingVerticesVector = AlignedVector<DistanceLabel>;

    public:
        explicit StaticFeasibleEllipticDistances(const int fleetSize, const RouteState &routeState)
            : routeState(routeState),
              fleetSize(fleetSize),
              maxStopId(routeState.getMaxStopId()),
              stopIdsWithRelevantPDLocs(fleetSize),
              minDistToPDLoc(fleetSize),
              minDistFromPDLocToNextStop(fleetSize) {
        }

        void init(const int newNumPDLocs, stats::EllipticBCHPerformanceStats &stats) {
            Timer timer;
            const auto prevNumLabelsPerStop = numLabelsPerStop;

            // Reset distances using stopIdsWithRelevantPDLocs from previous run,
            // then clear stopIdsWithRelevantPDLocs
            for (const auto &stopId: stopIdsWithRelevantPDLocs) {
                const auto startOfStopRange = prevNumLabelsPerStop * stopId;
                std::fill_n(distToRelevantPDLocs.begin() + startOfStopRange, prevNumLabelsPerStop, INFTY);
                std::fill_n(distFromRelevantPDLocsToNextStop.begin() + startOfStopRange, prevNumLabelsPerStop, INFTY);
                if constexpr (StoreMeetingVertices) {
                    std::fill_n(meetingVerticesToRelevantPDLocs.begin() + startOfStopRange, prevNumLabelsPerStop, INVALID_VERTEX);
                    std::fill_n(meetingVerticesFromRelevantPDLocsToNextStop.begin() + startOfStopRange, prevNumLabelsPerStop, INVALID_VERTEX);
                }
                minDistToPDLoc[stopId] = INFTY;
                minDistFromPDLocToNextStop[stopId] = INFTY;
            }
            KASSERT(std::all_of(distToRelevantPDLocs.begin(), distToRelevantPDLocs.end(),
                [](const auto &label) { return label == INFTY; }));
            KASSERT(std::all_of(distFromRelevantPDLocsToNextStop.begin(), distFromRelevantPDLocsToNextStop.end(),
                [](const auto &label) { return label == INFTY; }));
            stopIdsWithRelevantPDLocs.clear();


            if (maxStopId >= stopIdsWithRelevantPDLocs.size()) {
                stopIdsWithRelevantPDLocs.setCapacity(maxStopId + 1);
                minDistToPDLoc.resize(maxStopId + 1);
                minDistFromPDLocToNextStop.resize(maxStopId + 1);
            }

            numLabelsPerStop = newNumPDLocs / K + (newNumPDLocs % K != 0);
            const auto newTotalNumLabels = numLabelsPerStop * (maxStopId + 1);
            if (newTotalNumLabels > distToRelevantPDLocs.size()) {
                distToRelevantPDLocs.resize(newTotalNumLabels, INFTY);
                distFromRelevantPDLocsToNextStop.resize(newTotalNumLabels, INFTY);
                if constexpr (StoreMeetingVertices) {
                    meetingVerticesToRelevantPDLocs.resize(newTotalNumLabels, INVALID_VERTEX);
                    meetingVerticesFromRelevantPDLocsToNextStop.resize(newTotalNumLabels, INVALID_VERTEX);
                }
            }

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.initializationTime += time;
        }

        template<typename PDLocsAtExistingStopsT, typename InputGraphT>
        void initializeDistancesForPdLocsAtExistingStops(PDLocsAtExistingStopsT &&pdLocsAtExistingStops,
                                                         const InputGraphT &inputGraph,
                                                         stats::EllipticBCHPerformanceStats &stats) {
            Timer timer;

            // Pre-allocate entries for PD locs at existing stops. The distance 0 may otherwise not be found by the
            // BCH searches. Also, this way, the distance for such a PD loc never has to be updated, and we already
            // allocate the entry array for this stop, which is good since it will likely also be reachable by other PD locs.
            for (const auto &pdLocAtExistingStop: pdLocsAtExistingStops) {
                const auto &vehId = pdLocAtExistingStop.vehId;
                KASSERT(pdLocAtExistingStop.stopIndex < routeState.numStopsOf(vehId));
                const auto &stopId = routeState.stopIdsFor(vehId)[pdLocAtExistingStop.stopIndex];
                const auto &stopVertex = inputGraph.edgeHead(
                    routeState.stopLocationsFor(vehId)[pdLocAtExistingStop.stopIndex]);
                // allocateEntriesFor(stopId);
                stopIdsWithRelevantPDLocs.insert(stopId);

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

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.initializationTime += time;
        }

        // Updates the distance from stop to the PD loc. Distance is written if there are
        // entries for the stop already or dynamic allocation of entries is allowed.
        // Returns mask indicating where the distance has been improved (all false if we don't know the stop and dynamic
        // allocation is not allowed).
        void updateDistanceFromStopToPDLoc(const int stopId, const unsigned int firstPDLocId,
                                           const DistanceLabel newDistToPDLoc, const int meetingVertex) {
            KASSERT(stopId >= 0 && stopId <= maxStopId);
            KASSERT(firstPDLocId < numLabelsPerStop * K);
            KASSERT(firstPDLocId % K == 0);
            KASSERT(newDistToPDLoc.horizontalMin() >= 0 && newDistToPDLoc.horizontalMin() < INFTY);

            // Memorize that we have seen a distance for this stop
            stopIdsWithRelevantPDLocs.insert(stopId);

            // Write values for new entry and set pointer from PD loc to the entries
            const auto idx = stopId * numLabelsPerStop + firstPDLocId / K;
            if constexpr (StoreMeetingVertices) {
                const LabelMask improved = newDistToPDLoc < distToRelevantPDLocs[idx];
                distToRelevantPDLocs[idx].setIf(newDistToPDLoc, improved);
                meetingVerticesToRelevantPDLocs[idx].setIf(meetingVertex, improved);
            } else {
                distToRelevantPDLocs[idx].min(newDistToPDLoc);
            }

            auto &globalMin = minDistToPDLoc[stopId];
            globalMin.min(newDistToPDLoc);
        }

        // Updates the distance from the PD loc to the stop that follows stopId. Distance is written only if entries
        // for the stop exist already.
        // Returns mask indicating where the distance has been improved (all false if we don't know the stop).
        void updateDistanceFromPDLocToNextStop(const int stopId, const int firstPDLocId,
                                               const DistanceLabel newDistFromPDLocToNextStop,
                                               const int meetingVertex) {
            // We assume the from-searches are run after the to-searches. If the stop does not have entries yet, it was
            // considered irrelevant for the to-searches (regardless of whether we allow dynamic allocation or not).
            // Therefore, this stop cannot be relevant on both sides which means we can skip it here.
            if (!stopIdsWithRelevantPDLocs.contains(stopId))
                return;

            const auto idx = stopId * numLabelsPerStop + firstPDLocId / K;
            if constexpr (StoreMeetingVertices) {
                const LabelMask improved = newDistFromPDLocToNextStop < distFromRelevantPDLocsToNextStop[idx];
                distFromRelevantPDLocsToNextStop[idx].setIf(newDistFromPDLocToNextStop, improved);
                meetingVerticesFromRelevantPDLocsToNextStop[idx].setIf(meetingVertex, improved);
            } else {
                distFromRelevantPDLocsToNextStop[idx].min(newDistFromPDLocToNextStop);
            }

            auto &globalMin = minDistFromPDLocToNextStop[stopId];
            globalMin.min(newDistFromPDLocToNextStop);
        }

        bool hasPotentiallyRelevantPDLocs(const int stopId) const {
            KASSERT(stopId <= maxStopId);
            return stopIdsWithRelevantPDLocs.contains(stopId);
        }

        // Represents a block of DistanceLabels of size n that contains distances or meeting vertices for n * K PD locs.
        // Allows random access to individual label in the block given a PD loc id.
        // Used to hide intrinsics of DistanceLabels to caller.
        class PerPDLocFacade {
            using It = typename DistancesVector::const_iterator;

        public:
            int operator[](const unsigned int pdLocId) const {
                KASSERT(pdLocId / K < numLabelsPerStop);
                return labelBegin[pdLocId / K][pdLocId % K];
            }

        private:
            friend StaticFeasibleEllipticDistances;

            PerPDLocFacade(const It labelBegin, const int numLabelsPerStop) : labelBegin(labelBegin),
                                                                              numLabelsPerStop(numLabelsPerStop) {
            }

            const It labelBegin;
            const int numLabelsPerStop;
        };


        PerPDLocFacade distancesToRelevantPDLocsFor(const int stopId) const {
            KASSERT(stopId <= maxStopId);
            KASSERT(hasPotentiallyRelevantPDLocs(stopId));
            const auto start = stopId * numLabelsPerStop;
            KASSERT(distToRelevantPDLocs.begin() + start + numLabelsPerStop <= distToRelevantPDLocs.end());
            return {distToRelevantPDLocs.begin() + start, numLabelsPerStop};
        }

        int minDistToRelevantPDLocsFor(const int stopId) const {
            KASSERT(stopId <= maxStopId);
            KASSERT(hasPotentiallyRelevantPDLocs(stopId));
            return minDistToPDLoc[stopId].horizontalMin();
        }

        PerPDLocFacade meetingVerticesToRelevantPDLocsFor(const int stopId) const requires StoreMeetingVertices {
            KASSERT(stopId <= maxStopId);
            KASSERT(hasPotentiallyRelevantPDLocs(stopId));
            const auto start = stopId * numLabelsPerStop;
            KASSERT(meetingVerticesToRelevantPDLocs.begin() + start + numLabelsPerStop <=
                meetingVerticesToRelevantPDLocs.end());
            return {meetingVerticesToRelevantPDLocs.begin() + start, numLabelsPerStop};
        }

        PerPDLocFacade distancesFromRelevantPDLocsToNextStopOf(const int stopId) const {
            KASSERT(stopId <= maxStopId);
            KASSERT(hasPotentiallyRelevantPDLocs(stopId));
            const auto start = stopId * numLabelsPerStop;
            KASSERT(distFromRelevantPDLocsToNextStop.begin() + start + numLabelsPerStop <=
                distFromRelevantPDLocsToNextStop.end());
            return {distFromRelevantPDLocsToNextStop.begin() + start, numLabelsPerStop};
        }

        int minDistFromPDLocToNextStopOf(const int stopId) const {
            KASSERT(stopId <= maxStopId);
            KASSERT(hasPotentiallyRelevantPDLocs(stopId));
            return minDistFromPDLocToNextStop[stopId].horizontalMin();
        }

        PerPDLocFacade meetingVerticesFromRelevantPDLocsToNextStopOf(const int stopId) const requires
            StoreMeetingVertices {
            KASSERT(stopId <= maxStopId);
            KASSERT(hasPotentiallyRelevantPDLocs(stopId));
            const auto start = stopId * numLabelsPerStop;
            KASSERT(meetingVerticesFromRelevantPDLocsToNextStop.begin() + start + numLabelsPerStop <=
                meetingVerticesFromRelevantPDLocsToNextStop.end());
            return {meetingVerticesFromRelevantPDLocsToNextStop.begin() + start, numLabelsPerStop};
        }


        // Makes unexpectedly heavy computations, use with care.
        int numVehiclesWithRelevantPDLocs() const {
            BitVector hasPdLocs(fleetSize);
            for (const auto &stopId: stopIdsWithRelevantPDLocs) {
                hasPdLocs[routeState.vehicleIdOf(stopId)] = true;
            }
            return hasPdLocs.cardinality();
        }

        const Subset &getStopIdsWithRelevantPDLocs() const {
            return stopIdsWithRelevantPDLocs;
        }

        int numStopsWithRelevantPDLocs() const {
            return stopIdsWithRelevantPDLocs.size();
        }

    private:
        // void allocateEntriesFor(const int stopId) {
        //     stopIdsWithRelevantPDLocs.insert(stopId);
        //     minDistToPDLoc[stopId] = INFTY;
        //     minDistFromPDLocToNextStop[stopId] = INFTY;
        // }

        const RouteState &routeState;
        const int fleetSize;

        int numLabelsPerStop{};
        const int &maxStopId;

        // Value arrays.
        DistancesVector distToRelevantPDLocs;
        DistancesVector distFromRelevantPDLocsToNextStop;
        MeetingVerticesVector meetingVerticesToRelevantPDLocs;
        MeetingVerticesVector meetingVerticesFromRelevantPDLocsToNextStop;

        // Iterable set of stop IDs that have relevant PD locs.
        Subset stopIdsWithRelevantPDLocs;

        std::vector<DistanceLabel> minDistToPDLoc;
        std::vector<DistanceLabel> minDistFromPDLocToNextStop;
    };
}
