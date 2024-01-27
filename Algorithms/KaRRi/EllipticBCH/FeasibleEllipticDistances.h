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
#include "Tools/Simd/ConcurrentAlignedVector.h"
#include "Tools/Simd/AlignedVector.h"
#include "DataStructures/Containers/Subset.h"
#include "DataStructures/Containers/ThreadSafeSubset.h"

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Parallel/atomic_wrapper.h"

#include <atomic>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

namespace karri {


    template<typename LabelSetT>
    class FeasibleEllipticDistances {


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        using ConcurrentDistsVector = ConcurrentAlignedVector<DistanceLabel>;
        using ConcurrentMeetingVerticesVector = ConcurrentAlignedVector<DistanceLabel>;

        using LocalDistsVector = AlignedVector<DistanceLabel>;
        using LocalMeetingVerticesVector = AlignedVector<DistanceLabel>;
    public:

        explicit FeasibleEllipticDistances(const int fleetSize, const RouteState &routeState)
                : routeState(routeState),
                  maxStopId(routeState.getMaxStopId()),
                  startOfRangeInDistToPDLocs(fleetSize, INVALID_INDEX),
                  startOfRangeInDistFromPDLocs(fleetSize, INVALID_INDEX),
                  startOfRangeInMeetingVerticesToPDLocs(fleetSize, INVALID_INDEX),
                  startOfRangeInMeetingVerticesFromPDLocs(fleetSize, INVALID_INDEX),
                  stopLocks(fleetSize, SpinLock()),
                //   startOfRangeInValueArray(maxStopId + 1, INVALID_INDEX),
                  vehiclesWithRelevantPDLocs(fleetSize),
                  minDistToPDLoc(fleetSize),
                  minDistFromPDLocToNextStop(fleetSize) {}

        template<typename PDLocsAtExistingStopsT, typename InputGraphT>
        void init(const int newNumPDLocs, const PDLocsAtExistingStopsT &pdLocsAtExistingStops,
                  const InputGraphT &inputGraph) {
            numLabelsPerStop = newNumPDLocs / K + (newNumPDLocs % K != 0);

            std::fill(startOfRangeInDistToPDLocs.begin(), startOfRangeInDistToPDLocs.end(), INVALID_INDEX);
            std::fill(startOfRangeInDistFromPDLocs.begin(), startOfRangeInDistFromPDLocs.end(), INVALID_INDEX);
            std::fill(startOfRangeInMeetingVerticesToPDLocs.begin(), startOfRangeInMeetingVerticesToPDLocs.end(), INVALID_INDEX);
            std::fill(startOfRangeInMeetingVerticesFromPDLocs.begin(), startOfRangeInMeetingVerticesFromPDLocs.end(), INVALID_INDEX);


            if (maxStopId >= startOfRangeInDistToPDLocs.size()) {
                stopLocks.resize(maxStopId + 1, SpinLock());
                startOfRangeInDistToPDLocs.resize(maxStopId + 1, INVALID_INDEX);
                startOfRangeInDistFromPDLocs.resize(maxStopId + 1, INVALID_INDEX);
                startOfRangeInMeetingVerticesToPDLocs.resize(maxStopId + 1, INVALID_INDEX);
                startOfRangeInMeetingVerticesFromPDLocs.resize(maxStopId + 1, INVALID_INDEX);
                minDistToPDLoc.clear();
                minDistToPDLoc = std::vector<std::atomic_int>(maxStopId + 1);
                minDistFromPDLocToNextStop.clear();
                minDistFromPDLocToNextStop = std::vector<std::atomic_int>(maxStopId + 1);
            }

            // std::vector<int> &localStartOfRangeInValueArray = startOfRangeInValueArray.local();
            // if (maxStopId >= localStartOfRangeInValueArray.size()) {
            //     startOfRangeInValueArray.clear();
            //     startOfRangeInValueArray = enumerable_thread_specific<std::vector<int>>(maxStopId + 1, INVALID_INDEX);
            // }

            distToPDLoc = enumerable_thread_specific<LocalDistsVector>(maxStopId + 1);
            distFromPDLocToNextStop = enumerable_thread_specific<LocalDistsVector>(maxStopId + 1);
            meetingVerticesToPDLoc = enumerable_thread_specific<LocalMeetingVerticesVector>(maxStopId + 1);
            meetingVerticesFromPDLocToNextStop = enumerable_thread_specific<LocalMeetingVerticesVector>(maxStopId + 1);
                
            vehiclesWithRelevantPDLocs.clear();

            distToRelevantPDLocs.clear();
            distFromRelevantPDLocsToNextStop.clear();
            meetingVerticesToRelevantPDLocs.clear();
            meetingVerticesFromRelevantPDLocsToNextStop.clear();

            // Pre-allocate entries for PD locs at existing stops. The distance 0 may otherwise not be found by the
            // BCH searches. Also, this way, the distance for such a PD loc never has to be updated, and we already
            // allocate the entry array for this stop, which is good since it will likely also be reachable by other PD locs.
            for (const auto &pdLocAtExistingStop: pdLocsAtExistingStops) {
                const auto &vehId = pdLocAtExistingStop.vehId;
                assert(pdLocAtExistingStop.stopIndex < routeState.numStopsOf(vehId));
                const auto &stopId = routeState.stopIdsFor(vehId)[pdLocAtExistingStop.stopIndex];
                const auto &stopVertex = inputGraph.edgeHead(
                        routeState.stopLocationsFor(vehId)[pdLocAtExistingStop.stopIndex]);
                // Write values for new entry and set pointer from PD loc to the entries directly into global storages
                allocateEntriesFor(stopId);

                DistanceLabel zeroLabel = INFTY;
                zeroLabel[pdLocAtExistingStop.pdId % K] = 0;
                const auto firstIdInBatch = (pdLocAtExistingStop.pdId / K) * K;

                // Distances to PDLocs
                const auto toDistIdx = startOfRangeInDistToPDLocs[stopId] + firstIdInBatch / K;
                const auto toMeetingVertexIdx = startOfRangeInMeetingVerticesToPDLocs[stopId] + firstIdInBatch / K;

                const LabelMask improvedTo = zeroLabel < distToRelevantPDLocs[toDistIdx];

                distToRelevantPDLocs[toDistIdx].setIf(zeroLabel, improvedTo);
                meetingVerticesToRelevantPDLocs[toMeetingVertexIdx].setIf(stopVertex, improvedTo);

                if (anySet(improvedTo)) {

                    const int minNewDistToPDLoc = zeroLabel.horizontalMin();

                    auto& minToPDLocAtomic = minDistToPDLoc[stopId];
                    int expectedMinForStop = minToPDLocAtomic.load(std::memory_order_relaxed);
                    while(expectedMinForStop > minNewDistToPDLoc && !minToPDLocAtomic.compare_exchange_strong(expectedMinForStop, minNewDistToPDLoc, std::memory_order_relaxed));
                }

                const auto lengthOfLegStartingHere = time_utils::calcLengthOfLegStartingAt(
                        pdLocAtExistingStop.stopIndex, pdLocAtExistingStop.vehId, routeState);
                DistanceLabel lengthOfLegLabel = INFTY;
                lengthOfLegLabel[pdLocAtExistingStop.pdId % K] = lengthOfLegStartingHere;

                // Distances from PDLocs
                const auto fromDistIdx = startOfRangeInDistFromPDLocs[stopId] + firstIdInBatch / K;
                const auto fromMeetingVertexIdx = startOfRangeInMeetingVerticesFromPDLocs[stopId] + firstIdInBatch / K;
                const LabelMask improvedFrom = lengthOfLegLabel < distFromRelevantPDLocsToNextStop[fromDistIdx];

                distFromRelevantPDLocsToNextStop[fromDistIdx].setIf(lengthOfLegLabel, improvedFrom);
                meetingVerticesFromRelevantPDLocsToNextStop[fromMeetingVertexIdx].setIf(stopVertex, improvedFrom);

                if (anySet(improvedFrom)) {

                    const int minNewDistFromPDLocToNextStop = lengthOfLegLabel.horizontalMin();

                    auto& minFromPDLocAtomic = minDistFromPDLocToNextStop[stopId];
                    int expectedMinForStop = minFromPDLocAtomic.load(std::memory_order_relaxed);
                    while(expectedMinForStop > minNewDistFromPDLocToNextStop && !minFromPDLocAtomic.compare_exchange_strong(expectedMinForStop, minNewDistFromPDLocToNextStop, std::memory_order_relaxed));
                }
            }
        }

        void initLocal() {
            LocalDistsVector &localDistToPDLoc = distToPDLoc.local();
            LocalDistsVector &localDistFromPDLocToNextStop = distFromPDLocToNextStop.local();
            LocalMeetingVerticesVector &localMeetingVerticesToPDLoc = meetingVerticesToPDLoc.local();
            LocalMeetingVerticesVector &localMeetingVerticesFromPDLocToNextStop = meetingVerticesFromPDLocToNextStop.local();

            std::fill(localDistToPDLoc.begin(), localDistToPDLoc.end(), DistanceLabel(INFTY));
            std::fill(localDistFromPDLocToNextStop.begin(), localDistFromPDLocToNextStop.end(), DistanceLabel(INFTY));
            std::fill(localMeetingVerticesToPDLoc.begin(), localMeetingVerticesToPDLoc.end(), DistanceLabel(INVALID_VERTEX));
            std::fill(localMeetingVerticesFromPDLocToNextStop.begin(), localMeetingVerticesFromPDLocToNextStop.end(), DistanceLabel(INVALID_VERTEX));

            // for (int i = 0; i <= maxStopId; i++) {
            //     localDistToPDLoc[i] = DistanceLabel(INFTY);
            //     localDistFromPDLocToNextStop[i] = DistanceLabel(INFTY);
            //     localMeetingVerticesToPDLoc[i] = DistanceLabel(INVALID_VERTEX);
            //     localMeetingVerticesFromPDLocToNextStop[i] = DistanceLabel(INVALID_VERTEX);
            // }
        }


        // Updates the distance from stop to the PD loc. Distance is written if there are
        // entries for the stop already or dynamic allocation of entries is allowed.
        // Returns mask indicating where the distance has been improved (all false if we don't know the stop and dynamic
        // allocation is not allowed).
        LabelMask updateDistanceFromStopToPDLoc(const int stopId, const unsigned int,
                                                const DistanceLabel newDistToPDLoc, const int meetingVertex) {
            assert(stopId >= 0 && stopId <= maxStopId);
            assert(newDistToPDLoc.horizontalMin() >= 0 && newDistToPDLoc.horizontalMin() < INFTY);

            // std::vector<int> &localStartOfRange = startOfRangeInValueArray.local();
            // if (localStartOfRange[stopId] == INVALID_INDEX) {
            //     allocateLocalEntriesFor(stopId);
            // }

            LocalDistsVector &localDistToPDLoc = distToPDLoc.local();
            LocalMeetingVerticesVector &localMeetingVerticesToPDLoc = meetingVerticesToPDLoc.local();

            // const auto idx = localStartOfRange[stopId] + firstPDLocId / K;
            const LabelMask improvedLocal = newDistToPDLoc < localDistToPDLoc[stopId];

            localDistToPDLoc[stopId].setIf(newDistToPDLoc, improvedLocal);
            localMeetingVerticesToPDLoc[stopId].setIf(meetingVertex, improvedLocal);       
            
            return improvedLocal;
        }

        // Updates the distance from the PD loc to the stop that follows stopId. Distance is written only if entries
        // for the stop exist already.
        // Returns mask indicating where the distance has been improved (all false if we don't know the stop).
        LabelMask updateDistanceFromPDLocToNextStop(const int stopId, const int,
                                                    const DistanceLabel newDistFromPDLocToNextStop,
                                                    const int meetingVertex) {


            // std::vector<int> &localStartOfRange = startOfRangeInValueArray.local();
            // if (localStartOfRange[stopId] == INVALID_INDEX) 
            //     return LabelMask(false);
            
            LocalDistsVector &localDistFromPDLocToNextStop = distFromPDLocToNextStop.local();
            LocalMeetingVerticesVector &localMeetingVerticesFromPDLocToNextStop = meetingVerticesFromPDLocToNextStop.local();

            // const auto idx = localStartOfRange[stopId] + firstPDLocId / K;
            const LabelMask improvedLocal = newDistFromPDLocToNextStop < localDistFromPDLocToNextStop[stopId];

            localDistFromPDLocToNextStop[stopId].setIf(newDistFromPDLocToNextStop, improvedLocal);
            localMeetingVerticesFromPDLocToNextStop[stopId].setIf(meetingVertex, improvedLocal);

            return improvedLocal;
        }

        bool hasPotentiallyRelevantPDLocs(const int stopId) const {
            assert(stopId <= maxStopId);
            return startOfRangeInDistToPDLocs[stopId] != INVALID_INDEX || startOfRangeInDistFromPDLocs[stopId] != INVALID_INDEX;
        }

        void updateToDistancesInGlobalVectors(const int firstPDLocId) {
            // std::vector<int> &localStartOfRange = startOfRangeInValueArray.local();
            LocalDistsVector &localDistToPDLoc = distToPDLoc.local();
            LocalMeetingVerticesVector &localMeetingVerticesToPDLoc = meetingVerticesToPDLoc.local();

            for (int i = 0; i <= maxStopId; ++i) {
                // const auto &idx = localStartOfRange[i];
                // Allocate the entries in global storage if not yet done
                allocateEntriesFor(i);
                const auto toDistIdx = startOfRangeInDistToPDLocs[i] + firstPDLocId / K;
                const auto toMeetingVertexIdx = startOfRangeInMeetingVerticesToPDLocs[i] + firstPDLocId / K;

                const LabelMask improved = localDistToPDLoc[i] < distToRelevantPDLocs[toDistIdx];

                distToRelevantPDLocs[toDistIdx].setIf(localDistToPDLoc[i], improved);
                meetingVerticesToRelevantPDLocs[toMeetingVertexIdx].setIf(localMeetingVerticesToPDLoc[i], improved);

                // Write values for new entry and set pointer from PD loc to the entries
                if (anySet(improved)) {
                    const int minNewDistToPDLoc = localDistToPDLoc[i].horizontalMin();
                    auto& minToPDLocAtomic = minDistToPDLoc[i];
                    int expectedMinForStop = minToPDLocAtomic.load(std::memory_order_relaxed);
                    while(expectedMinForStop > minNewDistToPDLoc && !minToPDLocAtomic.compare_exchange_strong(expectedMinForStop, minNewDistToPDLoc, std::memory_order_relaxed));
                }
            }
        }

        void updateFromDistancesInGlobalVectors(const int firstPDLocId) {
            // std::vector<int> &localStartOfRange = startOfRangeInValueArray.local();
            LocalDistsVector &localDistFromPDLocToNextStop = distFromPDLocToNextStop.local();
            LocalMeetingVerticesVector &localMeetingVerticesFromPDLocToNextStop = meetingVerticesFromPDLocToNextStop.local();


            for (int i = 0; i <= maxStopId; ++i) {
                // const auto &idx = localStartOfRange[i];
                // We assume the from-searches are run after the to-searches. If the stop does not have entries yet, it was
                // considered irrelevant for the to-searches (regardless of whether we allow dynamic allocation or not).
                // Therefore, this stop cannot be relevant on both sides which means we can skip it here.
                if (startOfRangeInDistToPDLocs[i] != INVALID_INDEX) {
                    const auto fromDistIdx = startOfRangeInDistFromPDLocs[i] + firstPDLocId / K;
                    const auto fromMeetingVertexIdx = startOfRangeInMeetingVerticesFromPDLocs[i] + firstPDLocId / K;

                    const LabelMask improved = localDistFromPDLocToNextStop[i] < distFromRelevantPDLocsToNextStop[fromDistIdx];

                    distFromRelevantPDLocsToNextStop[fromDistIdx].setIf(localDistFromPDLocToNextStop[i], improved);
                    meetingVerticesFromRelevantPDLocsToNextStop[fromMeetingVertexIdx].setIf(localMeetingVerticesFromPDLocToNextStop[i], improved);

                    // Write values for new entry and set pointer from PD loc to the entries
                    if (anySet(improved)) {
                        const int minNewDistFromPDLocToNextStop = localDistFromPDLocToNextStop[i].horizontalMin();
                        auto& minFromPDLocAtomic = minDistFromPDLocToNextStop[i];
                        int expectedMinForStop = minFromPDLocAtomic.load(std::memory_order_relaxed);
                        while(expectedMinForStop > minNewDistFromPDLocToNextStop && !minFromPDLocAtomic.compare_exchange_strong(expectedMinForStop, minNewDistFromPDLocToNextStop, std::memory_order_relaxed));
                    }              
                }
            }
        }

        void endToSearches() {
            distToPDLoc.clear();
            meetingVerticesToPDLoc.clear();
        }

        void endFromSearches() {
            distFromPDLocToNextStop.clear();
            meetingVerticesFromPDLocToNextStop.clear();
        }

        // Represents a block of DistanceLabels of size n that contains distances or meeting vertices for n * K PD locs.
        // Allows random access to individual label in the block given a PD loc id.
        // Used to hide intrinsics of DistanceLabels to caller.
        class PerPDLocFacade {

            using It = typename ConcurrentDistsVector::const_iterator;

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
            assert(startOfRangeInDistToPDLocs[stopId] != INVALID_INDEX);
            const auto start = startOfRangeInDistToPDLocs[stopId];
            // const auto start = stopId * numLabelsPerStop;
            assert(distToRelevantPDLocs.begin() + start + numLabelsPerStop <= distToRelevantPDLocs.end());
            return {distToRelevantPDLocs.begin() + start, numLabelsPerStop};
        }

        int minDistToRelevantPDLocsFor(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRangeInDistToPDLocs[stopId] != INVALID_INDEX);
            return minDistToPDLoc[stopId];
        }

        PerPDLocFacade meetingVerticesToRelevantPDLocsFor(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRangeInMeetingVerticesToPDLocs[stopId] != INVALID_INDEX);
            const auto start = startOfRangeInMeetingVerticesToPDLocs[stopId];
            // const auto start = stopId * numLabelsPerStop;
            assert(meetingVerticesToRelevantPDLocs.begin() + start + numLabelsPerStop <=
                   meetingVerticesToRelevantPDLocs.end());
            return {meetingVerticesToRelevantPDLocs.begin() + start, numLabelsPerStop};
        }

        PerPDLocFacade distancesFromRelevantPDLocsToNextStopOf(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRangeInDistFromPDLocs[stopId] != INVALID_INDEX);
            const auto start = startOfRangeInDistFromPDLocs[stopId];
            // const auto start = stopId * numLabelsPerStop;
            assert(distFromRelevantPDLocsToNextStop.begin() + start + numLabelsPerStop <=
                   distFromRelevantPDLocsToNextStop.end());
            return {distFromRelevantPDLocsToNextStop.begin() + start, numLabelsPerStop};
        }

        int minDistFromPDLocToNextStopOf(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRangeInDistFromPDLocs[stopId] != INVALID_INDEX);
            return minDistFromPDLocToNextStop[stopId];
        }

        PerPDLocFacade meetingVerticesFromRelevantPDLocsToNextStopOf(const int stopId) const {
            assert(stopId <= maxStopId);
            assert(startOfRangeInMeetingVerticesFromPDLocs[stopId] != INVALID_INDEX);
            const auto start = startOfRangeInMeetingVerticesFromPDLocs[stopId];
            // const auto start = stopId * numLabelsPerStop;
            assert(meetingVerticesFromRelevantPDLocsToNextStop.begin() + start + numLabelsPerStop <=
                   meetingVerticesFromRelevantPDLocsToNextStop.end());
            return {meetingVerticesFromRelevantPDLocsToNextStop.begin() + start, numLabelsPerStop};
        }

        const ThreadSafeSubset &getVehiclesWithRelevantPDLocs() const {
            return vehiclesWithRelevantPDLocs;
        }

    private:
        // Dynamic Allocation
        // to & from searches combine: 2 allocate (to & from)
        // 2 subsets for from and to
        // void allocateLocalEntriesFor(const int stopId) {
        //     std::vector<int> &localStartOfRange = startOfRangeInValueArray.local();
        //     assert(localStartOfRange[stopId] == INVALID_INDEX);  

        //     LocalDistsVector &localDistToPDLoc = distToPDLoc.local();
        //     LocalDistsVector &localDistFromPDLocToNextStop = distFromPDLocToNextStop.local();
        //     LocalMeetingVerticesVector &localMeetingVerticesToPDLoc = meetingVerticesToPDLoc.local();
        //     LocalMeetingVerticesVector &localMeetingVerticesFromPDLocToNextStop = meetingVerticesFromPDLocToNextStop.local();

        //     const auto curNumLabels = localDistToPDLoc.size();
        //     localStartOfRange[stopId] = curNumLabels;
        //     vehiclesWithRelevantPDLocs.insert(routeState.vehicleIdOf(stopId));
        //     localDistToPDLoc.push_back(DistanceLabel(INFTY));
        //     localDistFromPDLocToNextStop.push_back(DistanceLabel(INFTY));
        //     localMeetingVerticesToPDLoc.push_back(DistanceLabel(INVALID_VERTEX));
        //     localMeetingVerticesFromPDLocToNextStop.push_back(DistanceLabel(INVALID_VERTEX));
        // }

        void allocateEntriesFor(const int stopId) {
            SpinLock& currLock = stopLocks[stopId];
            currLock.lock();

            if (startOfRangeInDistToPDLocs[stopId] != INVALID_INDEX &&
                startOfRangeInDistFromPDLocs[stopId] != INVALID_INDEX &&
                startOfRangeInMeetingVerticesToPDLocs[stopId] != INVALID_INDEX &&
                startOfRangeInMeetingVerticesFromPDLocs[stopId] != INVALID_INDEX) {
                currLock.unlock();
                return;
            }

            const auto distToIt = distToRelevantPDLocs.grow_by(numLabelsPerStop, DistanceLabel(INFTY));
            startOfRangeInDistToPDLocs[stopId] = distToIt - distToRelevantPDLocs.begin();

            const auto distFromIt = distFromRelevantPDLocsToNextStop.grow_by(numLabelsPerStop, DistanceLabel(INFTY));
            startOfRangeInDistFromPDLocs[stopId] = distFromIt - distFromRelevantPDLocsToNextStop.begin();

            const auto meetingVerticesToIt = meetingVerticesToRelevantPDLocs.grow_by(numLabelsPerStop, DistanceLabel(INVALID_VERTEX));
            startOfRangeInMeetingVerticesToPDLocs[stopId] = meetingVerticesToIt - meetingVerticesToRelevantPDLocs.begin();

            const auto meetingVerticesFromIt = meetingVerticesFromRelevantPDLocsToNextStop.grow_by(numLabelsPerStop, DistanceLabel(INVALID_VERTEX));
            startOfRangeInMeetingVerticesFromPDLocs[stopId] = meetingVerticesFromIt - meetingVerticesFromRelevantPDLocsToNextStop.begin();

            minDistToPDLoc[stopId].store(INFTY);
            minDistFromPDLocToNextStop[stopId].store(INFTY);

            currLock.unlock();

            vehiclesWithRelevantPDLocs.insert(routeState.vehicleIdOf(stopId));

        }

        const RouteState &routeState;

        int numLabelsPerStop{};
        const int &maxStopId;

        // Points from a stop id to the start of the entries in the value arrays for PD locs that are relevant
        // for this stop. Not used in case of static allocation
        std::vector<int> startOfRangeInDistToPDLocs;
        std::vector<int> startOfRangeInDistFromPDLocs;
        std::vector<int> startOfRangeInMeetingVerticesToPDLocs;
        std::vector<int> startOfRangeInMeetingVerticesFromPDLocs;
        // Spinlock for thread safe dynamic allocation
        std::vector<SpinLock> stopLocks;

        // Value arrays.
        ConcurrentDistsVector distToRelevantPDLocs;
        ConcurrentDistsVector distFromRelevantPDLocsToNextStop;
        ConcurrentMeetingVerticesVector meetingVerticesToRelevantPDLocs;
        ConcurrentMeetingVerticesVector meetingVerticesFromRelevantPDLocsToNextStop;
        
        // Thread Local Storage for local distances calculation
        // enumerable_thread_specific<std::vector<int>> startOfRangeInValueArray;
        enumerable_thread_specific<LocalDistsVector> distToPDLoc;
        enumerable_thread_specific<LocalDistsVector> distFromPDLocToNextStop;
        enumerable_thread_specific<LocalMeetingVerticesVector> meetingVerticesToPDLoc;
        enumerable_thread_specific<LocalMeetingVerticesVector> meetingVerticesFromPDLocToNextStop;

        ThreadSafeSubset vehiclesWithRelevantPDLocs;

        std::vector<std::atomic_int> minDistToPDLoc;
        std::vector<std::atomic_int> minDistFromPDLocToNextStop;

    };

}