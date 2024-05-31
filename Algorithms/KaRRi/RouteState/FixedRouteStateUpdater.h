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

#include "Tools/custom_assertion_levels.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/RouteState/RouteStateData.h"
#include "Algorithms/KaRRi/RouteState/RouteStateUpdater.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT>
    class FixedRouteStateUpdater {

    public:

        FixedRouteStateUpdater(const InputGraphT &inputGraph, const CHEnvT &chEnv, const RouteStateData &varRouteState)
                : inputGraph(inputGraph), ch(chEnv.getCH()), chQuery(chEnv.template getFullCHQuery<>()),
                  varRouteState(varRouteState) {}

        // Call when a vehicle has reached its next stop and _after_ updating the variable route state accordingly.
        // If the reached stop is a variable stop, it becomes fixed.
        // For every pickup performed at the stop, the stops containing the associated dropoffs become fixed as well.
        // In any case, the former previous/current stop in the fixed route is dropped.
        // Returns the indices of any newly inserted stops in the fixed route.
        std::vector<int> updateForReachedStop(RouteStateData &fixedRouteState, int vehId, bool& wasReachedStopFixed) {
            const int reachedStopId = varRouteState.stopIdsFor(vehId)[0];

            const auto &start = fixedRouteState.pos[vehId].start;
            const auto &end = fixedRouteState.pos[vehId].end;
            wasReachedStopFixed = fixedRouteState.numStopsOf(vehId) > 1 && reachedStopId == fixedRouteState.stopIds[start + 1];

            if (wasReachedStopFixed) {
                RouteStateUpdater::removeStartOfCurrentLeg(fixedRouteState, vehId);
            } else {

                // Invalidate data relating to ID of dropped stop
                const int droppedStopId = fixedRouteState.stopIds[start];
                fixedRouteState.stopIdToVehicleId[droppedStopId] = INVALID_ID;
                fixedRouteState.stopIdToLeeway[droppedStopId] = 0;
                fixedRouteState.stopIdToPosition[droppedStopId] = INVALID_INDEX;
                removalOfAllCols(droppedStopId, fixedRouteState.rangeOfRequestsPickedUpAtStop,
                                 fixedRouteState.requestsPickedUpAtStop);
                removalOfAllCols(droppedStopId, fixedRouteState.rangeOfRequestsDroppedOffAtStop,
                                 fixedRouteState.requestsDroppedOffAtStop);
                KASSERT(fixedRouteState.stopIdToIdOfPrevStop[droppedStopId] == INVALID_ID);
                if (fixedRouteState.numStopsOf(vehId) > 1) {
                    fixedRouteState.stopIdToIdOfPrevStop[fixedRouteState.stopIds[start + 1]] = INVALID_ID;
                }

                // Overwrite dropped stop with reached stop.
                fixedRouteState.stopIds[start] = reachedStopId;
                fixedRouteState.stopLocations[start] = varRouteState.stopLocationsFor(vehId)[0];
                fixedRouteState.schedArrTimes[start] = varRouteState.schedArrTimesFor(vehId)[0];
                fixedRouteState.schedDepTimes[start] = varRouteState.schedDepTimesFor(vehId)[0];
                fixedRouteState.maxArrTimes[start] = varRouteState.maxArrTimesFor(vehId)[0];
                fixedRouteState.occupancies[start] = varRouteState.occupanciesFor(vehId)[0];

                const int numDropoffsDiff =
                        varRouteState.numDropoffsPrefixSumFor(vehId)[0] - fixedRouteState.numDropoffsPrefixSum[start];
                fixedRouteState.numDropoffsPrefixSum[start] += numDropoffsDiff;
                for (int i = start + 1; i < end; ++i)
                    fixedRouteState.numDropoffsPrefixSum[i] += numDropoffsDiff;

                const auto newMinSize = reachedStopId + 1;
                if (fixedRouteState.stopIdToIdOfPrevStop.size() < newMinSize) {
                    fixedRouteState.stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                    fixedRouteState.stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                    fixedRouteState.stopIdToLeeway.resize(newMinSize, 0);
                    fixedRouteState.stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                    fixedRouteState.rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                    fixedRouteState.rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
                }
                fixedRouteState.stopIdToPosition[reachedStopId] = 0;
                fixedRouteState.stopIdToVehicleId[reachedStopId] = vehId;
                if (end - start > 1)
                    fixedRouteState.stopIdToIdOfPrevStop[fixedRouteState.stopIds[start + 1]] = reachedStopId;

                LIGHT_KASSERT(varRouteState.rangeOfRequestsDroppedOffAtStop[reachedStopId].end ==
                              varRouteState.rangeOfRequestsDroppedOffAtStop[reachedStopId].start);


                int distToStopAfterReached = INFTY;
                if (fixedRouteState.numStopsOf(vehId) > 1 &&
                    varRouteState.stopIdsFor(vehId)[1] == fixedRouteState.stopIdsFor(vehId)[1]) {
                    // If next stop of fixed and var routes are the same, we can read the distance to the next stop from the var route state.
                    distToStopAfterReached =
                            varRouteState.schedArrTimesFor(vehId)[1] - varRouteState.schedDepTimesFor(vehId)[0];
                } else {
                    // Otherwise, we need to compute the distance here.
                    distToStopAfterReached = computeDistanceBetween(fixedRouteState.stopLocations[0],
                                                                    fixedRouteState.stopLocations[1]);
                }

                fixedRouteState.propagateSchedArrAndDepForward(start + 1, end - 1, distToStopAfterReached);

                fixedRouteState.updateLeeways(vehId);
                if (distToStopAfterReached >= fixedRouteState.maxLegLength) {
                    fixedRouteState.maxLegLength = distToStopAfterReached;
                    fixedRouteState.stopIdOfMaxLegLength = reachedStopId;
                } else if (fixedRouteState.stopIdOfMaxLegLength == droppedStopId) {
                    fixedRouteState.recomputeMaxLegLength();
                }
            }

            const auto &pickupPos = varRouteState.rangeOfRequestsPickedUpAtStop[reachedStopId];
            std::vector<int> indicesOfInsertedStops;
            for (int i = pickupPos.start; i < pickupPos.end; ++i) {
                const int reqId = varRouteState.requestsPickedUpAtStop[i];
                insertion(reachedStopId, reqId,
                          fixedRouteState.rangeOfRequestsPickedUpAtStop, fixedRouteState.requestsPickedUpAtStop);

                insertFixedStopForDropoffOfPickedUpRequest(fixedRouteState, reqId, vehId, indicesOfInsertedStops);
            }

            fixScheduleAndConstraintsForInsertedFixedStops(fixedRouteState, vehId, indicesOfInsertedStops);

            return indicesOfInsertedStops;
        }

        void updateForNewPickupAtCurrentStop(RouteStateData &fixedRouteState, const int reqId, const int vehId) {
            insertion(varRouteState.stopIdsFor(vehId)[0], reqId,
                      fixedRouteState.rangeOfRequestsPickedUpAtStop, fixedRouteState.requestsPickedUpAtStop);

            // Departure time at first stop may have changed due to new pickup
            const auto& start = fixedRouteState.pos[vehId].start;
            const auto& end = fixedRouteState.pos[vehId].end;
            const auto oldDepTime = fixedRouteState.schedDepTimes[start];
            fixedRouteState.schedDepTimes[start] = varRouteState.schedDepTimesFor(vehId)[0];
            if (end > start + 1) {
                const auto distToNext = fixedRouteState.schedArrTimes[start + 1] - oldDepTime;
                fixedRouteState.propagateSchedArrAndDepForward(start + 1, end - 1, distToNext);
            }

            // Insert dropoff of new pickup as fixed stop
            std::vector<int> indicesOfInsertedStops;
            insertFixedStopForDropoffOfPickedUpRequest(fixedRouteState, reqId, vehId, indicesOfInsertedStops);
            fixScheduleAndConstraintsForInsertedFixedStops(fixedRouteState, vehId, indicesOfInsertedStops);
        }

    private:

        void
        findStopPosWithDropoffOfRequestInVarRouteState(const int reqId, const int vehId, int &stopIdx, int &maxArrTime,
                                                       int &numRiders) const {
            const auto &start = varRouteState.pos[vehId].start;
            const auto &end = varRouteState.pos[vehId].end;
            for (int i = start; i < end; ++i) {
                const auto &rangeOfDropoffsAtStop = varRouteState.rangeOfRequestsDroppedOffAtStop[varRouteState.stopIds[i]];
                for (int j = rangeOfDropoffsAtStop.start; j < rangeOfDropoffsAtStop.end; ++j) {
                    if (varRouteState.requestsDroppedOffAtStop[j] == reqId) {
                        stopIdx = i - start;
                        maxArrTime = varRouteState.maxArrTimesOfRequestsDroppedOffAtStop[j];
                        numRiders = varRouteState.numRidersOfRequestsDroppedOffAtStop[j];
                        return;
                    }
                }
            }
            LIGHT_KASSERT(false);
            stopIdx = INVALID_INDEX;
            maxArrTime = -1;
            numRiders = INFTY;
            return;
        }

        void
        insertFixedStopForDropoffOfPickedUpRequest(RouteStateData &fixedRouteState, const int reqId, const int vehId,
                                                   std::vector<int> &indicesOfInsertedStops) {

            int varStopIdx, maxArrTimeOfDropoff, numRiders;
            findStopPosWithDropoffOfRequestInVarRouteState(reqId, vehId, varStopIdx, maxArrTimeOfDropoff, numRiders);
            LIGHT_KASSERT(varStopIdx != INVALID_INDEX);
            const int stopId = varRouteState.stopIdsFor(vehId)[varStopIdx];

            const auto &start = fixedRouteState.pos[vehId].start;
            const auto &end = fixedRouteState.pos[vehId].end;

            // Find fixed stop before varStopIdx:
            int insertIdx = 0;
            for (; insertIdx < end - start - 1; ++insertIdx) {
                const int &stopIdAtNextIdx = fixedRouteState.stopIds[start + insertIdx + 1];
                if (varRouteState.stopIdToPosition[stopIdAtNextIdx] > varStopIdx)
                    break;
            }
            KASSERT(varRouteState.stopIdToPosition[fixedRouteState.stopIds[start + insertIdx]] <= varStopIdx);
            KASSERT(start + insertIdx + 1 == end || varRouteState.stopIdToPosition[fixedRouteState.stopIds[start + insertIdx + 1]] > varStopIdx);

            if (fixedRouteState.stopIds[start + insertIdx] == stopId) {
                // Stop is already fixed. Only add new dropoff:
                fixedRouteState.maxArrTimes[start + insertIdx] = std::max(
                        fixedRouteState.maxArrTimes[start + insertIdx], maxArrTimeOfDropoff);

            } else {
                // Stop has become fixed. Insert new stop into fixedRouteState:
                ++insertIdx;
                stableInsertion(vehId, insertIdx, stopId, fixedRouteState.pos, fixedRouteState.stopIds,
                                fixedRouteState.stopLocations,
                                fixedRouteState.schedArrTimes,
                                fixedRouteState.schedDepTimes,
                                fixedRouteState.vehWaitTimesPrefixSum,
                                fixedRouteState.maxArrTimes,
                                fixedRouteState.occupancies,
                                fixedRouteState.numDropoffsPrefixSum,
                                fixedRouteState.vehWaitTimesUntilDropoffsPrefixSum);
                // schedArrTime and schedDepTime are computed later collectively for all added fixed stops
                fixedRouteState.stopLocations[start + insertIdx] = varRouteState.stopLocationsFor(vehId)[varStopIdx];
                fixedRouteState.maxArrTimes[start + insertIdx] = maxArrTimeOfDropoff;
                fixedRouteState.occupancies[start + insertIdx] = fixedRouteState.occupancies[start + insertIdx - 1];
                fixedRouteState.numDropoffsPrefixSum[start + insertIdx] =
                        fixedRouteState.numDropoffsPrefixSum[start + insertIdx - 1];

                // Fixed route stops only have dropoffs so never need a vehicle wait time
                fixedRouteState.vehWaitTimesPrefixSum[start] = 0;
                fixedRouteState.vehWaitTimesUntilDropoffsPrefixSum[start] = 0;

                const auto newMinSize = stopId + 1;
                if (fixedRouteState.stopIdToIdOfPrevStop.size() < newMinSize) {
                    fixedRouteState.stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                    fixedRouteState.stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                    fixedRouteState.stopIdToLeeway.resize(newMinSize, 0);
                    fixedRouteState.stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                    fixedRouteState.rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                    fixedRouteState.rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
                }

                fixedRouteState.stopIdToVehicleId[stopId] = vehId;
                fixedRouteState.stopIdToIdOfPrevStop[stopId] = fixedRouteState.stopIds[start + insertIdx - 1];
                if (start + insertIdx != end - 1)
                    fixedRouteState.stopIdToIdOfPrevStop[fixedRouteState.stopIds[start + insertIdx + 1]] = stopId;

                indicesOfInsertedStops.push_back(insertIdx);
            }

            for (int idx = start; idx < start + insertIdx; ++idx)
                fixedRouteState.occupancies[idx] += numRiders;
            for (int idx = start + insertIdx; idx < end; ++idx)
                ++fixedRouteState.numDropoffsPrefixSum[idx];

            const int idxOfDropoffRequestData = insertion(stopId,
                                                          reqId,
                                                          fixedRouteState.rangeOfRequestsDroppedOffAtStop,
                                                          fixedRouteState.requestsDroppedOffAtStop,
                                                          fixedRouteState.maxArrTimesOfRequestsDroppedOffAtStop,
                                                          fixedRouteState.numRidersOfRequestsDroppedOffAtStop);
            fixedRouteState.maxArrTimesOfRequestsDroppedOffAtStop[idxOfDropoffRequestData] = maxArrTimeOfDropoff;
            fixedRouteState.numRidersOfRequestsDroppedOffAtStop[idxOfDropoffRequestData] = numRiders;
        }


        void fixScheduleAndConstraintsForInsertedFixedStops(RouteStateData &fixedRouteState, const int vehId,
                                                            std::vector<int> &indicesOfInsertedStops) {

            std::sort(indicesOfInsertedStops.begin(), indicesOfInsertedStops.end());

            const auto &start = fixedRouteState.pos[vehId].start;
            const auto &end = fixedRouteState.pos[vehId].end;
            const auto &stopLocs = fixedRouteState.stopLocations;
            auto &schedArrTimes = fixedRouteState.schedArrTimes;
            auto &schedDepTimes = fixedRouteState.schedDepTimes;
            auto &maxArrTimes = fixedRouteState.maxArrTimes;

            // Assign correct stop positions to all stops after insertions
            for (int i = start; i < end; ++i) {
                fixedRouteState.stopIdToPosition[fixedRouteState.stopIds[i]] = i - start;
            }

            // Reconstruct invariant that schedArrTimes[i] - schedDepTimes[i - 1] == dist(stopLocs[i-1], stopLocs[i])
            int idxInInsertedIndices = 0;
            int oldDepTimeOfPrevStop = schedDepTimes[start];
            for (int i = 1; i < end - start; ++i) {
                if (idxInInsertedIndices < indicesOfInsertedStops.size() &&
                    i - 1 > indicesOfInsertedStops[idxInInsertedIndices]) {
                    LIGHT_KASSERT(i == indicesOfInsertedStops[idxInInsertedIndices] + 2);
                    ++idxInInsertedIndices;
                }

                // If either stop i - 1 or stop i are newly inserted stops, we need to compute the shortest path
                // distance between them from scratch. Otherwise, we can read the distance using the invariant of the
                // old route.
                int dist = INFTY;
                if (idxInInsertedIndices < indicesOfInsertedStops.size() &&
                    (i - 1 == indicesOfInsertedStops[idxInInsertedIndices] ||
                     i == indicesOfInsertedStops[idxInInsertedIndices])) {
                    dist = computeDistanceBetween(stopLocs[start + i - 1], stopLocs[start + i]);
                } else {
                    dist = schedArrTimes[start + i] - oldDepTimeOfPrevStop;
                }
                schedArrTimes[start + i] = schedDepTimes[start + i - 1] + dist;

                // Set dep time of previous stop. Fixed route state only has dropoffs at stops so there are no
                // vehicle wait times and stop time is always stopTime. Memorize old dep time to read distance from
                // old route in next iteration (for the case that i and i + 1 are not newly inserted).
                oldDepTimeOfPrevStop = schedDepTimes[start + i];
                schedDepTimes[start + i] = schedArrTimes[start + i] + InputConfig::getInstance().stopTime;
            }

            // Propagate max arrival times backwards
            for (int l = end - start; l >= 0; --l) {
                const auto distToNext = schedArrTimes[start + l + 1] - schedDepTimes[start + l];
                const auto propagatedMaxArrTime = maxArrTimes[l + 1] - distToNext - InputConfig::getInstance().stopTime;
                maxArrTimes[start + l] = std::min(maxArrTimes[start + l], propagatedMaxArrTime);
            }

            // Update leeways:
            fixedRouteState.updateLeeways(vehId);
        }

        // Computes dist(head(srcLoc), tail(tarLoc)) + l(tarLoc), i.e. the distance between edges srcLoc and tarLoc
        // where tarLoc needs to be traversed in its entirety.
        int computeDistanceBetween(const int srcLoc, const int tarLoc) {
            const auto src = ch.rank(inputGraph.edgeHead(srcLoc));
            const auto tar = ch.rank(inputGraph.edgeTail(tarLoc));
            const auto offset = inputGraph.travelTime(tarLoc);
            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;

        const RouteStateData &varRouteState;

    };

} // end namespace karri
