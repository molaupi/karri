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
        void updateForReachedStop(RouteStateData &fixedRouteState, int vehId) {
            const int reachedStopId = varRouteState.stopIdsFor(vehId)[0];

            const auto &start = fixedRouteState.pos[vehId].start;
            const auto &end = fixedRouteState.pos[vehId].end;
            const bool wasReachedStopFixed =
                    fixedRouteState.numStopsOf(vehId) > 1 && reachedStopId == fixedRouteState.stopIds[start + 1];

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
                if (end - start > 0)
                    fixedRouteState.stopIdToIdOfPrevStop[fixedRouteState.stopIds[start + 1]] = reachedStopId;

                LIGHT_KASSERT(varRouteState.rangeOfRequestsDroppedOffAtStop[vehId].end ==
                              varRouteState.rangeOfRequestsDroppedOffAtStop[vehId].start);


                int distToStopAfterReached = INFTY;
                if (fixedRouteState.numStopsOf(vehId) > 1 &&
                    varRouteState.stopIdsFor(vehId)[1] == fixedRouteState.stopIdsFor(vehId)[1]) {
                    // If next stop of fixed and var routes are the same, we can read the distance to the next stop from the var route state.
                    distToStopAfterReached =
                            varRouteState.schedArrTimesFor(vehId)[1] - varRouteState.schedDepTimesFor(vehId)[0];
                } else {
                    // Otherwise, we need to compute the distance here:
                    const auto src = ch.rank(inputGraph.edgeHead(fixedRouteState.stopLocations[0]));
                    const auto tar = ch.rank(inputGraph.edgeTail(fixedRouteState.stopLocations[1]));
                    const auto offset = inputGraph.travelTime(fixedRouteState.stopLocations[1]);
                    chQuery.run(src, tar);
                    distToStopAfterReached = chQuery.getDistance() + offset;
                }

                fixedRouteState.propagateSchedArrAndDepForward(start + 1, end, distToStopAfterReached);

                fixedRouteState.updateLeeways(vehId);
                if (distToStopAfterReached >= fixedRouteState.maxLegLength) {
                    fixedRouteState.maxLegLength = distToStopAfterReached;
                    fixedRouteState.stopIdOfMaxLegLength = reachedStopId;
                } else if (fixedRouteState.stopIdOfMaxLegLength == droppedStopId) {
                    fixedRouteState.recomputeMaxLegLength();
                }
            }


            // todo: Insert new fixed stops for all requests picked up at reached stop
            const auto &pickupPos = varRouteState.rangeOfRequestsPickedUpAtStop[reachedStopId];
            for (int i = pickupPos.start; i < pickupPos.end; ++i) {
                insertion(reachedStopId, varRouteState.requestsPickedUpAtStop[i],
                          fixedRouteState.rangeOfRequestsPickedUpAtStop, fixedRouteState.requestsPickedUpAtStop);


            }
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
            for (; insertIdx < end - start; ++insertIdx) {
                const int &stopIdAtInsertIdx = fixedRouteState.stopIds[start + insertIdx];
                if (varRouteState.stopIdToPosition[stopIdAtInsertIdx] > varStopIdx) {
                    insertIdx--;
                    break;
                }
            }
            KASSERT(varRouteState.stopIdToPosition[fixedRouteState.stopIds[start + insertIdx]] <= varStopIdx);
            KASSERT(varRouteState.stopIdToPosition[fixedRouteState.stopIds[start + insertIdx + 1]] > varStopIdx);

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
                fixedRouteState.numDropoffsPrefixSum[start + insertIdx] = fixedRouteState.numDropoffsPrefixSum[start +
                                                                                                               insertIdx -
                                                                                                               1];

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

            // TODO FINISH! Update schedArrTimes, schedDepTimes, maxArrTimes, leeways

            const auto &start = fixedRouteState.pos[vehId].start;
            const auto &end = fixedRouteState.pos[vehId].end;
            int nextInsertedIdx = indicesOfInsertedStops[0];
            for (int i = start + 1; i < end; ++i) {
            }

        }

        const InputGraphT &inputGraph;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;

        const RouteStateData &varRouteState;

    };

} // end namespace karri
