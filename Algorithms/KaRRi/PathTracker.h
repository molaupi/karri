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


#include <vector>
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Tools/Logging/NullLogger.h"
#include "Tools/Logging/LogManager.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT, typename LoggerT = NullLogger>
    class PathTracker {

    public:

        explicit PathTracker(const InputGraphT &inputGraph, const CHEnvT &chEnv, const RequestState &requestState,
                             const RouteState &routeState, const Fleet &fleet) :
                inputGraph(inputGraph),
                requestState(requestState),
                routeState(routeState),
                ch(chEnv.getCH()),
                chQuery(chEnv.template getFullCHQuery<>()),
                pathUnpacker(ch),
                eventIndexRange(fleet.size(), {0, 0}),
                requestIds(),
                pdLocTypes(),
                locsInCurLegIndexRange(fleet.size(), {0, 0}),
                locsInCurLeg(),
                numCompletedStopsPerVeh(fleet.size(), 0),
                lastEdgeOfPrevLeg(fleet.size()),
                vehiclePathLogger(LogManager<LoggerT>::getLogger("vehpaths.csv",
                                                                 "vehicle_id, "
                                                                 "stop_number, "
                                                                 "arr_time, "
                                                                 "dep_time, "
                                                                 "events, "
                                                                 "osm_node_ids_path_to_stop, "
                                                                 "lat_lng_path_to_stop,"
                                                                 "graph_edge_ids_to_stop\n")) {
            for (const auto &veh: fleet) {
                lastEdgeOfPrevLeg[veh.vehicleId] = veh.initialLocation;
                KASSERT(lastEdgeOfPrevLeg[veh.vehicleId] > 0 &&
                        lastEdgeOfPrevLeg[veh.vehicleId] < inputGraph.numEdges(), "", kassert::assert::light);
                resetLocsOnCurLegFor(veh.vehicleId, veh.initialLocation);
            }
        }

        // Store a location along a vehicle's current route leg that is needed for reconstructing the path of the leg
        // later.
        // Ordinarily, a route only has a single location along its current leg, namely the location of the previous
        // stop.
        // However, if a vehicle is rerouted at its current location, the location of its previous stop (stop 0) is
        // changed to its current location in the route state. In order to later be able to reconstruct the full path
        // including the route from the old stop 0 to the current location, we need to store the old location of
        // stop 0. This may happen multiple times before finally reaching stop 1 and logging the route leg, so we
        // store a list of stop locations that the vehicle visited in its current leg.
        void registerLocAlongCurrentLeg(const int vehId, const int loc) {
            assert(vehId >= 0);
            assert(vehId < locsInCurLegIndexRange.size());
            const auto range = locsInCurLegIndexRange[vehId];
            int curNumLocsOnLeg = range.end - range.start;
            stableInsertion(vehId, curNumLocsOnLeg, loc, locsInCurLegIndexRange, locsInCurLeg);
        }


        // Updates paths of vehicle for best assignment of pickup and dropoff into the vehicle path.
        // Needs to be called after the insertion is performed on routeState.
        void registerPdEventsForBestAssignment(const int pickupIndexAfterInsertion,
                                               const int dropoffIndexAfterInsertion) {
            const auto &asgn = requestState.getBestAssignment();
            assert(asgn.vehicle);
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &stopIds = routeState.stopIdsFor(vehId);

            if (routeState.getMaxStopId() >= eventIndexRange.size()) {
                eventIndexRange.resize(routeState.getMaxStopId() + 1, {0, 0});
                locsInCurLegIndexRange.resize(routeState.getMaxStopId() + 1, {0, 0});
            }

            registerNewPDLocAtStop(stopIds[pickupIndexAfterInsertion], requestState.originalRequest.requestId, PICKUP);
            registerNewPDLocAtStop(stopIds[dropoffIndexAfterInsertion], requestState.originalRequest.requestId,
                                   DROPOFF);
        }


        // Called when a vehicle completes a stop and thus has performed all pickups and dropoffs at the stop.
        // Logs the completed stop along with the previous leg of the vehicle path.
        void logCompletedStop(const Vehicle &veh) {
            const auto idOfCompletedStop = routeState.stopIdsFor(veh.vehicleId)[0];
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[0];
            const auto arrTime = routeState.schedArrTimesFor(veh.vehicleId)[0];
            const auto depTime = routeState.schedDepTimesFor(veh.vehicleId)[0];

            const auto &requestIdsAtStop = requestIdsAt(idOfCompletedStop);
            const auto &pdLocTypesAtStop = pdLocTypesAt(idOfCompletedStop);
            assert(requestIdsAtStop.size() == pdLocTypesAtStop.size());
            const auto numEvents = requestIdsAtStop.size();
            KASSERT(numEvents > 0, "No events at stop with id " << idOfCompletedStop, kassert::assert::light);

//            if (numEvents == 0) {
//                KASSERT(edgePathToStop.size() == 0, "No events (finished idling) but non-empty path!",
//                        kassert::assert::light);
//                invalidateDataFor(idOfCompletedStop);
//                return;
//            }

            const auto stopCount = numCompletedStopsPerVeh[veh.vehicleId]++;

            vehiclePathLogger << veh.vehicleId << ", " << stopCount << ", " << arrTime << ", " << depTime << ", ";
            for (int i = 0; i < numEvents; ++i) {
                const auto typeStr = pdLocTypesAtStop[i] == PICKUP ? "pickup" : "dropoff";
                vehiclePathLogger << "(" << requestIdsAtStop[i] << " - " << typeStr << ")";
                if (i < numEvents - 1) vehiclePathLogger << " : ";
            }
            vehiclePathLogger << ", ";

            reconstructPathOfRouteLeg(veh.vehicleId); // Writes path into legPath

            KASSERT(legPath.size() == 0 || legPath[legPath.size() - 1] == stopLoc, "", kassert::assert::light);

            int prevVertex = inputGraph.edgeHead(lastEdgeOfPrevLeg[veh.vehicleId]);
            unused(prevVertex);
            for (int i = 0; i < legPath.size(); ++i) {
                const auto e = legPath[i];
                KASSERT(inputGraph.edgeTail(e) == prevVertex, "", kassert::assert::light);
                const auto head = inputGraph.edgeHead(e);
                const auto osmNodeId = inputGraph.osmNodeId(head);
                vehiclePathLogger << osmNodeId << (i < legPath.size() - 1 ? " : " : ", ");
                prevVertex = head;
            }

            prevVertex = inputGraph.edgeHead(lastEdgeOfPrevLeg[veh.vehicleId]);
            for (int i = 0; i < legPath.size(); ++i) {
                const auto e = legPath[i];
                KASSERT(inputGraph.edgeTail(e) == prevVertex, "", kassert::assert::light);
                const auto head = inputGraph.edgeHead(e);
                const auto latLng = inputGraph.latLng(head);
                vehiclePathLogger << latLngForCsv(latLng) << (i < legPath.size() - 1 ? " : " : ", ");
                prevVertex = head;
            }

            prevVertex = inputGraph.edgeHead(lastEdgeOfPrevLeg[veh.vehicleId]);
            for (int i = 0; i < legPath.size(); ++i) {
                const auto e = legPath[i];
                KASSERT(inputGraph.edgeTail(e) == prevVertex, "", kassert::assert::light);
                vehiclePathLogger << e << (i < legPath.size() - 1 ? " : " : "\n");
                prevVertex = inputGraph.edgeHead(e);
            }

            if (!legPath.empty()) {
                lastEdgeOfPrevLeg[veh.vehicleId] = legPath.back();
                KASSERT(lastEdgeOfPrevLeg[veh.vehicleId] >= 0 &&
                        lastEdgeOfPrevLeg[veh.vehicleId] < inputGraph.numEdges(), "Last edge on leg path was " << legPath.back() << " while graph has only " << inputGraph.numEdges() << " edges.", kassert::assert::light);
            }

            invalidateEventDataFor(idOfCompletedStop);
            resetLocsOnCurLegFor(veh.vehicleId, stopLoc);
        }

    private:

        void reconstructPathOfRouteLeg(const int vehId) {
            // Called when departing stop 0, so we reconstruct the path of the leg to stop 0.
            const auto &stopLoc = routeState.stopLocationsFor(vehId)[0];
            legPath.clear();

            const auto &locsAlongLeg = locsAlongCurLegOf(vehId);
            for (int i = 0; i < locsAlongLeg.size() - 1; ++i) {
                const int fromLoc = locsAlongLeg[i];
                const int toLoc = locsAlongLeg[i + 1];
                computePathBetweenEdgesAndAppend(fromLoc, toLoc, legPath);
            }
            computePathBetweenEdgesAndAppend(locsAlongLeg[locsAlongLeg.size() - 1], stopLoc, legPath);
        }

        void computePathBetweenEdgesAndAppend(const int from, const int to, std::vector<int> &path) {
            if (from == to)
                return;
            const auto src = inputGraph.edgeHead(from);
            const auto tar = inputGraph.edgeTail(to);
            chQuery.run(ch.rank(src), ch.rank(tar));
            const auto &upPath = chQuery.getUpEdgePath();
            const auto &downPath = chQuery.getDownEdgePath();
            pathUnpacker.unpackUpDownPath(upPath, downPath, path);
            path.push_back(to);
        }

        ConstantVectorRange<int> locsAlongCurLegOf(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < locsInCurLegIndexRange.size());
            const auto start = locsInCurLegIndexRange[vehId].start;
            const auto end = locsInCurLegIndexRange[vehId].end;
            return {locsInCurLeg.begin() + start, locsInCurLeg.begin() + end};
        }

        bool isEdgeIncident(const int v, const int e) const {
            FORALL_INCIDENT_EDGES(inputGraph, v, inc) {
                if (inc == e)
                    return true;
            }
            return false;
        }

        ConstantVectorRange<int> requestIdsAt(const int stopId) const {
            assert(stopId >= 0);
            assert(stopId < eventIndexRange.size());
            const auto start = eventIndexRange[stopId].start;
            const auto end = eventIndexRange[stopId].end;
            return {requestIds.begin() + start, requestIds.begin() + end};
        }

        ConstantVectorRange<PDLocType> pdLocTypesAt(const int stopId) const {
            assert(stopId >= 0);
            assert(stopId < eventIndexRange.size());
            const auto start = eventIndexRange[stopId].start;
            const auto end = eventIndexRange[stopId].end;
            return {pdLocTypes.begin() + start, pdLocTypes.begin() + end};
        }

        void registerNewPDLocAtStop(const int stopId, const int requestId, const PDLocType type) {
            assert(stopId >= 0);
            assert(stopId < eventIndexRange.size());
            const auto idx = insertion(stopId, requestId, eventIndexRange, requestIds, pdLocTypes);
            pdLocTypes[idx] = type;
        }

        void invalidateEventDataFor(const int stopId) {
            assert(stopId >= 0);
            assert(stopId < eventIndexRange.size());
            removalOfAllCols(stopId, eventIndexRange, requestIds);
        }

        void resetLocsOnCurLegFor(const int vehId, const int stopLoc) {
            assert(vehId >= 0);
            assert(vehId < locsInCurLegIndexRange.size());
            removalOfAllCols(vehId, locsInCurLegIndexRange, locsInCurLeg);
            insertion(vehId, stopLoc, locsInCurLegIndexRange, locsInCurLeg);
        }


        const InputGraphT &inputGraph;
        const RequestState &requestState;
        const RouteState &routeState;

        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;
        CHPathUnpacker pathUnpacker;


        std::vector<ValueBlockPosition> eventIndexRange;
        std::vector<int> requestIds;
        std::vector<PDLocType> pdLocTypes;

        std::vector<ValueBlockPosition> locsInCurLegIndexRange;
        std::vector<int> locsInCurLeg;

        std::vector<int> numCompletedStopsPerVeh;
        std::vector<int> lastEdgeOfPrevLeg;

        LoggerT &vehiclePathLogger;


        // Temporarily store paths for completed leg.
        std::vector<int> legPath;

    };

    struct NoOpPathTracker {

        void updateForBestAssignment(const int, const int, const int, const bool) {}

        void logCompletedStop(const Vehicle &) {}
    };
}