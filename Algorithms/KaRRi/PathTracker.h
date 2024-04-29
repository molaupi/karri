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
                             const RouteState &routeState, const int fleetSize) :
                inputGraph(inputGraph),
                requestState(requestState),
                routeState(routeState),
                ch(chEnv.getCH()),
                chQuery(chEnv.template getFullCHQuery<>()),
                pathUnpacker(ch),
                eventPos(fleetSize, {0, 0}),
                requestIds(),
                pdLocTypes(),
                pathPos(fleetSize, {0, 0}),
                pathEdges(),
                numCompletedStopsPerVeh(fleetSize, 0),
                vehiclePathLogger(LogManager<LoggerT>::getLogger("vehpaths.csv",
                                                                 "vehicle_id, "
                                                                 "stop_number, "
                                                                 "arr_time, "
                                                                 "dep_time, "
                                                                 "events, "
                                                                 "osm_node_ids_path_to_stop, "
                                                                 "lat_lng_path_to_stop,"
                                                                 "graph_edge_ids_to_stop\n")) {}


        // Updates paths of vehicle for best assignment of pickup and dropoff into the vehicle path.
        // Needs to be called after the insertion is performed on routeState.
        void updateForBestAssignment(const int pickupIndexAfterInsertion, const int dropoffIndexAfterInsertion,
                                     const int numStopsBeforeInsertion, const bool isDropoffAtExistingStop) {
            const auto &asgn = requestState.getBestAssignment();
            assert(asgn.vehicle);
            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsAfterInsertion = routeState.numStopsOf(vehId);
            const auto &stopIds = routeState.stopIdsFor(vehId);
            const auto &stopLocations = routeState.stopLocationsFor(vehId);

            if (routeState.getMaxStopId() >= eventPos.size()) {
                eventPos.resize(routeState.getMaxStopId() + 1, {0, 0});
                pathPos.resize(routeState.getMaxStopId() + 1, {0, 0});
            }

            registerNewPDLocAtStop(stopIds[pickupIndexAfterInsertion], requestState.originalRequest.requestId, PICKUP);
            registerNewPDLocAtStop(stopIds[dropoffIndexAfterInsertion], requestState.originalRequest.requestId,
                                   DROPOFF);

            computePathsForBestAssignment(pickupIndexAfterInsertion, dropoffIndexAfterInsertion);

            // If a new stop was inserted for the pickup, we set the path to the new stop and update the path to the next
            // stop (if the next stop is not the dropoff).
            if (stopLocations[asgn.pickupStopIdx] != asgn.pickup->loc) {
                assert(pathToPickup.empty() || inputGraph.edgeHead(pathToPickup.back()) ==
                                               inputGraph.edgeTail(stopLocations[pickupIndexAfterInsertion]));

                if (asgn.pickupStopIdx == 0 && numStopsBeforeInsertion > 1 &&
                    routeState.schedDepTimesFor(vehId)[0] < requestState.originalRequest.requestTime) {
                    // In this case, the vehicle was rerouted on its way to the next stop and stopLocations[0] has been
                    // changed to the vehicles location at the point in time when it was rerouted (by a call to
                    // routeState.updateStartOfCurrentLeg()).
                    // We traverse the stored path to the stop that was next before the insertion until we find the vehicles
                    // current location, truncate the path from that location and replace it with the path from the vehicles
                    // current location to the inserted stop (which is stored in pathToPickup).
                    assert(pickupIndexAfterInsertion == 1 &&
                           (asgn.pickupStopIdx != asgn.dropoffStopIdx || dropoffIndexAfterInsertion == 2));
                    const auto indexOfOldNextStop = asgn.pickupStopIdx == asgn.dropoffStopIdx ? 3 : 2;
                    auto pathToOldNextStop = getEdgePathTo(stopIds[indexOfOldNextStop]);

                    const auto currentVehLoc = stopLocations[0];

                    // If vehicle has already reached the old next stop, we need to take the whole path up to the old next
                    // stop. Otherwise, the vehicle is somewhere on the path to the old next stop, so we traverse the path
                    // till we find the current vehicle location.
                    std::vector<int> edgePathViaCurrentVehLoc;
                    int i = 0;
                    for (; i < pathToOldNextStop.size() && pathToOldNextStop[i] != currentVehLoc; ++i) {
                        edgePathViaCurrentVehLoc.push_back(pathToOldNextStop[i]);
                    }
                    assert(currentVehLoc == stopLocations[indexOfOldNextStop] ||
                           (i < pathToOldNextStop.size() && pathToOldNextStop[i] == currentVehLoc));
                    edgePathViaCurrentVehLoc.push_back(currentVehLoc);

                    // Concatenate the path to the current vehicles location with the path
                    // from the current vehicles location to the newly inserted stop (pathToPickup).
                    assert(std::none_of(pathToPickup.begin(), pathToPickup.end(),
                                        [&](const int &e) { return e == currentVehLoc; }));
                    assert(pathToPickup.empty() ||
                           inputGraph.edgeHead(currentVehLoc) == inputGraph.edgeTail(pathToPickup[0]));
                    edgePathViaCurrentVehLoc.insert(edgePathViaCurrentVehLoc.end(), pathToPickup.begin(),
                                                    pathToPickup.end());

                    setPathTo(stopIds[pickupIndexAfterInsertion], edgePathViaCurrentVehLoc);
                } else {
                    setPathTo(stopIds[pickupIndexAfterInsertion], pathToPickup);
                }

                if (asgn.pickupStopIdx != asgn.dropoffStopIdx) {
                    setPathTo(stopIds[pickupIndexAfterInsertion + 1], pathFromPickup);
                }
            }

            // If a new stop was inserted for the dropoff, we set the path to the new stop and update the path to the next
            // stop (if one exists).
            if (!isDropoffAtExistingStop) {
                assert(pathToDropoff.empty() || inputGraph.edgeHead(pathToDropoff.back()) ==
                                                inputGraph.edgeTail(stopLocations[dropoffIndexAfterInsertion]));
                setPathTo(stopIds[dropoffIndexAfterInsertion], pathToDropoff);
                if (dropoffIndexAfterInsertion + 1 < numStopsAfterInsertion) {
                    setPathTo(stopIds[dropoffIndexAfterInsertion + 1], pathFromDropoff);
                }
            }
        }


        // Called when a vehicle completes a stop and thus has performed all pickups and dropoffs at the stop.
        // Logs the completed stop along with the previous leg of the vehicle path.
        void logCompletedStop(const Vehicle &veh) {
            const auto idOfCompletedStop = routeState.stopIdsFor(veh.vehicleId)[0];
            const auto stopLoc = routeState.stopLocationsFor(veh.vehicleId)[0];
            const auto arrTime = routeState.schedArrTimesFor(veh.vehicleId)[0];
            const auto depTime = routeState.schedDepTimesFor(veh.vehicleId)[0];

            const auto &edgePathToStop = getEdgePathTo(idOfCompletedStop);
            const auto &requestIdsAtStop = requestIdsAt(idOfCompletedStop);
            const auto &pdLocTypesAtStop = pdLocTypesAt(idOfCompletedStop);
            assert(requestIdsAtStop.size() == pdLocTypesAtStop.size());
            const auto numEvents = requestIdsAtStop.size();

            if (numEvents == 0) {
                KASSERT(edgePathToStop.size() == 0, "No events (finished idling) but non-empty path!", kassert::assert::light);
                invalidateDataFor(idOfCompletedStop);
                return;
            }

            const auto stopCount = numCompletedStopsPerVeh[veh.vehicleId]++;
            vehiclePathLogger << veh.vehicleId << ", " << stopCount << ", " << arrTime << ", " << depTime << ", ";
            for (int i = 0; i < numEvents; ++i) {
                const auto typeStr = pdLocTypesAtStop[i] == PICKUP ? "pickup" : "dropoff";
                vehiclePathLogger << "(" << requestIdsAtStop[i] << " - " << typeStr << ")";
                if (i < numEvents - 1) vehiclePathLogger << " : ";
            }
            vehiclePathLogger << ", ";


            int prevVertex = edgePathToStop.size() == 0 ? INVALID_VERTEX : inputGraph.edgeTail(edgePathToStop[0]);
            unused(prevVertex);
            for (int i = 0; i < edgePathToStop.size(); ++i) {
                const auto e = edgePathToStop[i];
                assert(inputGraph.edgeTail(e) == prevVertex);
                const auto head = inputGraph.edgeHead(e);
                const auto osmNodeId = inputGraph.osmNodeId(head);
                vehiclePathLogger << osmNodeId << " : ";
                prevVertex = head;
            }
            assert(edgePathToStop.size() == 0 || inputGraph.edgeTail(stopLoc) == prevVertex);
            vehiclePathLogger << inputGraph.osmNodeId(inputGraph.edgeHead(stopLoc)) << ", ";

            prevVertex = edgePathToStop.size() == 0 ? INVALID_VERTEX : inputGraph.edgeTail(edgePathToStop[0]);
            for (int i = 0; i < edgePathToStop.size(); ++i) {
                const auto e = edgePathToStop[i];
                assert(inputGraph.edgeTail(e) == prevVertex);
                const auto head = inputGraph.edgeHead(e);
                const auto latLng = inputGraph.latLng(head);
                vehiclePathLogger << latLngForCsv(latLng) << " : ";
                prevVertex = head;
            }
            assert(edgePathToStop.size() == 0 || inputGraph.edgeTail(stopLoc) == prevVertex);
            vehiclePathLogger << latLngForCsv(inputGraph.latLng(inputGraph.edgeHead(stopLoc))) << ", ";

            prevVertex = edgePathToStop.size() == 0 ? INVALID_VERTEX : inputGraph.edgeTail(edgePathToStop[0]);
            for (int i = 0; i < edgePathToStop.size(); ++i) {
                const auto e = edgePathToStop[i];
                assert(inputGraph.edgeTail(e) == prevVertex);
                vehiclePathLogger << e;
                if (i < edgePathToStop.size() - 1)
                    vehiclePathLogger << " : ";
                prevVertex = inputGraph.edgeHead(e);
            }
            assert(edgePathToStop.size() == 0 || inputGraph.edgeTail(stopLoc) == prevVertex);
            vehiclePathLogger << "\n";

            invalidateDataFor(idOfCompletedStop);
        }

    private:

        ConstantVectorRange<int> requestIdsAt(const int stopId) const {
            assert(stopId >= 0);
            assert(stopId < eventPos.size());
            const auto start = eventPos[stopId].start;
            const auto end = eventPos[stopId].end;
            return {requestIds.begin() + start, requestIds.begin() + end};
        }

        ConstantVectorRange<PDLocType> pdLocTypesAt(const int stopId) const {
            assert(stopId >= 0);
            assert(stopId < eventPos.size());
            const auto start = eventPos[stopId].start;
            const auto end = eventPos[stopId].end;
            return {pdLocTypes.begin() + start, pdLocTypes.begin() + end};
        }

        ConstantVectorRange<int> getEdgePathTo(const int stopId) const {
            assert(stopId >= 0);
            assert(stopId < pathPos.size());
            const auto start = pathPos[stopId].start;
            const auto end = pathPos[stopId].end;
            return {pathEdges.begin() + start, pathEdges.begin() + end};
        }

        void registerNewPDLocAtStop(const int stopId, const int requestId, const PDLocType type) {
            assert(stopId >= 0);
            assert(stopId < eventPos.size());
            const auto idx = insertion(stopId, requestId, eventPos, requestIds, pdLocTypes);
            pdLocTypes[idx] = type;
        }

        template<typename PathT>
        void setPathTo(const int stopId, const PathT &edgePath) {
            assert(stopId >= 0);
            assert(stopId < pathPos.size());
            removalOfAllCols(stopId, pathPos, pathEdges);
            int edgeCount = 0;
            for (const auto &e: edgePath) {
                stableInsertion(stopId, edgeCount, e, pathPos, pathEdges);
                ++edgeCount;
            }
        }

        void invalidateDataFor(const int stopId) {
            assert(stopId >= 0);
            assert(stopId < eventPos.size());
            removalOfAllCols(stopId, eventPos, requestIds);
            removalOfAllCols(stopId, pathPos, pathEdges);
        }

        void computePathsForBestAssignment(const int pickupIndexAfterInsertion, const int dropoffIndexAfterInsertion) {

            pathToPickup.clear();
            pathFromPickup.clear();
            pathToDropoff.clear();
            pathFromDropoff.clear();

            auto asgn = requestState.getBestAssignment();
            assert(asgn.vehicle != nullptr);
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &stopLocations = routeState.stopLocationsFor(vehId);

            // Retrieve path to pickup (if not pickup at stop).
            if (stopLocations[asgn.pickupStopIdx] != asgn.pickup->loc) {
                const auto toPickupSrc = inputGraph.edgeHead(stopLocations[pickupIndexAfterInsertion - 1]);
                const auto toPickupTar = inputGraph.edgeTail(stopLocations[pickupIndexAfterInsertion]);
                chQuery.run(ch.rank(toPickupSrc), ch.rank(toPickupTar));
                const auto &toPickupUpPath = chQuery.getUpEdgePath();
                const auto &toPickupDownPath = chQuery.getDownEdgePath();
                pathUnpacker.unpackUpDownPath(toPickupUpPath, toPickupDownPath, pathToPickup);
            }

            // Retrieve path from pickup to next stop (if not dropoff and pickup inserted after same stop).
            if (asgn.pickupStopIdx != asgn.dropoffStopIdx) {
                const auto fromPickupSrc = inputGraph.edgeHead(stopLocations[pickupIndexAfterInsertion]);
                const auto fromPickupTar = inputGraph.edgeTail(stopLocations[pickupIndexAfterInsertion + 1]);
                chQuery.run(ch.rank(fromPickupSrc), ch.rank(fromPickupTar));
                const auto &upPath = chQuery.getUpEdgePath();
                const auto &downPath = chQuery.getDownEdgePath();
                pathUnpacker.unpackUpDownPath(upPath, downPath, pathFromPickup);
            }

            // Retrieve path to dropoff (if not dropoff at existing stop).
            if (stopLocations[asgn.dropoffStopIdx] != asgn.dropoff->loc) {
                const auto toDropoffSrc = inputGraph.edgeHead(stopLocations[dropoffIndexAfterInsertion - 1]);
                const auto toDropoffTar = inputGraph.edgeTail(stopLocations[dropoffIndexAfterInsertion]);
                chQuery.run(ch.rank(toDropoffSrc), ch.rank(toDropoffTar));
                const auto &toDropoffUpPath = chQuery.getUpEdgePath();
                const auto &toDropoffDownPath = chQuery.getDownEdgePath();
                pathUnpacker.unpackUpDownPath(toDropoffUpPath, toDropoffDownPath, pathToDropoff);
            }

            // Retrieve path from dropoff (if dropoff is not new last stop).
            if (dropoffIndexAfterInsertion != routeState.numStopsOf(vehId) - 1) {
                const auto fromDropoffSrc = inputGraph.edgeHead(stopLocations[dropoffIndexAfterInsertion]);
                const auto fromDropoffTar = inputGraph.edgeTail(stopLocations[dropoffIndexAfterInsertion + 1]);
                chQuery.run(ch.rank(fromDropoffSrc), ch.rank(fromDropoffTar));
                const auto &fromDropoffUpPath = chQuery.getUpEdgePath();
                const auto &fromDropoffDownPath = chQuery.getDownEdgePath();
                pathUnpacker.unpackUpDownPath(fromDropoffUpPath, fromDropoffDownPath, pathFromDropoff);
            }
        }


        const InputGraphT &inputGraph;
        const RequestState &requestState;
        const RouteState &routeState;

        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;
        CHPathUnpacker pathUnpacker;


        std::vector<ValueBlockPosition> eventPos;
        std::vector<int> requestIds;
        std::vector<PDLocType> pdLocTypes;

        std::vector<ValueBlockPosition> pathPos;
        std::vector<int> pathEdges;

        std::vector<int> numCompletedStopsPerVeh;

        LoggerT &vehiclePathLogger;


        // Temporarily store paths for new assignment.
        std::vector<int> pathToPickup;
        std::vector<int> pathFromPickup;
        std::vector<int> pathToDropoff;
        std::vector<int> pathFromDropoff;

    };

    struct NoOpPathTracker {

        void updateForBestAssignment(const int, const int, const int, const bool) {}

        void logCompletedStop(const Vehicle &) {}
    };
}