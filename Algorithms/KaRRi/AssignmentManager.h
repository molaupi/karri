//
// Created by tim on 19.01.24.
//

#pragma once


#include <vector>
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RouteState/RouteStateData.h"
#include "Algorithms/KaRRi/BaseObjects/VehicleRouteData.h"
#include "Algorithms/KaRRi/BaseObjects/ChangesWrapper.h"

namespace karri {

    template<
            typename SystemStateUpdaterT,
            typename AssignmentFinderT,
            typename CostCalculatorT,
            typename RequestStateInitializerT,
            typename BucketsWrapperT,
            typename RouteStateUpdaterT,
            typename CurVehLocT>
    class AssignmentManager {

        using CostFunction = typename CostCalculatorT::CostFunction;

    public:
        AssignmentManager(SystemStateUpdaterT &systemStateUpdater, AssignmentFinderT &asgnFinder, CostCalculatorT &calc,
                          InputConfig &config, RequestStateInitializerT &requestStateInitializer,
                          RouteStateData &variableRouteStateData, RouteStateData &fixedRouteStateData,
                          BucketsWrapperT &variableBuckets, BucketsWrapperT &fixedBuckets,
                          RouteStateUpdaterT &varUpdater, CurVehLocT &vehLocator):
                          systemStateUpdater(systemStateUpdater),
                          asgnFinder(asgnFinder),
                          calc(calc),
                          config(config),
                          requestStateInitializer(requestStateInitializer),
                          variableRouteStateData(variableRouteStateData),
                          fixedRouteStateData(fixedRouteStateData),
                          varUpdater(varUpdater),
                          variableBuckets(variableBuckets),
                          fixedBuckets(fixedBuckets),
                          vehLocator(vehLocator) {}

        void calculateChanges(Request &req, ChangesWrapper &changes) {
            vehLocator.resetDistances();
            std::vector<RequestState<CostCalculatorT>*> reqStates;

            // Initial search on fixed and variable routeState
            auto *varReqState = createAndInitializeRequestState(req, RouteStateDataType::VARIABLE, req.requestTime);
            auto *fixedReqState = createAndInitializeRequestState(req, RouteStateDataType::FIXED, req.requestTime);
            searchBestAssignmentOn(variableRouteStateData, variableBuckets, *varReqState);
            searchBestAssignmentOn(fixedRouteStateData, fixedBuckets, *fixedReqState);

            int pickupId, dropoffId;

            // If there are no costs to save the normal assignment is used
            int costsSaved = varReqState->getBestCost() - fixedReqState->getBestCost();
            if (costsSaved <= 0) {
                systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *varReqState);
                addAssignmentData(changes, *varReqState, false);
                reqStates.push_back(varReqState);
                updateOldRequestData(reqStates);
                return;
            }

            // Data needed for the reassignments (current routes of vehicles that are affected by reassignments)
            const auto fixedAssignment = fixedReqState->getBestAssignment();
            std::vector<VehicleRouteData> prevRouteData;
            std::vector<int> affectedVehicles;
            prevRouteData.push_back(VehicleRouteData(*fixedAssignment.vehicle));
            affectedVehicles.push_back(fixedAssignment.vehicle->vehicleId);
            variableRouteStateData.getRouteDataForVehicle(fixedAssignment.vehicle->vehicleId, prevRouteData[0]);

            // Get request data of requests that need to be moved and calculate total cost savings
            std::vector<int> movedReqIds;
            std::vector<int> movedReqDepTimes;
            getMovedRequestsAndDepTime(fixedAssignment.vehicle->vehicleId, movedReqIds, movedReqDepTimes);
            costsSaved += calcCostSavings(fixedAssignment.vehicle->vehicleId, movedReqIds, movedReqDepTimes);

            // Exchange routes of variable routeState with route of fixed routeState and insert best fixed assignment
            systemStateUpdater.exchangeRoutesFor(*fixedReqState->getBestAssignment().vehicle);
            systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *fixedReqState, true);
            addAssignmentData(changes, *fixedReqState, false);
            reqStates.push_back(fixedReqState);

            // Reassignment procedure
            for (const auto reqId: movedReqIds) {
                // Find new assignment for moved request
                auto oldData = oldReqData[reqId];
                assert(std::get<0>(oldData).requestId == reqId);
                auto *newReqState = createAndInitializeRequestState(std::get<0>(oldData),
                                                                    RouteStateDataType::VARIABLE, std::get<0>(oldData).requestTime, &std::get<2>(oldData));
                searchBestAssignmentOn(variableRouteStateData, variableBuckets, *newReqState);
                costsSaved -= newReqState->getBestCost();

                // If no costs are saved, changes are reverted and the initial normal assignment is used
                if (costsSaved < 0) {
                    systemStateUpdater.revertChangesOfReoptimizationRun(prevRouteData);
                    testReset(prevRouteData);
                    systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *varReqState);
                    changes.reassignments.clear();
                    changes.affectedVehicles.clear();
                    addAssignmentData(changes, *varReqState, false);
                    for (auto state: reqStates) delete state;
                    reqStates.clear();
                    reqStates.push_back(varReqState);
                    updateOldRequestData(reqStates);
                    return;
                }

                // Store current route of vehicle that is going to be affected by a reassignment
                if (!newReqState->isNotUsingVehicleBest() && (std::find(affectedVehicles.begin(), affectedVehicles.end(), newReqState->getBestAssignment().vehicle->vehicleId) == affectedVehicles.end())) {
                    prevRouteData.push_back(VehicleRouteData(*newReqState->getBestAssignment().vehicle));
                    affectedVehicles.push_back(newReqState->getBestAssignment().vehicle->vehicleId);
                    variableRouteStateData.getRouteDataForVehicle(newReqState->getBestAssignment().vehicle->vehicleId, prevRouteData.back());
                }

                systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *newReqState, true);
                addAssignmentData(changes, *newReqState, true);
                reqStates.push_back(newReqState);
            }

            // Reoptimization has been successful
            systemStateUpdater.setChangesOfReoptimizationRun();
            changes.affectedVehicles = affectedVehicles;
            changes.costsSaved = costsSaved;
            updateOldRequestData(reqStates);
        }

    private:

        void addAssignmentData(ChangesWrapper &changes, const RequestState<CostCalculatorT> &reqState, const bool reassignment) {
            AssignmentData data{};

            data.requestId = reqState.originalRequest.requestId;
            data.requestTime = reqState.originalRequest.requestTime;
            data.cost = reqState.getBestCost();
            data.isUsingVehicle = !reqState.isNotUsingVehicleBest();
            data.assignedVehicleId = (data.isUsingVehicle ? reqState.getBestAssignment().vehicle->vehicleId : -1);
            data.walkingTimeFromDropoff = (data.isUsingVehicle ? reqState.getBestAssignment().dropoff->walkingDist : reqState.getNotUsingVehicleDist());
            data.walkingTimeToPickup = (data.isUsingVehicle ? reqState.getBestAssignment().pickup->walkingDist : -1);

            if (reassignment) {
                changes.reassignments.push_back(data);
                return;
            }
            changes.initialAssignment = data;
        }

        void testReset(std::vector<VehicleRouteData> &data) {
            for (const auto &route: data) {
                const int vehId = route.veh.vehicleId;
                const int numStops = route.stopIds.size();

                assert(numStops == variableRouteStateData.numStopsOf(vehId));

                for (int i = 0; i < numStops; i++) {
                    assert(variableRouteStateData.stopIdsFor(vehId)[i] == route.stopIds[i]);
                    assert(variableRouteStateData.stopLocationsFor(vehId)[i] == route.stopLocations[i]);
                    assert(variableRouteStateData.schedArrTimesFor(vehId)[i] == route.schedArrTimes[i]);
                    assert(variableRouteStateData.schedDepTimesFor(vehId)[i] == route.schedDepTimes[i]);
                    assert(variableRouteStateData.maxArrTimesFor(vehId)[i] == route.maxArrTimes[i]);
                    assert(variableRouteStateData.occupanciesFor(vehId)[i] == route.occupancies[i]);
                    assert(variableRouteStateData.vehWaitTimesPrefixSumFor(vehId)[i] == route.vehWaitTimesPrefixSum[i]);
                    assert(variableRouteStateData.vehWaitTimesUntilDropoffsPrefixSumsFor(vehId)[i] == route.vehWaitTimesUntilDropoffsPrefixSum[i]);
                    assert(variableRouteStateData.numDropoffsPrefixSumFor(vehId)[i] == route.numDropoffsPrefixSum[i]);
                    // Pickup and dropoffs are not checked

                    // Data concerning stopIds
                    if (i == 0) {
                        assert(variableRouteStateData.idOfPreviousStopOf(route.stopIds[i]) == INVALID_ID);
                    } else {
                        assert(variableRouteStateData.idOfPreviousStopOf(route.stopIds[i]) == route.stopIds[i - 1]);
                    }
                    assert(variableRouteStateData.stopPositionOf(route.stopIds[i]) == i);
                    assert(variableRouteStateData.leewayOfLegStartingAt(route.stopIds[i]) == route.leeways[i]);
                    assert(variableRouteStateData.vehicleIdOf(route.stopIds[i]) == vehId);
                }
            }
        }

        int calcCostSavings(const int vehId, const std::vector<int> &movedReqIds, const std::vector<int> &movedReqDepTimes) {
            const int numFixedStops = fixedRouteStateData.numStopsOf(vehId);
            const int numVariableStops = variableRouteStateData.numStopsOf(vehId);
            const auto &varStopIds = variableRouteStateData.stopIdsFor(vehId);

            int savings = 0;

            // Detour savings
            savings += CostFunction::
                    calcVehicleCost(variableRouteStateData.schedDepTimesFor(vehId)[numVariableStops - 1] - fixedRouteStateData.schedDepTimesFor(vehId)[numFixedStops - 1]);
            assert(savings >= 0);

            // Trip time savings for occupancies of vehicle
            savings += calcTripTimeSavings(vehId);

            // Old walking time of moved requests
            for (const int reqId: movedReqIds) {
                savings += CostFunction::calcWalkingCost(std::get<2>(oldReqData[reqId]).walkingDist + std::get<3>(oldReqData[reqId]).walkingDist, -1);
            }

            // Old trip time of moved requests (with penalty)
            for (int i = 2; i < numVariableStops; i++) {
                for (const int dropoffReqId: variableRouteStateData.getRequestsDroppedOffAt(varStopIds[i])) {
                    const auto it = std::find(movedReqIds.begin(), movedReqIds.end(), dropoffReqId);
                    if (it != movedReqIds.end()) {
                        const int reqIdIndex = it - movedReqIds.begin();
                        const int tripTime = variableRouteStateData.schedArrTimesFor(vehId)[i]- movedReqDepTimes[reqIdIndex];
                        savings += CostFunction::calcTripCostOnly(tripTime);
                        savings += CostFunction::calcTripViolationCost(tripTime, std::get<4>(oldReqData[dropoffReqId]));
                    }
                }
            }

            // Saved wait time penalties of moved requests
            for (int i = 0; i < movedReqIds.size(); i++) {
                const int reqTime = std::get<0>(oldReqData[movedReqIds[i]]).requestTime;
                savings += CostFunction::calcWaitViolationCost(movedReqDepTimes[i], reqTime, config.maxWaitTime);
            }

            return savings;
        }

        int calcTripTimeSavings(const int vehId) {
            int savings = 0;
            std::vector<int> occupancies;
            std::vector<int> dropOffStopIds;

            const int depFromCurrStart = fixedRouteStateData.schedDepTimesFor(vehId)[0];

            for (int i = 1; i < fixedRouteStateData.numStopsOf(vehId); i++) {
                const int currStopId = fixedRouteStateData.stopIdsFor(vehId)[i];
                const auto &dropoffs = fixedRouteStateData.getRequestsDroppedOffAt(currStopId);
                const int newRestTripTime = fixedRouteStateData.schedArrTimesFor(vehId)[i] - depFromCurrStart;

                for (int reqId: dropoffs) {
                    const int currTripTimeEstimation = depFromCurrStart - std::get<0>(oldReqData[reqId]).requestTime;
                    savings -= CostFunction::calcTripCostOnly(newRestTripTime);
                    savings -= CostFunction::calcTripViolationCost(newRestTripTime + currTripTimeEstimation, std::get<4>(oldReqData[reqId]));
                    occupancies.push_back(reqId);
                    dropOffStopIds.push_back(currStopId);
                }
            }

            assert(savings <= 0);

            const auto &variableStopIds = variableRouteStateData.stopIdsFor(vehId);
            int currVarIndex = 1;
            for (int i = 0; i < occupancies.size(); i++) {
                while (variableStopIds[currVarIndex] != dropOffStopIds[i]) currVarIndex++;
                const int oldRestTripTIme = variableRouteStateData.schedArrTimesFor(vehId)[currVarIndex] - depFromCurrStart;
                const int currTripTimeEstimation = depFromCurrStart - std::get<0>(oldReqData[occupancies[i]]).requestTime;
                savings += CostFunction::calcTripCostOnly(oldRestTripTIme);
                savings += CostFunction::calcTripViolationCost(oldRestTripTIme + currTripTimeEstimation, std::get<4>(oldReqData[occupancies[i]]));
            }

            assert(savings >= 0);
            return savings;
        }

        void updateOldRequestData(std::vector<RequestState<CostCalculatorT>*> &currentResult) {
            const auto mainReqState = currentResult[0];
            int reqId = mainReqState->originalRequest.requestId;
            PDLoc pickup = (mainReqState->isNotUsingVehicleBest() ? PDLoc() :  *mainReqState->getBestAssignment().pickup);
            PDLoc dropoff = (mainReqState->isNotUsingVehicleBest() ? PDLoc() : *mainReqState->getBestAssignment().dropoff);
            pickup.id = 0;
            dropoff.id = 1;
            if (oldReqData.size() < mainReqState->originalRequest.requestId + 1) {
                oldReqData.resize(mainReqState->originalRequest.requestId + 1);
            }
            oldReqData[reqId] = std::tuple(mainReqState->originalRequest, mainReqState->getBestCost(), pickup, dropoff, mainReqState->getOriginalReqMaxTripTime());
            assert(std::get<0>(oldReqData[reqId]).requestId == reqId);
            for (int i = 1; i < currentResult.size(); i++) {
                const auto *currReqState = currentResult[i];
                reqId = currReqState->originalRequest.requestId;
                std::get<1>(oldReqData[reqId]) = currReqState->getBestCost();
                if (!currReqState->isNotUsingVehicleBest()) {
                    PDLoc newPickup = *currReqState->getBestAssignment().pickup;
                    PDLoc newDropoff = *currReqState->getBestAssignment().dropoff;
                    newPickup.id = 0;
                    std::get<2>(oldReqData[reqId]) = newPickup;
                    std::get<3>(oldReqData[reqId]) = newDropoff;
                }
                assert(std::get<0>(oldReqData[reqId]).requestId == reqId);
            }
            for (auto state: currentResult) delete state;
            currentResult.clear();
        }

        // Puts all requests picked up after the 0-th stop into a list
        void getMovedRequestsAndDepTime(const int vehId, std::vector<int> &reqIds, std::vector<int> &depTimes) {
            const auto stopIds = variableRouteStateData.stopIdsFor(vehId);
            for (int i = 1; i < variableRouteStateData.numStopsOf(vehId); i++) {
                for (const auto req:  variableRouteStateData.getRequestsPickedUpAt(stopIds[i])) {
                    reqIds.push_back(req);
                    depTimes.push_back(variableRouteStateData.schedDepTimesFor(vehId)[i]);
                }
            }
        }

        void searchBestAssignmentOn(RouteStateData &data, BucketsWrapperT &buckets, RequestState<CostCalculatorT> &reqState) {
            calc.exchangeRouteStateData(data);
            asgnFinder.findBestAssignment(reqState, data, buckets);
        }

        RequestState<CostCalculatorT> *createAndInitializeRequestState(Request &req, const RouteStateDataType type, const int now, const PDLoc *setLoc = nullptr) {
            auto *newRequestState = new RequestState<CostCalculatorT>(calc, config, type, now);
            requestStateInitializer.initializeRequestState(req, *newRequestState, setLoc);
            return newRequestState;
        }

        SystemStateUpdaterT &systemStateUpdater;

        AssignmentFinderT &asgnFinder;

        // Old Requests, their cost and location: oldReqData[req.requestId] = (Request, cost, pickup, dropoff, maxTripTime)
        std::vector<std::tuple<Request, int, PDLoc, PDLoc, int>> oldReqData;

        // Data needed for RequestStates
        CostCalculatorT &calc;
        InputConfig &config;
        RequestStateInitializerT &requestStateInitializer;

        // Different RouteStates and corresponding elliptic buckets
        RouteStateData &variableRouteStateData;
        RouteStateData &fixedRouteStateData;

        RouteStateUpdaterT &varUpdater;

        BucketsWrapperT &variableBuckets;
        BucketsWrapperT &fixedBuckets;

        CurVehLocT &vehLocator;
    };

}
