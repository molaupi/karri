//
// Created by tim on 19.01.24.
//

#pragma once


#include <vector>
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RouteState/RouteStateData.h"
#include "Algorithms/KaRRi/BaseObjects/VehicleRouteData.h"

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

        //TODO: Wenn Einfügen auf selbem Fahrzeug der losgelösten Requests möglich sein soll, muss man
        // es möglich sein Route (und Bucket) exchanges rückgängig zu machen (falls Kosten doch größer sind)

        //TODO: Hier wird jetzt nicht beachtete, ob das assignmenht ein Fahrzeug verwendet oder nicht. Da muss dann noch im EventSimulator angepasst werden
        std::vector<RequestState<CostCalculatorT>*> &calculateChanges(const Request &req) {
            for (int i = 0; i < currentResult.size(); i++) delete currentResult[i];
            currentResult.clear();
            vehLocator.resetDistances();

            auto *varReqState = createAndInitializeRequestState(req, RouteStateDataType::VARIABLE, req.requestTime);
            auto *fixedReqState = createAndInitializeRequestState(req, RouteStateDataType::FIXED, req.requestTime);
            searchBestAssignmentOn(variableRouteStateData, variableBuckets, *varReqState);
            searchBestAssignmentOn(fixedRouteStateData, fixedBuckets, *fixedReqState);

            int pickupId, dropoffId;

            int costsSaved = varReqState->getBestCost() - fixedReqState->getBestCost();
            if (costsSaved > 0) {
                const auto fixedAssignment = fixedReqState->getBestAssignment();
                std::vector<VehicleRouteData> prevRouteData;
                std::vector<int> affectedVehicles;
                prevRouteData.push_back(VehicleRouteData(*fixedAssignment.vehicle));
                affectedVehicles.push_back(fixedAssignment.vehicle->vehicleId);

                variableRouteStateData.getRouteDataForVehicle(fixedAssignment.vehicle->vehicleId, prevRouteData[0]);
                currentResult.push_back(fixedReqState);

                std::vector<int> movedReqIds;
                std::vector<int> movedReqDepTimes;
                getMovedRequestsAndDepTime(fixedAssignment.vehicle->vehicleId, movedReqIds, movedReqDepTimes);
                costsSaved += calcCostSavings(fixedAssignment.vehicle->vehicleId, movedReqIds, movedReqDepTimes);



                systemStateUpdater.exchangeRoutesFor(*fixedReqState->getBestAssignment().vehicle);
                systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *fixedReqState, true);

                for (const auto reqId: movedReqIds) {
                    const auto oldData = oldReqData[reqId];
                    auto *newReqState = createAndInitializeRequestState(std::get<0>(oldData),
                            RouteStateDataType::VARIABLE, std::get<0>(oldData).requestTime, &std::get<2>(oldData));
                    searchBestAssignmentOn(variableRouteStateData, variableBuckets, *newReqState);

                    costsSaved -= newReqState->getBestCost();

                    if (costsSaved < 0) {
                        systemStateUpdater.revertChangesOfReoptimizationRun(prevRouteData);
                        for (int i = 0; i < currentResult.size(); i++) delete currentResult[i];
                        currentResult.clear();

                        systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *varReqState);
                        currentResult.push_back(varReqState);
                        updateOldRequestData();
                        return currentResult;
                    }

                    if (!newReqState->isNotUsingVehicleBest() && (std::find(affectedVehicles.begin(), affectedVehicles.end(), newReqState->getBestAssignment().vehicle->vehicleId) == affectedVehicles.end())) {
                        prevRouteData.push_back(VehicleRouteData(*newReqState->getBestAssignment().vehicle));
                        affectedVehicles.push_back(newReqState->getBestAssignment().vehicle->vehicleId);
                        variableRouteStateData.getRouteDataForVehicle(newReqState->getBestAssignment().vehicle->vehicleId, prevRouteData.back());
                    }

                    systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *newReqState, true);

                    currentResult.push_back(newReqState);
                }

                systemStateUpdater.setChangesOfReoptimizationRun();

                updateOldRequestData();
                return currentResult;
            }

            systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *varReqState);

            currentResult.push_back(varReqState);
            updateOldRequestData();
            return currentResult;
        }

    private:

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

                //TODO: Die currTripTime Estimation stimmt hier nicht. Man müsste die depTimes von jedem Request speichern
                for (int reqId: dropoffs) {
                    //const int currTripTimeEstimation = depFromCurrStart - std::get<0>(oldReqData[reqId]).requestTime;
                    savings -= CostFunction::calcTripCostOnly(newRestTripTime);
                    //savings -= CostFunction::calcTripViolationCost(newRestTripTime + currTripTimeEstimation, std::get<4>(oldReqData[reqId]));
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
                //const int currTripTimeEstimation = depFromCurrStart - std::get<0>(oldReqData[occupancies[i]]).requestTime;
                savings += CostFunction::calcTripCostOnly(oldRestTripTIme);
                //savings += CostFunction::calcTripViolationCost(oldRestTripTIme + currTripTimeEstimation, std::get<4>(oldReqData[occupancies[i]]));
            }

            assert(savings >= 0);
            return savings;
        }

        void updateOldRequestData() {
            const auto mainReqState = currentResult[0];
            PDLoc pickup = mainReqState->isNotUsingVehicleBest() ? PDLoc() :  *mainReqState->getBestAssignment().pickup;
            PDLoc dropoff = mainReqState->isNotUsingVehicleBest() ? PDLoc() : *mainReqState->getBestAssignment().dropoff;
            pickup.id = 0;
            dropoff.id = 1;
            if (oldReqData.size() < mainReqState->originalRequest.requestId + 1) {
                oldReqData.resize(mainReqState->originalRequest.requestId + 1);
            }
            oldReqData[mainReqState->originalRequest.requestId] = std::tuple(mainReqState->originalRequest, mainReqState->getBestCost(), pickup, dropoff, mainReqState->getOriginalReqMaxTripTime());
            for (int i = 1; i < currentResult.size(); i++) {
                const auto *currReqState = currentResult[i];
                std::get<1>(oldReqData[currReqState->originalRequest.requestId]) = currReqState->getBestCost();
            }
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

        RequestState<CostCalculatorT> *createAndInitializeRequestState(const Request &req, const RouteStateDataType type, const int now, const PDLoc *setLoc = nullptr) {
            auto *newRequestState = new RequestState<CostCalculatorT>(calc, config, type, now);
            requestStateInitializer.initializeRequestState(req, *newRequestState, setLoc);
            return newRequestState;
        }

        std::vector<RequestState<CostCalculatorT>*> currentResult = {};

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
