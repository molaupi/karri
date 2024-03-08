//
// Created by tim on 19.01.24.
//

#pragma once


#include <vector>
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RouteState/RouteStateData.h"

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

        //TODO: Inserts müssen rückgängig gemacht werden können, falls Kosten doch zu hoch sind

        //TODO: Hier wird jetzt nicht beachtete, ob das assignmenht ein Fahrzeug verwendet oder nicht. Da muss dann noch im EventSimulator angepasst werden
        std::vector<RequestState<CostCalculatorT>*> &calculateChanges(const Request &req) {
            for (int i = 0; i < currentResult.size(); i++) delete currentResult[i];
            currentResult.clear();
            vehLocator.resetDistances();

            auto *varReqState = createAndInitializeRequestState(req, RouteStateDataType::VARIABLE);
            auto *fixedReqState = createAndInitializeRequestState(req, RouteStateDataType::FIXED);
            searchBestAssignmentOn(variableRouteStateData, variableBuckets, *varReqState);
            searchBestAssignmentOn(fixedRouteStateData, fixedBuckets, *fixedReqState);

            int pickupId, dropoffId;

            int costBarrier = varReqState->getBestCost() - fixedReqState->getBestCost();
            if (costBarrier > 0) {

                currentResult.push_back(fixedReqState);

                const auto &fixedAssignment = fixedReqState->getBestAssignment();
                std::vector<int> movedReqIds;
                std::vector<int> movedReqDepTimes;
                getMovedRequestsAndDepTime(fixedAssignment.vehicle->vehicleId, req.requestId, movedReqIds, movedReqDepTimes); // TODO: Muss vor exchange routes aufegrufen werden. Sonst immer leer

                systemStateUpdater.exchangeRoutesFor(*fixedReqState->getBestAssignment().vehicle);
                systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *fixedReqState);

                for (const auto reqId: movedReqIds) {
                    const auto oldData = oldReqData[reqId];
                    auto *newReqState = createAndInitializeRequestState(std::get<0>(oldData),
                            RouteStateDataType::VARIABLE, &std::get<2>(oldData));
                    searchBestAssignmentOn(variableRouteStateData, variableBuckets, *newReqState);

                    systemStateUpdater.insertBestAssignment(pickupId, dropoffId, *newReqState);

                    currentResult.push_back(newReqState);
                }

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
            savings += variableRouteStateData.schedDepTimesFor(vehId)[numVariableStops - 1] - fixedRouteStateData.schedDepTimesFor(vehId)[numFixedStops - 1];

            // Trip time savings for occupancies of vehicle
            savings += calcTripTimeSavings(vehId);

            // Old walking time of moved requests
            for (const int reqId: movedReqIds) {
                savings += std::get<2>(oldReqData[reqId]).walkingDist + std::get<3>(oldReqData[reqId]).walkingDist;
            }

            for (int i = 2; i < numVariableStops; i++) {
                for (const int dropoffReqId: variableRouteStateData.getRequestsDroppedOffAt(varStopIds[i])) {
                    const auto it = std::find(movedReqIds.begin(), movedReqIds.end(), dropoffReqId);
                    if (it != movedReqIds.end()) {
                        const int reqIdIndex = it - movedReqIds.begin();
                        const int tripTime = variableRouteStateData.schedArrTimesFor(vehId)[i]- movedReqDepTimes[reqIdIndex];
                        savings += tripTime; // Old trip time of moved requests
                        //TODO: Trip time penalty (originalReqMaxTripTime zwischen speichern und in costfunction alternative methode, die wert direkt annimmt)
                    }
                }
            }

            // Saved wait time penalties
            // TODO: Berechnung in CostFunction separieren sodass man werte einfach übergebn kann

            return savings;
        }

        int calcTripTimeSavings(const int vehId) {
            int savings = 0;
            std::vector<int> occupancies;
            const int depFromStart = fixedRouteStateData.schedDepTimesFor(vehId)[0];

            for (int i = 1; i < fixedRouteStateData.numStopsOf(vehId); i++) {
                const auto &dropoffs = fixedRouteStateData.getRequestsDroppedOffAt(fixedRouteStateData.stopIdsFor(vehId)[i]);
                const int newTripTime = fixedRouteStateData.schedArrTimesFor(vehId)[i] - depFromStart;
                savings -= dropoffs.size() * newTripTime;
                //TODO: Penalties abziehen:    savings -= dropoffs.size() * penalty(newTripTime)
                occupancies.insert(occupancies.end(), dropoffs.begin(), dropoffs.end());
            }

            for (int i = 1; i < variableRouteStateData.numStopsOf(vehId); i++) {
                const int currStopId = variableRouteStateData.stopIdsFor(vehId)[i];
                const auto &dropoffs = variableRouteStateData.getRequestsDroppedOffAt(currStopId);
                while (!occupancies.empty() && std::find(dropoffs.begin(), dropoffs.end(), occupancies[0]) != dropoffs.end()) {
                    const int oldTripTime = variableRouteStateData.schedArrTimesFor(vehId)[i] - depFromStart;
                    savings += oldTripTime;
                    // TODO: Analog tripTime penalty oben drauf rechnen
                    occupancies.erase(occupancies.begin());
                }
            }
            assert(occupancies.empty() && savings >= 0);
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
            oldReqData[mainReqState->originalRequest.requestId] = std::tuple(mainReqState->originalRequest, mainReqState->getBestCost(), pickup, dropoff);
            for (int i = 1; i < currentResult.size(); i++) {
                const auto *currReqState = currentResult[i];
                std::get<1>(oldReqData[currReqState->originalRequest.requestId]) = currReqState->getBestCost();
            }
        }

        // Puts all requests picked up after the 0-th stop into a list
        void getMovedRequestsAndDepTime(const int vehId, const int initiatorRequestId, std::vector<int> &reqIds, std::vector<int> &depTimes) {
            const auto stopIds = variableRouteStateData.stopIdsFor(vehId);
            for (int i = 1; i < variableRouteStateData.numStopsOf(vehId); i++) {
                for (const auto req:  variableRouteStateData.getRequestsPickedUpAt(stopIds[i])) {
                    if (initiatorRequestId != req) {
                        reqIds.push_back(req);
                        depTimes.push_back(variableRouteStateData.schedDepTimesFor(vehId)[i]);
                    }
                }
            }
        }

        void searchBestAssignmentOn(RouteStateData &data, BucketsWrapperT &buckets, RequestState<CostCalculatorT> &reqState) {
            calc.exchangeRouteStateData(data);
            asgnFinder.findBestAssignment(reqState, data, buckets);
        }

        RequestState<CostCalculatorT> *createAndInitializeRequestState(const Request &req, const RouteStateDataType type, const PDLoc *setLoc = nullptr) {
            auto *newRequestState = new RequestState<CostCalculatorT>(calc, config, type);
            requestStateInitializer.initializeRequestState(req, *newRequestState, setLoc);
            return newRequestState;
        }

        std::vector<RequestState<CostCalculatorT>*> currentResult = {};

        SystemStateUpdaterT &systemStateUpdater;

        AssignmentFinderT &asgnFinder;

        // Old Requests, their cost and location: oldReqData[req.requestId] = (Request, cost, pickup, dropoff)
        std::vector<std::tuple<Request, int, PDLoc, PDLoc>> oldReqData;

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