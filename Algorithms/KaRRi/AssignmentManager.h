//
// Created by tim on 19.01.24.
//

#pragma once


#include <vector>
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RouteState/RouteStateData.h"

namespace karri {

    template<typename AssignmentFinderT,
            typename CostCalculatorT,
            typename RequestStateInitializerT,
            typename BucketsWrapperT,
            typename RouteStateUpdaterT,
            typename CurVehLocT>
    class AssignmentManager {

    public:
        AssignmentManager(AssignmentFinderT &asgnFinder, CostCalculatorT &calc,
                          InputConfig &config, RequestStateInitializerT &requestStateInitializer,
                          RouteStateData &variableRouteStateData, RouteStateData &fixedRouteStateData,
                          BucketsWrapperT &variableBuckets, BucketsWrapperT &fixedBuckets,
                          RouteStateUpdaterT &varUpdater, CurVehLocT &vehLocator):
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
        std::vector<RequestState<CostCalculatorT>*> &calculateChanges(const Request &req) {
            for (int i = 0; i < currentResult.size(); i++) delete currentResult[i];
            currentResult.clear();
            vehLocator.resetDistances();

            auto *varReqState = createAndInitializeRequestState(req, RouteStateDataType::VARIABLE);
            auto *fixedReqState = createAndInitializeRequestState(req, RouteStateDataType::FIXED);
            searchBestAssignmentOn(variableRouteStateData, variableBuckets, *varReqState);
            searchBestAssignmentOn(fixedRouteStateData, fixedBuckets, *fixedReqState);


            int costBarrier = varReqState->getBestCost() - fixedReqState->getBestCost();
            if (costBarrier > 0) {
                currentResult.push_back(fixedReqState);
                const auto &fixedAssignment = fixedReqState->getBestAssignment();
                const auto reassignableRequests = getReassignableRequests(fixedAssignment.vehicle->vehicleId);

                for (const auto reqId: reassignableRequests) {
                    const auto oldData = oldReqData[reqId];
                    assert(std::get<0>(oldData).requestId == reqId);
                    auto *newReqState = createAndInitializeRequestState(std::get<0>(oldData),
                            RouteStateDataType::VARIABLE, &std::get<2>(oldData));
                    newReqState->blockedVehId = fixedAssignment.vehicle->vehicleId;
                    searchBestAssignmentOn(variableRouteStateData, variableBuckets, *newReqState);
                    currentResult.push_back(newReqState);
                }

                updateOldRequestData();
                return currentResult;
            }

            currentResult.push_back(varReqState);
            updateOldRequestData();
            return currentResult;
        }

    private:

        void updateOldRequestData() {
            const auto mainReqState = currentResult[0];
            PDLoc loc = mainReqState->isNotUsingVehicleBest() ? PDLoc() :  *mainReqState->getBestAssignment().pickup;
            loc.id = 0;
            if (oldReqData.size() < mainReqState->originalRequest.requestId + 1) {
                oldReqData.resize(mainReqState->originalRequest.requestId + 1);
            }
            oldReqData[mainReqState->originalRequest.requestId] = std::tuple(mainReqState->originalRequest, mainReqState->getBestCost(), loc);
            for (int i = 1; i < currentResult.size(); i++) {
                const auto *currReqState = currentResult[i];
                std::get<1>(oldReqData[currReqState->originalRequest.requestId]) = currReqState->getBestCost();
            }
        }

        // Puts all requests picked up after the 0-th stop into a list
        std::vector<int> getReassignableRequests(const int vehId) {
            std::vector<int> result;
            const auto stopIds = variableRouteStateData.stopIdsFor(vehId);
            for (int i = 1; i < variableRouteStateData.numStopsOf(vehId); i++) {
                for (const auto req:  variableRouteStateData.getRequestsPickedUpAt(stopIds[i])) {
                    result.push_back(req);
                }
            }
            return result;
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

        AssignmentFinderT &asgnFinder;

        // Old Requests, their cost and location: oldReqData[req.requestId] = (Request, cost, location)
        std::vector<std::tuple<Request, int, PDLoc>> oldReqData;

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
