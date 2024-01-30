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
            if (currentResult.size() > 0) {
                delete currentResult[0];
            }
            currentResult.clear();
            auto *varReqState = createAndInitializeRequestState(req);
            auto *fixedReqState = createAndInitializeRequestState(req);
            searchBestAssignmentOn(variableRouteStateData, variableBuckets, *varReqState);
            searchBestAssignmentOn(fixedRouteStateData, fixedBuckets, *fixedReqState);

            delete fixedReqState;

            const int varVehId = varReqState->getBestAssignment().vehicle->vehicleId;
            //TODO: Können hier Probleme auftreten mit dem vehLocator? Die aktuelle Location wird immer auf der variablen Strecke berechnet unabhängig vom reqState.
            // Also könnte man stattdessen einfach den vehLocator eine run(reqState) und eine initialize() (clearing) methode geben?
            // Für die if Bedingung könnte man alternativ auch fragen, ob es ein PBNS Assinment war (In RequestState ein Asgn. Typen hinzufügen?)
            if(!varReqState->isNotUsingVehicleBest() && !vehLocator.knowsCurrentLocationOf(varVehId) && variableRouteStateData.numStopsOf(varVehId) > 1) {
                vehLocator.initialize(req.requestTime, *varReqState);
                vehLocator.addPickupForProcessing(varReqState->getBestAssignment().pickup->id, varReqState->getBestAssignment().distToPickup);
                vehLocator.computeExactDistancesVia(*varReqState->getBestAssignment().vehicle, variableRouteStateData);
            }

            currentResult.push_back(varReqState);
            return currentResult;
        }

        void searchBestAssignmentOn(RouteStateData &data, BucketsWrapperT &buckets, RequestState<CostCalculatorT> &reqState) {
            calc.exchangeRouteStateData(data);
            asgnFinder.findBestAssignment(reqState, data, buckets);
        }

        RequestState<CostCalculatorT> *createAndInitializeRequestState(const Request &req) {
            auto *newRequestState = new RequestState<CostCalculatorT>(calc, config);
            requestStateInitializer.initializeRequestState(req, *newRequestState);
            return newRequestState;
        }
    private:
        std::vector<RequestState<CostCalculatorT>*> currentResult = {};

        AssignmentFinderT &asgnFinder;

        // Old Requests, their cost and location: oldReqData[req.requestId] = (Request, cost, location)
        std::vector<std::tuple<Request, int, int>> oldReqData;

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
