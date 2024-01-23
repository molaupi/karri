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
            typename BucketsWrapperT>
    class AssignmentManager {

    public:
        AssignmentManager(AssignmentFinderT &asgnFinder, CostCalculatorT &calc,
                          InputConfig &config, RequestStateInitializerT &requestStateInitializer,
                          RouteStateData &variableRouteStateData, RouteStateData &fixedRouteStateData,
                          BucketsWrapperT &variableBuckets, BucketsWrapperT &fixedBuckets):
                          asgnFinder(asgnFinder),
                          calc(calc),
                          config(config),
                          requestStateInitializer(requestStateInitializer),
                          variableRouteStateData(variableRouteStateData),
                          fixedRouteStateData(fixedRouteStateData),
                          variableBuckets(variableBuckets),
                          fixedBuckets(fixedBuckets) {}

        std::vector<RequestState<CostCalculatorT>*> &calculateChanges(const Request &req) {
            if (currentResult.size() > 0) {
                delete currentResult[0];
            }
            currentResult.clear();
            auto *reqState = createAndInitializeRequestState(req);
            calc.exchangeRouteStateData(variableRouteStateData);
            asgnFinder.findBestAssignment(*reqState, variableRouteStateData, variableBuckets);
            currentResult.push_back(reqState);
            return currentResult;
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

        BucketsWrapperT &variableBuckets;
        BucketsWrapperT &fixedBuckets;
    };

}
