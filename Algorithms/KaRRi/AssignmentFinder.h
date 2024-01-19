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

#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Tools/Timer.h"

namespace karri {


    // Core of the KaRRi algorithm: Given a ride request r, this facility finds the optimal assignment of r to the route
    // of a vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<
            typename RequestStateInitializerT,
            typename EllipticBCHSearchesT,
            typename PDDistanceSearchesT,
            typename OrdAssignmentsT,
            typename PbnsAssignmentsT,
            typename PalsAssignmentsT,
            typename DalsAssignmentsT,
            typename RelevantPDLocsFilterT,
            typename CostCalculatorT,
            typename BucketsWrapperT
    >
    class AssignmentFinder {

    public:

        AssignmentFinder(RequestState<CostCalculatorT> &requestState, RequestStateInitializerT &requestStateInitializer,
                         EllipticBCHSearchesT &ellipticBchSearches, PDDistanceSearchesT &pdDistanceSearches,
                         OrdAssignmentsT &ordinaryAssigments, PbnsAssignmentsT &pbnsAssignments,
                         PalsAssignmentsT &palsAssignments, DalsAssignmentsT &dalsAssignments,
                         RelevantPDLocsFilterT &relevantPdLocsFilter,
                         RouteStateData &variableRouteStateData, RouteStateData &fixedRouteStateData,
                         BucketsWrapperT &variableBuckets, BucketsWrapperT &fixedBuckets,
                         CostCalculatorT &calc)
                : reqState(requestState),
                  requestStateInitializer(requestStateInitializer),
                  ellipticBchSearches(ellipticBchSearches),
                  pdDistanceSearches(pdDistanceSearches),
                  ordAssignments(ordinaryAssigments),
                  pbnsAssignments(pbnsAssignments),
                  palsAssignments(palsAssignments),
                  dalsAssignments(dalsAssignments),
                  relevantPdLocsFilter(relevantPdLocsFilter),
                  calc(calc),
                  variableRouteStateData(variableRouteStateData),
                  fixedRouteStateData(fixedRouteStateData),
                  variableBuckets(variableBuckets),
                  fixedBuckets(fixedBuckets){}

        const RequestState<CostCalculatorT> &findBestAssignment(const Request &req) {
            reqState.resetFixedData();
            reqState.fixedRunOn();
            findBestAssignmentProcedure(req, fixedRouteStateData);
            reqState.fixedRunOff();
            findBestAssignmentProcedure(req, variableRouteStateData);

            auto costBarrier = reqState.getBestCost() - reqState.getBestCostOnFixedRoutes();

            /**
            if (costBarrier > 0) {
                const auto missingStops = getMissingStops(reqState.getBestAssignmentOnFixedRoutes().vehicle->vehicleId);
                const auto reassignableRequests = getReassignableRequests(missingStops);
            }

            oldRequests.push_back(req);
            oldCost.push_back(reqState.getBestCost());
            if (reqState.isNotUsingVehicleBest())
                oldLocation.push_back(-1);
            else
                oldLocation.push_back(reqState.getBestAssignment().pickup->loc);

            */
            return reqState;
        }

        std::vector<int> getReassignableRequests(const std::vector<int> stopIds) {
            std::vector<int> result;
            for (const auto id: stopIds) {
               for (const auto req:  variableRouteStateData.getRequestsPickedUpAt(id)) {
                   result.push_back(req);
               }
            }
            return result;
        }

        std::vector<int> getMissingStops(const int vehId) {
            const auto fixedStopIds = fixedRouteStateData.stopIdsFor(vehId);
            const auto variableStopIds = variableRouteStateData.stopIdsFor(vehId);
            int counter = 0;
            std::vector<int> result;

            for (const auto id: variableStopIds) {
                if (counter >= fixedStopIds.size() || fixedStopIds[counter] != id) {
                    result.push_back(id);
                    continue;
                }
                counter++;
            }
            return result;

        }


    private:


        void findBestAssignmentProcedure(const Request &req, RouteStateData &data) {
            // Initialize finder for this request:
            initializeForRequest(req, data);

            // Compute PD distances:
            pdDistanceSearches.run();  // Nutzt keinen routestate

            // Try PALS assignments:
            palsAssignments.findAssignments(data);

            // Run elliptic BCH searches:
            ellipticBchSearches.run(data);

            // Filter feasible PD-locations between ordinary stops:
            relevantPdLocsFilter.filterOrdinary(data);

            // Try ordinary assignments:
            ordAssignments.findAssignments(data);

            // Filter feasible PD-locations before next stops:
            relevantPdLocsFilter.filterBeforeNextStop(data);

            // Try DALS assignments:
            dalsAssignments.findAssignments(data);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(data);
        }

        void initializeForRequest(const Request &req, RouteStateData &data) {
            calc.exchangeRouteStateData(data);
            requestStateInitializer.initializeRequestState(req);

            // Initialize components according to new request state:
            if (data.getTypeOfData() == RouteStateDataType::VARIABLE) {
                ellipticBchSearches.init(data, variableBuckets);
            } else {
                ellipticBchSearches.init(data, fixedBuckets);
            }
            pdDistanceSearches.init();
            ordAssignments.init();
            pbnsAssignments.init();
            palsAssignments.init();
            dalsAssignments.init();
        }

        RequestState<CostCalculatorT> &reqState;
        RequestStateInitializerT &requestStateInitializer;
        EllipticBCHSearchesT &ellipticBchSearches; // Elliptic BCH searches that find distances between existing stops and PD-locations (except after last stop).
        PDDistanceSearchesT &pdDistanceSearches; // PD-distance searches that compute distances from pickups to dropoffs.
        OrdAssignmentsT &ordAssignments; // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        PbnsAssignmentsT &pbnsAssignments; // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PalsAssignmentsT &palsAssignments; // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        DalsAssignmentsT &dalsAssignments; // Tries DALS assignments where only the dropoff is inserted after the last stop.
        RelevantPDLocsFilterT &relevantPdLocsFilter; // Additionally filters feasible pickups/dropoffs found by elliptic BCH searches.

        CostCalculatorT &calc;

        RouteStateData &variableRouteStateData;
        RouteStateData &fixedRouteStateData;

        BucketsWrapperT &variableBuckets;
        BucketsWrapperT &fixedBuckets;

        std::vector<Request> oldRequests; // Stores Request, cost, location for a request at oldRequests[reqId]
        std::vector<int> oldCost;
        std::vector<int> oldLocation;
    };
}