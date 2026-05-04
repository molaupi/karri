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
            typename InputGraphT,
            typename FeasibleEllipticDistancesT,
            typename RequestStateInitializerT,
            typename PDLocsInRadiusQueryT,
            typename PdLocsAtExistingStopsFinderT,
            typename EllipticBchSearchesT,
            typename FfPdDistanceSearchesT,
            typename OrdAssignmentsT,
            typename PbnsAssignmentsT,
            typename PalsAssignmentsT,
            typename DalsAssignmentsT,
            typename RelevantPDLocsFilterT,
            typename AssignmentsWithTransferT,
            typename InsertionAsserterT
    >
    class AssignmentFinder {

    public:

        AssignmentFinder(RequestState &requestState,
                         const InputGraphT &inputGraph,
                         const Fleet &fleet,
                         const RouteState &routeState,
                         RequestStateInitializerT &requestStateInitializer,
                         PDLocsInRadiusQueryT &pdLocsInRadiusQuery,
                         PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder,
                         EllipticBchSearchesT &ellipticBchSearches,
                         FfPdDistanceSearchesT &ffPdDistanceSearches,
                         OrdAssignmentsT &ordinaryAssigments,
                         PbnsAssignmentsT &pbnsAssignments,
                         PalsAssignmentsT &palsAssignments,
                         DalsAssignmentsT &dalsAssignments,
                         RelevantPDLocsFilterT &relevantPdLocsFilter,
                         AssignmentsWithTransferT &assignmentsWithTransfer,
                         InsertionAsserterT &insertionAsserter
        )
                : reqState(requestState),
                  inputGraph(inputGraph),
                  fleet(fleet),
                  routeState(routeState),
                  feasibleEllipticPickups(fleet.size(), routeState, reqState.stats().ellipticBchStats),
                  feasibleEllipticDropoffs(fleet.size(), routeState, reqState.stats().ellipticBchStats),
                  requestStateInitializer(requestStateInitializer),
                    pdLocsInRadiusQuery(pdLocsInRadiusQuery),
                  pdLocsAtExistingStopsFinder(pdLocsAtExistingStopsFinder),
                  ellipticBchSearches(ellipticBchSearches),
                  ffPDDistanceSearches(ffPdDistanceSearches),
                  ordAssignments(ordinaryAssigments),
                  pbnsAssignments(pbnsAssignments),
                  palsAssignments(palsAssignments),
                  dalsAssignments(dalsAssignments),
                  relevantPdLocsFilter(relevantPdLocsFilter),
                  assignmentsWithTransfer(assignmentsWithTransfer),
                  insertionAsserter(insertionAsserter) {}

        RequestState findBestAssignment(const Request &req) {

            // Initialize finder for this request:
            auto pdLocs = pdLocsInRadiusQuery.findPDLocs(req.origin, req.destination, reqState.stats().initializationStats);
            requestStateInitializer.initializeRequestState(req);
            reqState.stats().numPickups = pdLocs.numPickups();
            reqState.stats().numDropoffs = pdLocs.numDropoffs();

            init(pdLocs);

            // Compute PD distances:
            const PDDistances pdDistances = ffPDDistanceSearches.run(pdLocs);

            // Try PALS assignments:
            palsAssignments.findAssignments(pdDistances, pdLocs);


            // Run elliptic BCH searches (populates feasibleEllipticPickups and feasibleEllipticDropoffs):
            ellipticBchSearches.run(feasibleEllipticPickups, feasibleEllipticDropoffs, pdLocs);

            // Filter feasible PD-locations between ordinary stops:
            const auto relOrdinaryPickups = relevantPdLocsFilter.filterOrdinaryPickups(feasibleEllipticPickups, reqState, pdLocs);
            const auto relOrdinaryDropoffs = relevantPdLocsFilter.filterOrdinaryDropoffs(feasibleEllipticDropoffs, reqState, pdLocs);

            // Try ordinary assignments:
            ordAssignments.findAssignments(relOrdinaryPickups, relOrdinaryDropoffs, pdDistances, pdLocs);

            // Filter feasible pickups before next stops:
            const auto relPickupsBeforeNextStop = relevantPdLocsFilter.filterPickupsBeforeNextStop(
                    feasibleEllipticPickups, reqState, pdLocs);

            // Try DALS assignments:
            dalsAssignments.findAssignments(relOrdinaryPickups, relPickupsBeforeNextStop, pdLocs);

            // Filter feasible dropoffs before next stop:
            const auto relDropoffsBeforeNextStop = relevantPdLocsFilter.filterDropoffsBeforeNextStop(
                    feasibleEllipticDropoffs, reqState, pdLocs);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(relPickupsBeforeNextStop, relOrdinaryDropoffs, relDropoffsBeforeNextStop,
                                            pdDistances, pdLocs);

            KASSERT(insertionAsserter.assertAssignment(reqState.getBestAssignmentWithoutTransfer()));

            // * Find the best assignment that contains a transfer
            if (InputConfig::getInstance().includeTransfers)
                assignmentsWithTransfer.findBestAssignment(relOrdinaryPickups, relPickupsBeforeNextStop, relOrdinaryDropoffs, relDropoffsBeforeNextStop, pdDistances, pdLocs);

            //* Log the cost data
            const auto &costWOT = reqState.getCostObjectWithoutTransfer();
            const auto &costWT = reqState.getCostObjectWithTransfer();

            auto &costStats = reqState.stats().costStats;

            costStats.totalWOT = costWOT.total;
            costStats.walkingCostWOT = costWOT.walkingCost;
            costStats.tripCostWOT = costWOT.tripCost;
            costStats.waitTimeViolationCostWOT = costWOT.waitTimeViolationCost;
            costStats.changeInTripCostsOfOthersWOT = costWOT.changeInTripCostsOfOthers;
            costStats.vehCostWOT = costWOT.vehCost;

            costStats.totalWT = costWT.total;
            costStats.walkingCostWT = costWT.walkingCost;
            costStats.tripCostWT = costWT.tripCost;
            costStats.waitTimeViolationCostWT = costWT.waitTimeViolationCost;
            costStats.changeInTripCostsOfOthersWT = costWT.changeInTripCostsOfOthers;
            costStats.vehCostWT = costWT.vehCost;

            costStats.inftyWOT = costWOT.total >= INFTY;
            costStats.inftyWT = costWT.total >= INFTY;
            costStats.transferImproves = costWT.total < costWOT.total;

            return reqState;
        }

    private:

        void init(const PDLocs& pdLocs) {
            feasibleEllipticPickups.init(pdLocs.numPickups());
            feasibleEllipticPickups.initializeDistancesForPdLocsAtExistingStops(
                    pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<PICKUP>(pdLocs.pickups),
                    inputGraph);
            feasibleEllipticDropoffs.init(pdLocs.numDropoffs());
            feasibleEllipticDropoffs.initializeDistancesForPdLocsAtExistingStops(
                    pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<DROPOFF>(pdLocs.dropoffs),
                    inputGraph);

            // Initialize components according to new request state:
            ellipticBchSearches.init();
            ffPDDistanceSearches.init();
            ordAssignments.init();
            pbnsAssignments.init(pdLocs);
            palsAssignments.init();
            dalsAssignments.init();
            assignmentsWithTransfer.init();
        }

        RequestState &reqState;
        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const RouteState &routeState;
        FeasibleEllipticDistancesT feasibleEllipticPickups;
        FeasibleEllipticDistancesT feasibleEllipticDropoffs;

        RequestStateInitializerT &requestStateInitializer;
        PDLocsInRadiusQueryT &pdLocsInRadiusQuery; // Finds PD-locs for the request (e.g. in a radius around origin/destination)
        PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder; // Identifies pd locs that coincide with existing stops
        EllipticBchSearchesT &ellipticBchSearches; // Elliptic BCH searches that find distances between existing stops and PD-locations (except after last stop).
        FfPdDistanceSearchesT &ffPDDistanceSearches; // PD-distance searches that compute distances from pickups to dropoffs.
        OrdAssignmentsT &ordAssignments; // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        PbnsAssignmentsT &pbnsAssignments; // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PalsAssignmentsT &palsAssignments; // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        DalsAssignmentsT &dalsAssignments; // Tries DALS assignments where only the dropoff is inserted after the last stop.
        RelevantPDLocsFilterT &relevantPdLocsFilter; // Additionally filters feasible pickups/dropoffs found by elliptic BCH searches.

        AssignmentsWithTransferT &assignmentsWithTransfer;

        InsertionAsserterT& insertionAsserter;
    };
}