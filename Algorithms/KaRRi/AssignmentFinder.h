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
            typename PdLocsAtExistingStopsFinderT,
            typename EllipticBchSearchesT,
            typename FfPdDistanceSearchesT,
            typename OrdAssignmentsT,
            typename PbnsAssignmentsT,
            typename PalsAssignmentsT,
            typename DalsAssignmentsT,
            typename RelevantPdLocsFilterT
    >
    class AssignmentFinder {

    public:

        AssignmentFinder(RequestState &requestState,
                         const InputGraphT &inputGraph,
                         const Fleet &fleet,
                         const RouteState &routeState,
                         RequestStateInitializerT &requestStateInitializer,
                         PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder,
                         EllipticBchSearchesT &ellipticBchSearches,
                         FfPdDistanceSearchesT &ffPdDistanceSearches,
                         OrdAssignmentsT &ordinaryAssigments,
                         PbnsAssignmentsT &pbnsAssignments,
                         PalsAssignmentsT &palsAssignments,
                         DalsAssignmentsT &dalsAssignments,
                         RelevantPdLocsFilterT &relevantPdLocsFilter)
                : reqState(requestState),
                  inputGraph(inputGraph),
                  fleet(fleet),
                  routeState(routeState),
                  feasibleEllipticPickups(fleet.size(), routeState, reqState.stats().ellipticBchStats),
                  feasibleEllipticDropoffs(fleet.size(), routeState, reqState.stats().ellipticBchStats),
                  requestStateInitializer(requestStateInitializer),
                  pdLocsAtExistingStopsFinder(pdLocsAtExistingStopsFinder),
                  ellipticBchSearches(ellipticBchSearches),
                  ffPDDistanceSearches(ffPdDistanceSearches),
                  ordAssignments(ordinaryAssigments),
                  pbnsAssignments(pbnsAssignments),
                  palsAssignments(palsAssignments),
                  dalsAssignments(dalsAssignments),
                  relevantPdLocsFilter(relevantPdLocsFilter) {}

        const RequestState &findBestAssignment(const Request &req) {

            // Initialize finder for this request:
            initializeForRequest(req);

            // Compute PD distances:
            const auto ffPdDistances = ffPDDistanceSearches.run();

            // Try PALS assignments:
            palsAssignments.findAssignments(ffPdDistances);

            // Run elliptic BCH searches (populates feasibleEllipticPickups and feasibleEllipticDropoffs):
            ellipticBchSearches.run(feasibleEllipticPickups, feasibleEllipticDropoffs);

            // Filter feasible PD-locations between ordinary stops:
            const auto relOrdinaryPickups = relevantPdLocsFilter.filterOrdinaryPickups(feasibleEllipticPickups);
            const auto relOrdinaryDropoffs = relevantPdLocsFilter.filterOrdinaryDropoffs(feasibleEllipticDropoffs);

            // Try ordinary assignments:
            ordAssignments.findAssignments(relOrdinaryPickups, relOrdinaryDropoffs, ffPdDistances);

            // Filter feasible pickups before next stops:
            const auto relPickupsBeforeNextStop = relevantPdLocsFilter.filterPickupsBeforeNextStop(
                    feasibleEllipticPickups);

            // Try DALS assignments:
            dalsAssignments.findAssignments(relOrdinaryPickups, relPickupsBeforeNextStop);

            // Filter feasible dropoffs before next stop:
            const auto relDropoffsBeforeNextStop = relevantPdLocsFilter.filterDropoffsBeforeNextStop(
                    feasibleEllipticDropoffs);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(relPickupsBeforeNextStop, relOrdinaryDropoffs, relDropoffsBeforeNextStop,
                                            ffPdDistances);

            return reqState;
        }

    private:

        void initializeForRequest(const Request &req) {
            requestStateInitializer.initializeRequestState(req);
            feasibleEllipticPickups.init(reqState.numPickups());
            feasibleEllipticPickups.initializeDistancesForPdLocsAtExistingStops(
                    pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<PICKUP>(reqState.pickups),
                    inputGraph);
            feasibleEllipticDropoffs.init(reqState.numDropoffs());
            feasibleEllipticDropoffs.initializeDistancesForPdLocsAtExistingStops(
                    pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<DROPOFF>(reqState.dropoffs),
                    inputGraph);

            // Initialize components according to new request state:
            ellipticBchSearches.init();
            ffPDDistanceSearches.init();
            ordAssignments.init();
            pbnsAssignments.init();
            palsAssignments.init();
            dalsAssignments.init();
        }

        RequestState &reqState;
        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const RouteState &routeState;
        FeasibleEllipticDistancesT feasibleEllipticPickups;
        FeasibleEllipticDistancesT feasibleEllipticDropoffs;

        RequestStateInitializerT &requestStateInitializer;
        PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder; // Identifies pd locs that coincide with existing stops
        EllipticBchSearchesT &ellipticBchSearches; // Elliptic BCH searches that find distances between existing stops and PD-locations (except after last stop).
        FfPdDistanceSearchesT &ffPDDistanceSearches; // PD-distance searches that compute distances from pickups to dropoffs.
        OrdAssignmentsT &ordAssignments; // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        PbnsAssignmentsT &pbnsAssignments; // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PalsAssignmentsT &palsAssignments; // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        DalsAssignmentsT &dalsAssignments; // Tries DALS assignments where only the dropoff is inserted after the last stop.
        RelevantPdLocsFilterT &relevantPdLocsFilter; // Additionally filters feasible pickups/dropoffs found by elliptic BCH searches.


    };
}