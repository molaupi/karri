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
#include "Algorithms/KaRRi/EllipticBCH/FeasiblePDLocs.h"
#include "Tools/Timer.h"

namespace karri {


    // Core of the KaRRi algorithm: Given a ride request r, this facility finds the optimal assignment of r to the route
    // of a vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<
            typename InputGraphT,
            typename EllipticBCHSearchResultT,
            typename RequestStateInitializerT,
            typename PDLocsFinderT,
            typename PdLocsAtExistingStopsFinderT,
            typename EllipticBchSearchesT,
            typename FfPdDistanceSearchesT,
            typename OrdAssignmentsT,
            typename PbnsAssignmentsT,
            typename PalsAssignmentsT,
            typename DalsAssignmentsT,
            typename RepositioningAssignmentsT,
            typename RelevantPdLocsFilterT
    >
    class AssignmentFinder {

    public:

        AssignmentFinder(const InputGraphT &inputGraph,
                         RequestStateInitializerT &requestStateInitializer,
                         PDLocsFinderT &pdLocsFinder,
                         const PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder,
                         EllipticBCHSearchResultT &pickupsEllipticResult,
                         EllipticBCHSearchResultT &dropoffsEllipticResult,
                         FeasiblePDLocs &feasiblePickups,
                         FeasiblePDLocs &feasibleDropoffs,
                         EllipticBchSearchesT &ellipticBchSearches,
                         FfPdDistanceSearchesT &ffPdDistanceSearches,
                         OrdAssignmentsT &ordinaryAssigments,
                         PbnsAssignmentsT &pbnsAssignments,
                         PalsAssignmentsT &palsAssignments,
                         DalsAssignmentsT &dalsAssignments,
                         RepositioningAssignmentsT &repositioningAssignments,
                         RelevantPdLocsFilterT &relevantPdLocsFilter)
                : inputGraph(inputGraph),
                  pickupsEllipticResult(pickupsEllipticResult),
                  dropoffsEllipticResult(dropoffsEllipticResult),
                  feasiblePickups(feasiblePickups),
                  feasibleDropoffs(feasibleDropoffs),
                  requestStateInitializer(requestStateInitializer),
                  pdLocsFinder(pdLocsFinder),
                  pdLocsAtExistingStopsFinder(pdLocsAtExistingStopsFinder),
                  ellipticBchSearches(ellipticBchSearches),
                  ffPDDistanceSearches(ffPdDistanceSearches),
                  ordAssignments(ordinaryAssigments),
                  pbnsAssignments(pbnsAssignments),
                  palsAssignments(palsAssignments),
                  dalsAssignments(dalsAssignments),
                  repositioningAssignments(repositioningAssignments),
                  relevantPdLocsFilter(relevantPdLocsFilter) {}

        RequestState findBestAssignment(const Request &req, const int now, stats::DispatchingPerformanceStats& stats) {

            // Initialize finder for this request, find PD locations:
            RequestState rs = requestStateInitializer.initializeRequestState(req, now, stats.initializationStats);
            PDLocs pdLocs = pdLocsFinder.findPDLocs(req.origin, req.destination, req.maxPickupWalkingDist, req.maxDropoffWalkingDist, req.walkingSpeed, stats.initializationStats);
            stats.numPickups = pdLocs.numPickups();
            stats.numDropoffs = pdLocs.numDropoffs();
            initializeComponentsForRequest(rs, pdLocs, stats);

            // Compute PD distances:
            const auto ffPdDistances = ffPDDistanceSearches.run(rs, pdLocs, stats.pdDistancesStats);

            // Try PALS assignments:
            palsAssignments.findAssignments(rs, ffPdDistances, pdLocs, stats.palsAssignmentsStats);

            // Run elliptic BCH searches (populates pickupsEllipticResult and dropoffsEllipticResult):
            ellipticBchSearches.run(pickupsEllipticResult, dropoffsEllipticResult, rs, pdLocs, stats.ellipticBchStats);

            // Consolidate results of elliptic BCH searches into feasible pickups and dropoffs for each stop:
            constructFeasiblePDLocs(rs, pdLocs, stats.ellipticBchStats);

            // Filter feasible PD-locations between ordinary stops (filtering is postponed until now so that the cost
            // of the best known assignment is as tight as possible):
            const auto relOrdinaryPickups = relevantPdLocsFilter.filterOrdinaryPickups(feasiblePickups, rs, pdLocs,
                                                                                       stats.ordAssignmentsStats);
            const auto relOrdinaryDropoffs = relevantPdLocsFilter.filterOrdinaryDropoffs(feasibleDropoffs, rs, pdLocs,
                                                                                         stats.ordAssignmentsStats);

            // Try ordinary assignments:
            ordAssignments.findAssignments(relOrdinaryPickups, relOrdinaryDropoffs, feasiblePickups, feasibleDropoffs,
                                           rs, ffPdDistances, pdLocs, stats.ordAssignmentsStats);

            // Filter feasible pickups before next stops:
            const auto relPickupsBeforeNextStop = relevantPdLocsFilter.filterPickupsBeforeNextStop(
                    feasiblePickups, rs, pdLocs, stats.pbnsAssignmentsStats);

            // Try DALS assignments:
            dalsAssignments.findAssignments(relOrdinaryPickups, relPickupsBeforeNextStop, rs, pdLocs, stats.dalsAssignmentsStats);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(relPickupsBeforeNextStop, relOrdinaryDropoffs, feasibleDropoffs,
                                            rs, ffPdDistances, pdLocs, stats.pbnsAssignmentsStats);

            // Try repositioning assignments (vehicles currently being repositioned):
            repositioningAssignments.findAssignments(rs, ffPdDistances, pdLocs, stats.repositioningAssignmentsStats);

            return rs;
        }

    private:

        void initializeComponentsForRequest(const RequestState& requestState, const PDLocs &pdLocs, stats::DispatchingPerformanceStats& stats) {
            pickupsEllipticResult.init(pdLocs.numPickups(), stats.ellipticBchStats);
            auto pickupsAtExistingStops = pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<PICKUP>(pdLocs.pickups, stats.ellipticBchStats);
            pickupsEllipticResult.initializeDistancesForPdLocsAtExistingStops(std::move(pickupsAtExistingStops), inputGraph, stats.ellipticBchStats);

            dropoffsEllipticResult.init(pdLocs.numDropoffs(), stats.ellipticBchStats);
            auto dropoffsAtExistingStops = pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<DROPOFF>(pdLocs.dropoffs, stats.ellipticBchStats);
            dropoffsEllipticResult.initializeDistancesForPdLocsAtExistingStops(std::move(dropoffsAtExistingStops), inputGraph, stats.ellipticBchStats);

            // Initialize components according to new request state:
            ellipticBchSearches.init(requestState, pdLocs, stats.ellipticBchStats);
            ffPDDistanceSearches.init(requestState, pdLocs, stats.pdDistancesStats);
            ordAssignments.init(requestState, pdLocs, stats.ordAssignmentsStats);
            pbnsAssignments.init(requestState, pdLocs, stats.pbnsAssignmentsStats);
            palsAssignments.init(requestState, pdLocs, stats.palsAssignmentsStats);
            dalsAssignments.init(requestState, pdLocs, stats.dalsAssignmentsStats);
            repositioningAssignments.init(requestState, pdLocs, stats.repositioningAssignmentsStats);
        }

        void constructFeasiblePDLocs(const RequestState &, const PDLocs &pdLocs,
                                     stats::EllipticBCHPerformanceStats &stats) {
            Timer timer;
            feasiblePickups.construct(pickupsEllipticResult, pdLocs.numPickups(),
                                      stats.pickupNumFeasibleStops, stats.pickupNumFeasiblePDLocs);
            feasibleDropoffs.construct(dropoffsEllipticResult, pdLocs.numDropoffs(),
                                       stats.dropoffNumFeasibleStops, stats.dropoffNumFeasiblePDLocs);
            stats.constructFeasiblePDLocsTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        const InputGraphT &inputGraph;
        EllipticBCHSearchResultT &pickupsEllipticResult; // Result of elliptic BCH searches for pickups.
        EllipticBCHSearchResultT &dropoffsEllipticResult; // Result of elliptic BCH searches for dropoffs.
        FeasiblePDLocs &feasiblePickups; // Feasible pickups for every stop (based on elliptic BCH search result).
        FeasiblePDLocs &feasibleDropoffs; // Feasible dropoffs for every stop (based on elliptic BCH search result).

        RequestStateInitializerT &requestStateInitializer;
        PDLocsFinderT &pdLocsFinder; // Finds possible pickup and dropoff locations for a given request
        const PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder; // Identifies pd locs that coincide with existing stops
        EllipticBchSearchesT &ellipticBchSearches; // Elliptic BCH searches that find distances between existing stops and PD-locations (except after last stop).
        FfPdDistanceSearchesT &ffPDDistanceSearches; // PD-distance searches that compute distances from pickups to dropoffs.
        OrdAssignmentsT &ordAssignments; // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        PbnsAssignmentsT &pbnsAssignments; // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PalsAssignmentsT &palsAssignments; // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        DalsAssignmentsT &dalsAssignments; // Tries DALS assignments where only the dropoff is inserted after the last stop.
        RepositioningAssignmentsT &repositioningAssignments; // Tries assignments using a vehicle that is currently being repositioned.
        RelevantPdLocsFilterT &relevantPdLocsFilter; // Filters feasible pickups/dropoffs into relevant ones for specific assignment types.


    };
}
