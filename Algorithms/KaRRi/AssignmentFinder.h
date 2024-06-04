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
#include "Algorithms/KaRRi/RequestState/BestAsgn.h"

namespace karri {


    // Core of the KaRRi algorithm: Given a ride request r, this facility finds the optimal assignment of r to the route
    // of a vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<
            typename RequestStateInitializerT,
            typename AssignmentFinderResponseT,
            typename EllipticBucketsEnvT,
            typename PALSPrecomputedLastStopInfoT,
            typename DALSPrecomputedLastStopInfoT,
            typename LastStopsAtVerticesInfoT,
            typename EllipticBCHSearchesT,
            typename PDDistanceSearchesT,
            typename OrdAssignmentsT,
            typename PbnsAssignmentsT,
            typename PalsAssignmentsT,
            typename DalsAssignmentsT,
            typename RelevantPDLocsFilterT
    >
    class AssignmentFinder {

    public:

        AssignmentFinder(RequestState &requestState,
                         AssignmentFinderResponseT assignmentFinderResponse,
                         const RouteStateData &varRouteState,
                         const EllipticBucketsEnvT &varEllipticBuckets,
                         const PALSPrecomputedLastStopInfoT &varPalsPrecomputedLastStopInfo,
                         const DALSPrecomputedLastStopInfoT &varDalsPrecomputedLastStopInfo,
                         const LastStopsAtVerticesInfoT &varLastStopsAtVerticesInfo,
                         const RouteStateData &fixedRouteState,
                         const EllipticBucketsEnvT &fixedEllipticBuckets,
                         const PALSPrecomputedLastStopInfoT &fixedPalsPrecomputedLastStopInfo,
                         const DALSPrecomputedLastStopInfoT &fixedDalsPrecomputedLastStopInfo,
                         const LastStopsAtVerticesInfoT &fixedLastStopsAtVerticesInfo,
                         RequestStateInitializerT &requestStateInitializer,
                         EllipticBCHSearchesT &ellipticBchSearches,
                         PDDistanceSearchesT &pdDistanceSearches,
                         OrdAssignmentsT &ordinaryAssigments,
                         PbnsAssignmentsT &pbnsAssignments,
                         PalsAssignmentsT &palsAssignments,
                         DalsAssignmentsT &dalsAssignments,
                         RelevantPDLocsFilterT &relevantPdLocsFilter)
                : reqState(requestState),
                  asgnFinderResponse(assignmentFinderResponse),
                  varRouteState(varRouteState),
                  varEllipticBuckets(varEllipticBuckets),
                  varPalsPrecomputedLastStopInfo(varPalsPrecomputedLastStopInfo),
                  varDalsPrecomputedLastStopInfo(varDalsPrecomputedLastStopInfo),
                  varLastStopsAtVerticesInfo(varLastStopsAtVerticesInfo),
                  fixedRouteState(fixedRouteState),
                  fixedEllipticBuckets(fixedEllipticBuckets),
                  fixedPalsPrecomputedLastStopInfo(fixedPalsPrecomputedLastStopInfo),
                  fixedDalsPrecomputedLastStopInfo(fixedDalsPrecomputedLastStopInfo),
                  fixedLastStopsAtVerticesInfo(fixedLastStopsAtVerticesInfo),
                  requestStateInitializer(requestStateInitializer),
                  ellipticBchSearches(ellipticBchSearches),
                  pdDistanceSearches(pdDistanceSearches),
                  ordAssignments(ordinaryAssigments),
                  pbnsAssignments(pbnsAssignments),
                  palsAssignments(palsAssignments),
                  dalsAssignments(dalsAssignments),
                  relevantPdLocsFilter(relevantPdLocsFilter) {}

        const AssignmentFinderResponseT &findBestAssignment(const Request &req) {

            // Initialize for this request:
            requestStateInitializer.initializeRequestState(req);
            asgnFinderResponse.initNotUsingVehicle();

            // Compute PD distances:
            pdDistanceSearches.init();
            pdDistanceSearches.run();

            // Find the best regular assignment:
            asgnFinderResponse.initRegular();
            findBestAssignment(asgnFinderResponse.getBestRegularAsgn(), varRouteState, varEllipticBuckets,
                               varPalsPrecomputedLastStopInfo, varDalsPrecomputedLastStopInfo,
                               varLastStopsAtVerticesInfo);

            // Find the best displacing assignment:
            asgnFinderResponse.initDisplacing();
            findBestAssignment(asgnFinderResponse.getBestDisplacingAsgn(), fixedRouteState, fixedEllipticBuckets,
                               fixedPalsPrecomputedLastStopInfo, fixedDalsPrecomputedLastStopInfo,
                               fixedLastStopsAtVerticesInfo);
            if (asgnFinderResponse.getBestDisplacingAsgn().cost() < asgnFinderResponse.getBestRegularAsgn().cost()) {
                std::cout << "Request " << req.requestId <<  ": displacing cost = "
                          << asgnFinderResponse.getBestDisplacingAsgn().cost()
                          << " < regular cost = "
                          << asgnFinderResponse.getBestRegularAsgn().cost() << ".\n";
            }

            // TODO: Remove this once we can handle displacing assignments.
            asgnFinderResponse.initDisplacing();


            return asgnFinderResponse;
        }

    private:

        void findBestAssignment(BestAsgn &bestAsgn, const RouteStateData &routeState,
                                const EllipticBucketsEnvT &ellipticBucketsEnv,
                                const PALSPrecomputedLastStopInfoT &palsPrecomputedLastStopInfo,
                                const DALSPrecomputedLastStopInfoT &dalsPrecomputedLastStopInfo,
                                const LastStopsAtVerticesInfoT &lastStopsAtVerticesInfo) {

            initializeComponents(routeState, ellipticBucketsEnv, lastStopsAtVerticesInfo);

            // Try PALS assignments:
            palsAssignments.findAssignments(bestAsgn, routeState, palsPrecomputedLastStopInfo, lastStopsAtVerticesInfo);

            // Run elliptic BCH searches:
            ellipticBchSearches.run(bestAsgn.cost(), routeState, ellipticBucketsEnv.getSourceBuckets(),
                                    ellipticBucketsEnv.getTargetBuckets());

            // Filter feasible PD-locations between ordinary stops:
            relevantPdLocsFilter.filterOrdinary(bestAsgn.cost(), routeState);

            // Try ordinary assignments:
            ordAssignments.findAssignments(bestAsgn, routeState);

            // Filter feasible PD-locations before next stops:
            relevantPdLocsFilter.filterBeforeNextStop(bestAsgn.cost(), routeState);

            // Try DALS assignments:
            dalsAssignments.findAssignments(bestAsgn, routeState, dalsPrecomputedLastStopInfo);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(bestAsgn, routeState);
        }

        void
        initializeComponents(const RouteStateData &routeState,
                             const EllipticBucketsEnvT &ellipticBucketsEnv,
                             const LastStopsAtVerticesInfoT &lastStopsAtVertices) {
            ellipticBchSearches.init(routeState, ellipticBucketsEnv.getSourceBuckets(), lastStopsAtVertices);
            ordAssignments.init(routeState);
            pbnsAssignments.init(routeState);
            palsAssignments.init(routeState);
            dalsAssignments.init(routeState);
        }

        RequestState &reqState;
        AssignmentFinderResponseT asgnFinderResponse;

        const RouteStateData &varRouteState;
        const EllipticBucketsEnvT &varEllipticBuckets;
        const PALSPrecomputedLastStopInfoT &varPalsPrecomputedLastStopInfo;
        const DALSPrecomputedLastStopInfoT &varDalsPrecomputedLastStopInfo;
        const LastStopsAtVerticesInfoT &varLastStopsAtVerticesInfo;

        const RouteStateData &fixedRouteState;
        const EllipticBucketsEnvT &fixedEllipticBuckets;
        const PALSPrecomputedLastStopInfoT &fixedPalsPrecomputedLastStopInfo;
        const DALSPrecomputedLastStopInfoT &fixedDalsPrecomputedLastStopInfo;
        const LastStopsAtVerticesInfoT &fixedLastStopsAtVerticesInfo;

        RequestStateInitializerT &requestStateInitializer;
        EllipticBCHSearchesT &ellipticBchSearches; // Elliptic BCH searches that find distances between existing stops and PD-locations (except after last stop).
        PDDistanceSearchesT &pdDistanceSearches; // PD-distance searches that compute distances from pickups to dropoffs.
        OrdAssignmentsT &ordAssignments; // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        PbnsAssignmentsT &pbnsAssignments; // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PalsAssignmentsT &palsAssignments; // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        DalsAssignmentsT &dalsAssignments; // Tries DALS assignments where only the dropoff is inserted after the last stop.
        RelevantPDLocsFilterT &relevantPdLocsFilter; // Additionally filters feasible pickups/dropoffs found by elliptic BCH searches.


    };
}