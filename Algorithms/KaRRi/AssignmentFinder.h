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

        AssignmentFinder(EllipticBCHSearchesT &ellipticBchSearches, PDDistanceSearchesT &pdDistanceSearches,
                         OrdAssignmentsT &ordinaryAssigments, PbnsAssignmentsT &pbnsAssignments,
                         PalsAssignmentsT &palsAssignments, DalsAssignmentsT &dalsAssignments,
                         RelevantPDLocsFilterT &relevantPdLocsFilter)
                : ellipticBchSearches(ellipticBchSearches),
                  pdDistanceSearches(pdDistanceSearches),
                  ordAssignments(ordinaryAssigments),
                  pbnsAssignments(pbnsAssignments),
                  palsAssignments(palsAssignments),
                  dalsAssignments(dalsAssignments),
                  relevantPdLocsFilter(relevantPdLocsFilter) {}

        void findBestAssignment(RequestState<CostCalculatorT> &requestState, RouteStateData &data, BucketsWrapperT &buckets) {
            // Initialize finder for this request:
            initializeForRequest(requestState, data, buckets);

            // Compute PD distances:
            pdDistanceSearches.run();  // Nutzt keinen routestate

            // Try PALS assignments:
            palsAssignments.findAssignments(data);

            // Run elliptic BCH searches:
            ellipticBchSearches.run(data);

            // Filter feasible PD-locations between ordinary stops:
            relevantPdLocsFilter.filterOrdinary(data, requestState);

            // Try ordinary assignments:
            ordAssignments.findAssignments(data);

            // Filter feasible PD-locations before next stops:
            relevantPdLocsFilter.filterBeforeNextStop(data, requestState);

            // Try DALS assignments:
            dalsAssignments.findAssignments(data);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(data);
        }


    private:

        void initializeForRequest(RequestState<CostCalculatorT> &requestState, RouteStateData &data, BucketsWrapperT &buckets) {
            // Initialize components according to new request state:
            ellipticBchSearches.init(data, buckets, requestState);
            pdDistanceSearches.init(requestState);
            ordAssignments.init(requestState);
            pbnsAssignments.init(requestState);
            palsAssignments.init(requestState);
            dalsAssignments.init(requestState);
        }

        EllipticBCHSearchesT &ellipticBchSearches; // Elliptic BCH searches that find distances between existing stops and PD-locations (except after last stop).
        PDDistanceSearchesT &pdDistanceSearches; // PD-distance searches that compute distances from pickups to dropoffs.
        OrdAssignmentsT &ordAssignments; // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        PbnsAssignmentsT &pbnsAssignments; // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PalsAssignmentsT &palsAssignments; // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        DalsAssignmentsT &dalsAssignments; // Tries DALS assignments where only the dropoff is inserted after the last stop.
        RelevantPDLocsFilterT &relevantPdLocsFilter; // Additionally filters feasible pickups/dropoffs found by elliptic BCH searches.
    };
}