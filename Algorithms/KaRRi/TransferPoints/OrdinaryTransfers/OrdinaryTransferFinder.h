/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
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

#include <map>

#include "Algorithms/KaRRi/TransferPoints/VertexInEllipse.h"
#include "Tools/Logging/LogManager.h"
#include "DataStructures/Containers/LightweightSubset.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Tools/Timer.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "Algorithms/CH/CH.h"
#include "DataStructures/Containers/FastResetFlagArray.h"
#include "Algorithms/KaRRi/TransferPoints/EdgeEllipseContainer.h"
#include "Algorithms/KaRRi/TransferPoints/EdgeEllipseIntersector.h"
#include "Algorithms/KaRRi/TransferPoints/DropoffALS/RelevantDropoffsAfterLastStop.h"

#pragma once

namespace karri {

    class NoOpOrdinaryTransferFinder {

    public:
        void init() {}

        template<typename EllipsesT>
        void findAssignments(const std::vector<int> &, const std::vector<int> &,
                             const RelevantDropoffsAfterLastStop &, const EllipsesT &) {}
    };

    template<
            typename InputGraphT,
            typename VehCHEnvT,
            typename CurVehLocToPickupSearchesT,
            bool UseCostLowerBounds,
            bool DoTransferPointParetoChecks,
            typename InsertionAsserterT,
            typename DirectTransferDistancesFinderT>
    class OrdinaryTransferFinder {

        using VehCHQueryLabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<VehCHQueryLabelSet>;


        // Both vehicles can drive a detour to the transfer point, but none drives the detour ALS
        // This implies, that the pickup is BNS or ORD, the dropoff can be BNS, ORD or ALS
        // If the dropoff is ALS, we need to consider a different set of vehicles to calculate the transfer psints between
    public:

        OrdinaryTransferFinder(
                const InputGraphT &inputGraph,
                const VehCHEnvT &vehChEnv,
                CurVehLocToPickupSearchesT &searches,
                DirectTransferDistancesFinderT &pickupToTransferDistancesFinder,
                DirectTransferDistancesFinderT &transferToDropoffDistancesFinder,
                const Fleet &fleet,
                const RouteState &routeState,
                RequestState &requestState,
                CostCalculator &calc,
                InsertionAsserterT &asserter
        ) :
                inputGraph(inputGraph),
                vehCh(vehChEnv.getCH()),
                vehChQuery(vehChEnv.template getFullCHQuery<VehCHQueryLabelSet>()),
                searches(searches),
                pickupToTransferDistancesFinder(pickupToTransferDistancesFinder),
                transferToDropoffDistancesFinder(transferToDropoffDistancesFinder),
                fleet(fleet),
                routeState(routeState),
                requestState(requestState),
                calc(calc),
                asserter(asserter),
                ellipseIntersector(inputGraph, fleet, requestState, routeState, calc),
                edgesSubset(inputGraph.numEdges()),
                stopSeen(fleet.size()) {}


        void init() {
            totalTime = 0;
            numPartialsTriedPickupBNS = 0;
            numPartialsTriedPickupORD = 0;
            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedDropoffBNS = 0;
            numAssignmentsTriedDropoffORD = 0;
            numAssignmentsTriedDropoffALS = 0;
            numStopPairs = 0;
            numTransferPoints = 0;
        }

        // Given stop IDs at which a transfer with an ordinary detour (i.e. a detour between two existing stops) may
        // be inserted, this method finds all possible assignments with an ordinary transfer.
        template<typename EllipsesT>
        void findAssignments(const std::vector<int> &pVehStopIds, const std::vector<int> &dVehStopIds,
            const RelevantPDLocs &relORDPickups, const RelevantPDLocs &relBNSPickups,
                                const RelevantPDLocs &relORDDropoffs, const RelevantPDLocs &relBNSDropoffs,
                             const RelevantDropoffsAfterLastStop &relALSDropoffs,
                             const EllipsesT &ellipseContainer,
                             const PDLocs &pdLocs) {
            Timer total;
            Timer innerTimer;
            if (relORDPickups.getVehiclesWithRelevantPDLocs().empty() &&
                relBNSPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            // Calculate transfer points
            int64_t numTransferPointsBuilt = 0;
            innerTimer.restart();
            ellipseIntersector.computeTransferPoints(pVehStopIds, dVehStopIds, ellipseContainer, numTransferPointsBuilt);
            const auto intersectEllipsesTime = innerTimer.elapsed<std::chrono::nanoseconds>();

            // Run selection phase for many-to-many searches used to find distances from pickups to transfers and from
            // transfers to dropoffs.
            std::vector<int> pdLocRanks;
            std::vector<int> pdLocOffsets;
            pdLocRanks.reserve(pdLocs.numPickups());
            pdLocOffsets.reserve(pdLocs.numPickups());
            for (const auto &p: pdLocs.pickups) {
                pdLocRanks.push_back(vehCh.rank(inputGraph.edgeHead(p.loc)));
                pdLocOffsets.push_back(0);
            }
            pickupToTransferDistancesFinder.runSelectionForPdLocs(pdLocRanks, pdLocOffsets);

            pdLocRanks.clear();
            pdLocOffsets.clear();
            pdLocRanks.reserve(pdLocs.numDropoffs());
            pdLocOffsets.reserve(pdLocs.numDropoffs());
            for (const auto &d: pdLocs.dropoffs) {
                pdLocRanks.push_back(vehCh.rank(inputGraph.edgeTail(d.loc)));
                pdLocOffsets.push_back(inputGraph.travelTime(d.loc));
            }
            transferToDropoffDistancesFinder.runSelectionForPdLocs(pdLocRanks, pdLocOffsets);



            // Loop over all the possible combinations of stop pairs between which an ordinary transfer is possible
            int64_t tryPostponedTime = 0;
            int64_t numPostponed = 0;
            Timer postponedTimer;
            innerTimer.restart();
            std::vector<AssignmentWithTransfer> promisingPartials;
            std::vector<AssignmentWithTransfer> postponedFullAssignments;
            for (const auto &pStopId: pVehStopIds) {
                const auto pVehId = routeState.vehicleIdOf(pStopId);
                KASSERT(relORDPickups.hasRelevantSpotsFor(pVehId) ||
                        relBNSPickups.hasRelevantSpotsFor(pVehId));
                for (const auto &dStopId: dVehStopIds) {
                    const auto dVehId = routeState.vehicleIdOf(dStopId);
                    KASSERT(relORDDropoffs.hasRelevantSpotsFor(dVehId) ||
                            relBNSDropoffs.hasRelevantSpotsFor(dVehId) ||
                            relALSDropoffs.hasRelevantSpotsFor(dVehId));

                    // pVeh and dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;


                    // Find the assignments with the transfer points
                    promisingPartials.clear();
                    findPartialAssignmentsForPairOfStopPairs(pStopId, dStopId, relORDPickups, relBNSPickups, promisingPartials, pdLocs);

                    if (promisingPartials.empty())
                        continue;

                    KASSERT(postponedFullAssignments.empty());
                    for (const auto &partialAsgn: promisingPartials) {
                        tryDropoffBNS(partialAsgn, relBNSDropoffs, postponedFullAssignments, pdLocs);
                        tryDropoffORD(partialAsgn, relORDDropoffs, postponedFullAssignments, pdLocs);
                        tryDropoffALS(partialAsgn, relALSDropoffs, postponedFullAssignments, pdLocs);
                    }

                    promisingPartials.clear();

                    if (postponedFullAssignments.empty())
                        continue;

                    // Finish the postponed assignments
                    numPostponed += postponedFullAssignments.size();
                    postponedTimer.restart();
                    finishAssignments(pVehId, dVehId, postponedFullAssignments, pdLocs);
                    tryPostponedTime += postponedTimer.elapsed<std::chrono::nanoseconds>();

                    postponedFullAssignments.clear();
                }
            }


            const auto tryAssignmentsTime = innerTimer.elapsed<std::chrono::nanoseconds>();

            // Write the statss
            auto &stats = requestState.stats().ordinaryTransferStats;

            stats.totalTime = total.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesDropoffBNS += relBNSDropoffs.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesDropoffORD += relORDDropoffs.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesDropoffALS += relALSDropoffs.getVehiclesWithRelevantPDLocs().size();
            stats.numPartialsTriedPickupBNS += numPartialsTriedPickupBNS;
            stats.numPartialsTriedPickupORD += numPartialsTriedPickupORD;
            stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
            stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
            stats.numAssignmentsTriedDropoffBNS += numAssignmentsTriedDropoffBNS;
            stats.numAssignmentsTriedDropoffORD += numAssignmentsTriedDropoffORD;
            stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.tryPostponedAssignmentsTime += tryPostponedTime;
            stats.numPostponedAssignments += numPostponed;
            stats.numStopPairs += numStopPairs;
            stats.numInputTransferPoints += numTransferPointsBuilt;
            stats.numNonPrunedTransferPoints += numTransferPoints;
            stats.intersectEllipsesTime += intersectEllipsesTime;
        }

    private:

        void findPartialAssignmentsForPairOfStopPairs(const int pStopId, const int dStopId,
            const RelevantPDLocs &relORDPickups,
            const RelevantPDLocs &relBNSPickups,
                                                      std::vector<AssignmentWithTransfer> &promisingPartials,
                                                      const PDLocs &pdLocs) {
            ++numStopPairs;

            const auto transferPoints = ellipseIntersector.getTransferPoints(pStopId, dStopId);
            numTransferPoints += transferPoints.size();

            // For fixed transfer indices, try to find possible pickups
            const auto &pVehId = routeState.vehicleIdOf(pStopId);
            const auto &dVehId = routeState.vehicleIdOf(dStopId);
            const auto trIdxPVeh = routeState.stopPositionOf(pStopId);
            const auto trIdxDVeh = routeState.stopPositionOf(dStopId);
            for (const auto &tp: transferPoints) {
                tryPickupBNS(pVehId, dVehId, trIdxPVeh, trIdxDVeh, tp, relBNSPickups, promisingPartials, pdLocs);
                tryPickupORD(pVehId, dVehId, trIdxPVeh, trIdxDVeh, tp, relORDPickups, promisingPartials, pdLocs);
            }
        }

        void tryPickupORD(const int pVehId, const int dVehId, const int trIdxPVeh, const int trIdxDVeh,
        const TransferPoint &tp,
        const RelevantPDLocs &relORDPickups,
                          std::vector<AssignmentWithTransfer> &promisingPartials,
                          const PDLocs &pdLocs) {
            if (trIdxPVeh == 0 || !relORDPickups.hasRelevantSpotsFor(pVehId))
                return;

            for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                if (pickup.stopIndex > trIdxPVeh)
                    break; // pickups in relevant stops are ordered by stop index

                // Build the partial assignment with the transfer point
                const PDLoc &pickupPDLoc = pdLocs.pickups[pickup.pdId];

                if (pickupPDLoc.loc == tp.loc || transferIsLaterOnRoute(dVehId, trIdxDVeh, tp.loc))
                    continue;

                AssignmentWithTransfer asgn(&fleet[pVehId], &fleet[dVehId], tp, pickupPDLoc, pickup.stopIndex,
                                            pickup.distToPDLoc,
                                            pickup.distFromPDLocToNextStop, trIdxPVeh, trIdxDVeh);

                finishDistancesPVeh(asgn);

                asgn.pickupType = ORDINARY;
                asgn.transferTypePVeh = ORDINARY;

                KASSERT(asgn.pickup.id >= 0);
                // Try the partial assignment
                tryPartialAssignment(asgn, promisingPartials);
            }
        }

        void tryPickupBNS(const int pVehId, const int dVehId, const int trIdxPVeh, const int trIdxDVeh,
        const TransferPoint &tp,
        const RelevantPDLocs &relBNSPickups,
        std::vector<AssignmentWithTransfer> &promisingPartials,
        const PDLocs &pdLocs) {
            if (!relBNSPickups.hasRelevantSpotsFor(pVehId))
                return;

            for (const auto &pickup: relBNSPickups.relevantSpotsFor(pVehId)) {
                // Build the partial assignment with the transfer point
                const PDLoc &pickupPDLoc = pdLocs.pickups[pickup.pdId];

                if (pickupPDLoc.loc == tp.loc || transferIsLaterOnRoute(dVehId, trIdxDVeh, tp.loc))
                    continue;

                AssignmentWithTransfer asgn(&fleet[pVehId], &fleet[dVehId], tp, pickupPDLoc, pickup.stopIndex,
                                            pickup.distToPDLoc,
                                            pickup.distFromPDLocToNextStop, trIdxPVeh, trIdxDVeh);
                finishDistancesPVeh(asgn);

                asgn.pickupType = BEFORE_NEXT_STOP;
                asgn.transferTypePVeh = trIdxPVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                KASSERT(asgn.pickup.id >= 0);
                // Try the partial assignment
                tryPartialAssignment(asgn, promisingPartials);
            }
        }

        bool transferIsLaterOnRoute(const int vehId, const int idx, const int loc) {
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            for (int i = idx + 1; i < stopLocations.size(); ++i) {
                if (stopLocations[i] == loc)
                    return true;
            }

            return false;
        }

        void
        tryPartialAssignment(AssignmentWithTransfer &asgn, std::vector<AssignmentWithTransfer> &promisingPartials) {
            // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) are set

            KASSERT(asgn.pVeh && asgn.pickup.id != INVALID_ID);
            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY || asgn.distToTransferPVeh == INFTY ||
                asgn.distFromTransferPVeh == INFTY)
                return;

            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    ++numPartialsTriedPickupBNS;
                    break;
                case ORDINARY:
                    ++numPartialsTriedPickupORD;
                    break;
                default:
                    KASSERT(false);
            }

            const auto lowerBoundCost = calc.calcLowerBoundForPartialOrdinaryTransfer<true>(asgn, requestState);

            if (lowerBoundCost.total >= requestState.getBestCost())
                return;

            promisingPartials.push_back(asgn);
        }

        void tryDropoffBNS(const AssignmentWithTransfer &partialAsgn,
                           const RelevantPDLocs &relBNSDropoffs,
                           std::vector<AssignmentWithTransfer> &postponedAssignments,
                           const PDLocs &pdLocs) {
            if (partialAsgn.transferIdxDVeh != 0 ||
                relBNSDropoffs.relevantSpotsFor(partialAsgn.dVeh->vehicleId).size() == 0)
                return;

            for (const auto &dropoff: relBNSDropoffs.relevantSpotsFor(partialAsgn.dVeh->vehicleId)) {
                KASSERT(partialAsgn.transferIdxDVeh == 0);
                KASSERT(dropoff.stopIndex == 0);

                const PDLoc &dropoffPDLoc = pdLocs.dropoffs[dropoff.pdId];
                if (dropoffPDLoc.loc == partialAsgn.transfer.loc)
                    continue;

                AssignmentWithTransfer newAssignment(partialAsgn);
                newAssignment.dropoff = pdLocs.dropoffs[dropoff.pdId];
                newAssignment.dropoffIdx = dropoff.stopIndex;

                newAssignment.distToDropoff = dropoff.distToPDLoc;
                newAssignment.distFromDropoff = dropoff.distFromPDLocToNextStop;
                finishDistancesDVeh(newAssignment, 0);

                newAssignment.dropoffType = BEFORE_NEXT_STOP;
                newAssignment.transferTypeDVeh = BEFORE_NEXT_STOP;

                if (newAssignment.dropoff.loc == newAssignment.transfer.loc)
                    continue;

                tryAssignment(newAssignment, postponedAssignments);
            }
        }

        void tryDropoffORD(const AssignmentWithTransfer &partialAsgn,
                           const RelevantPDLocs &relORDDropoffs,
                           std::vector<AssignmentWithTransfer> &postponedAssignments,
                           const PDLocs &pdLocs) {
            if (relORDDropoffs.relevantSpotsFor(partialAsgn.dVeh->vehicleId).size() == 0)
                return;

            for (const auto &dropoff: relORDDropoffs.relevantSpotsFor(partialAsgn.dVeh->vehicleId)) {
                if (dropoff.stopIndex < partialAsgn.transferIdxDVeh)
                    continue;

                const PDLoc &dropoffPDLoc = pdLocs.dropoffs[dropoff.pdId];
                if (dropoffPDLoc.loc == partialAsgn.transfer.loc)
                    continue;

                AssignmentWithTransfer newAssignment(partialAsgn);
                newAssignment.dropoffIdx = dropoff.stopIndex;
                newAssignment.dropoff = dropoffPDLoc;

                newAssignment.distToDropoff = dropoff.distToPDLoc;
                newAssignment.distFromDropoff = dropoff.distFromPDLocToNextStop;
                finishDistancesDVeh(newAssignment, 0);

                newAssignment.dropoffType = ORDINARY;
                newAssignment.transferTypeDVeh = newAssignment.transferIdxDVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                if (newAssignment.dropoff.loc == newAssignment.transfer.loc)
                    continue;

                tryAssignment(newAssignment, postponedAssignments);
            }
        }

        void
        tryDropoffALS(const AssignmentWithTransfer &partialAsgn, const RelevantDropoffsAfterLastStop &relALSDropoffs,
                      std::vector<AssignmentWithTransfer> &postponedAssignments,
                      const PDLocs &pdLocs) {
            const auto vehId = partialAsgn.dVeh->vehicleId;
            if (!relALSDropoffs.hasRelevantSpotsFor(vehId))
                return;

            for (const auto &dropoffAlsEntry: relALSDropoffs.relevantSpotsFor(vehId)) {
                const auto &dropoff = pdLocs.dropoffs[dropoffAlsEntry.dropoffId];
                int distanceToDropoff = dropoffAlsEntry.distToDropoff;
                KASSERT(distanceToDropoff < INFTY);

                if (dropoff.loc == partialAsgn.transfer.loc)
                    continue;

                AssignmentWithTransfer newAssignment(partialAsgn);
                newAssignment.dropoffIdx = routeState.numStopsOf(partialAsgn.dVeh->vehicleId) - 1;
                newAssignment.dropoff = dropoff;

                newAssignment.distToDropoff = distanceToDropoff;
                newAssignment.distFromDropoff = 0;
                finishDistancesDVeh(newAssignment, distanceToDropoff);

                newAssignment.dropoffType = AFTER_LAST_STOP;
                newAssignment.transferTypeDVeh = newAssignment.transferIdxDVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                tryAssignment(newAssignment, postponedAssignments);
            }
        }

        void finishAssignments(const int pVehId, const int dVehId,
                               std::vector<AssignmentWithTransfer> &postponedAssignments, const PDLocs &pdLocs) {
            // Method to finish the assignments that have lower bounds used
            std::vector<AssignmentWithTransfer> toCalculate;
            std::vector<AssignmentWithTransfer> currentlyCalculating;
            std::vector<AssignmentWithTransfer> temp;

            RequestCost total;
            // Start with the pickups with postponed bns distance
            for (auto &asgn: postponedAssignments) {
                if (asgn.pickupBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    searches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                } else {
                    toCalculate.push_back(asgn);
                }
            }

            postponedAssignments.clear();

            if (currentlyCalculating.size() > 0)
                searches.computeExactDistancesVia(fleet[pVehId], pdLocs.pickups);

            for (auto &asgn: currentlyCalculating) {
                KASSERT(searches.knowsCurrentLocationOf(pVehId));
                KASSERT(searches.knowsDistance(pVehId, asgn.pickup.id));
                const int distance = searches.getDistance(pVehId, asgn.pickup.id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;

                if (!asgn.isFinished()) {
//                    total = calc.calcBaseLowerBound<true>(asgn, requestState);
                    total = calc.calc(asgn, requestState);

                    if (total.total > requestState.getBestCost())
                        continue;

                    toCalculate.push_back(asgn);
                } else {
                    total = calc.calc(asgn, requestState);
                    requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, total);
                    continue;
                }
            }
            currentlyCalculating.clear();

            // Calculate the exact paired distance between pickup and transfer
            std::vector<int> sources;
            std::vector<int> targets;
            std::vector<int> offsets;

            for (auto &asgn: toCalculate) {
                if (asgn.pickupPairedLowerBoundUsed) {
                    const int transferRank = vehCh.rank(inputGraph.edgeTail(asgn.transfer.loc));
                    const int transferOffset = inputGraph.travelTime(asgn.transfer.loc);

                    pickupToTransferDistancesFinder.runQueryForTransferRank(transferRank);
                    const int distance =
                            pickupToTransferDistancesFinder.getDistances().getDistance(asgn.pickup.id, transferRank) +
                            transferOffset;

                    asgn.distToTransferPVeh = distance;
                    asgn.pickupPairedLowerBoundUsed = false;

                    // Try the assignments with the calculated distances
                    if (!asgn.isFinished()) {
//                        total = calc.calcBaseLowerBound<true>(asgn, requestState);
                        total = calc.calc(asgn, requestState);

                        if (total.total > requestState.getBestCost())
                            continue;

                        temp.push_back(asgn);
                    } else {
                        total = calc.calc(asgn, requestState);
                        requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, total);
                        continue;
                    }
                } else {
                    temp.push_back(asgn);
                }
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();
            currentlyCalculating.clear();
            // Calculate the dropoffs with postponed bns distance
            for (auto &asgn: toCalculate) {
                if (asgn.dropoffBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    asgn.dropoffBNSLowerBoundUsed = false;
                    searches.addTransferForProcessing(asgn.transfer.loc, asgn.distToTransferDVeh);
                } else {
                    temp.push_back(asgn);
                }
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();

            if (currentlyCalculating.size() > 0)
                searches.computeExactTransferDistancesVia(fleet[dVehId]);

            for (auto &asgn: currentlyCalculating) {
                KASSERT(searches.knowsDistanceTransfer(dVehId, asgn.transfer.loc));
                KASSERT(searches.knowsCurrentLocationOf(dVehId));
                const int distance = searches.getDistanceTransfer(dVehId, asgn.transfer.loc);
                asgn.distToTransferDVeh = distance;
                asgn.dropoffBNSLowerBoundUsed = false;

                if (!asgn.isFinished()) {
//                    total = calc.calcBaseLowerBound<true>(asgn, requestState);
                    total = calc.calc(asgn, requestState);

                    if (total.total > requestState.getBestCost())
                        continue;

                    toCalculate.push_back(asgn);
                } else {
                    total = calc.calc(asgn, requestState);
                    requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, total);
                }
            }

            // Calculate the exact paired distance between transfer and dropoff
            currentlyCalculating.clear();

            sources.clear();
            targets.clear();
            offsets.clear();

            for (auto &asgn: toCalculate) {
                KASSERT(asgn.dropoffPairedLowerBoundUsed);

                const int transferRank = vehCh.rank(inputGraph.edgeHead(asgn.transfer.loc));
                transferToDropoffDistancesFinder.runQueryForTransferRank(transferRank);
                const int distance = transferToDropoffDistancesFinder.getDistances().getDistance(asgn.dropoff.id,
                                                                                                 transferRank);

                asgn.distToDropoff = distance;
                asgn.dropoffPairedLowerBoundUsed = false;

                // Try the assignments with the calculated distances
                KASSERT(asgn.isFinished());
                total = calc.calc(asgn, requestState);
                requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, total);
            }
        }

        void finishDistancesPVeh(AssignmentWithTransfer &asgn) {
            const auto stopLocations = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const int numStops = routeState.numStopsOf(asgn.pVeh->vehicleId);
            unused(numStops);

            const auto schedDepTimes = routeState.schedDepTimesFor(asgn.pVeh->vehicleId);
            const auto schedArrTimes = routeState.schedArrTimesFor(asgn.pVeh->vehicleId);

            const int pickupIdx = asgn.pickupIdx;
            const int transferIdx = asgn.transferIdxPVeh;

            KASSERT(pickupIdx < numStops - 1);

            const int pickup = asgn.pickup.loc;
            const int transfer = asgn.transfer.loc;

            const bool bns = pickupIdx == 0;
            const bool paired = pickupIdx == transferIdx;

            const bool pickupAtStop = stopLocations[pickupIdx] == pickup;
            const bool transferAtStop = stopLocations[transferIdx] == transfer;

            const int legPickup = schedArrTimes[pickupIdx + 1] - schedDepTimes[pickupIdx];
            const int legTransfer = transferIdx == numStops - 1? 0 : schedArrTimes[transferIdx + 1] - schedDepTimes[transferIdx];

            //* Pickup distances
            if (pickupAtStop)
                asgn.distToPickup = 0;

            if (paired) {
                // Paired Assignment (pVeh)
                // Try the lower bound for the paired assignment
                asgn.distFromPickup = 0;
                asgn.pickupPairedLowerBoundUsed = true;
            }

            if (pickupAtStop && !paired)
                asgn.distFromPickup = legPickup;

            if (bns) {
                // Assignment with pickup BNS
                if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup.id)) {
                    asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup.id);
                } else {
                    asgn.pickupBNSLowerBoundUsed = true;
                }
            }

            //* Transfer distances pVeh
            if (paired) {
                asgn.distToTransferPVeh = 0;
                asgn.pickupPairedLowerBoundUsed = true;
            }

            if (transferAtStop && !paired)
                asgn.distToTransferPVeh = 0;

            if (transferAtStop)
                asgn.distFromTransferPVeh = legTransfer;

        }

        void finishDistancesDVeh(AssignmentWithTransfer &asgn, const int dropoffAlsDistance) {
            const auto stopLocations = routeState.stopLocationsFor(asgn.dVeh->vehicleId);
            const int numStops = routeState.numStopsOf(asgn.dVeh->vehicleId);

            const auto schedDepTimes = routeState.schedDepTimesFor(asgn.dVeh->vehicleId);
            const auto schedArrTimes = routeState.schedArrTimesFor(asgn.dVeh->vehicleId);

            const int transferIdx = asgn.transferIdxDVeh;
            const int dropoffIdx = asgn.dropoffIdx;

            const int transfer = asgn.transfer.loc;
            const int dropoff = asgn.dropoff.loc;

            const bool bns = transferIdx == 0;
            const bool paired = transferIdx == dropoffIdx;
            const bool dropoffALS = dropoffIdx == numStops - 1;

            const bool transferAtStop = stopLocations[transferIdx] == transfer;
            const bool dropoffAtStop = stopLocations[dropoffIdx] == dropoff;

            const int legTransfer = transferIdx == numStops - 1? 0 : schedArrTimes[transferIdx + 1] - schedDepTimes[transferIdx];
            const int legDropoff =
                    dropoffIdx == numStops - 1 ? 0 : schedArrTimes[dropoffIdx + 1] - schedDepTimes[dropoffIdx];

            //* Transfer distances
            if (transferAtStop)
                asgn.distToTransferDVeh = 0;

            if (paired) {
                // Paired Assignment (dVeh)
                // Try the lower bound for the paired assignment
                asgn.distFromTransferDVeh = 0;
                asgn.dropoffPairedLowerBoundUsed = true;
            }

            if (transferAtStop && !paired)
                asgn.distFromTransferDVeh = legTransfer;

            if (bns) {
                // Transfer bns in dropoff vehicle
                if (searches.knowsDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc)) {
                    asgn.distToTransferDVeh = searches.getDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc);
                } else {
                    asgn.dropoffBNSLowerBoundUsed = true;
                }
            }

            //* Dropoff distances
            if (paired) {
                asgn.distToDropoff = 0;
                asgn.dropoffPairedLowerBoundUsed = true;
            }

            if (dropoffAtStop && !paired)
                asgn.distToDropoff = 0;

            if (dropoffALS)
                asgn.distFromDropoff = dropoffAlsDistance;

            if (dropoffAtStop)
                asgn.distFromDropoff = legDropoff;
        }

        void tryAssignment(AssignmentWithTransfer &asgn, std::vector<AssignmentWithTransfer> &postponedAssignments) {
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);

            if ((asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup.loc == stopLocationsPVeh[asgn.pickupIdx + 1])
                || (asgn.transferIdxPVeh < numStopsPVeh - 1 &&
                    asgn.transfer.loc == stopLocationsPVeh[asgn.transferIdxPVeh + 1])
                || (asgn.transferIdxDVeh < numStopsDVeh - 1 &&
                    asgn.transfer.loc == stopLocationsDVeh[asgn.transferIdxDVeh + 1])
                || (asgn.dropoffIdx < numStopsDVeh - 1 && asgn.dropoff.loc == stopLocationsDVeh[asgn.dropoffIdx + 1]))
                return;

            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    ++numAssignmentsTriedPickupBNS;
                    break;
                case ORDINARY:
                    ++numAssignmentsTriedPickupORD;
                    break;
                default:
                    KASSERT(false);
            }

            switch (asgn.dropoffType) {
                case BEFORE_NEXT_STOP:
                    ++numAssignmentsTriedDropoffBNS;
                    break;
                case ORDINARY:
                    ++numAssignmentsTriedDropoffORD;
                    break;
                case AFTER_LAST_STOP:
                    ++numAssignmentsTriedDropoffALS;
                    break;
                default:
                    KASSERT(false);
            }

            if (!asgn.isFinished()) {
                const auto cost = calc.calc(asgn, requestState);
                if (cost.total <= requestState.getBestCost()) {
                    postponedAssignments.push_back(asgn);
                }
            } else {
                const auto cost = calc.calc(asgn, requestState);
                requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, cost);
            }
        }

        bool assertTransferPointCalculation(const TransferPoint tp) {
            const auto stopLocationsPVeh = routeState.stopLocationsFor(tp.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(tp.dVeh->vehicleId);

            const auto stopIndexPVeh = tp.stopIdxDVeh;
            const auto stopIndexDVeh = tp.stopIdxPVeh;

            const auto stopLocPVeh = stopLocationsPVeh[stopIndexPVeh];
            const auto nextStopLocPVeh = stopLocationsPVeh[stopIndexPVeh + 1];
            const auto stopLocDVeh = stopLocationsDVeh[stopIndexDVeh];
            const auto nextStopLocDVeh = stopLocationsDVeh[stopIndexDVeh + 1];

            const auto headStopPVeh = inputGraph.edgeHead(stopLocPVeh);
            const auto tailNextStopPVeh = inputGraph.edgeTail(nextStopLocPVeh);
            const auto headStopDVeh = inputGraph.edgeHead(stopLocDVeh);
            const auto tailNextStopDVeh = inputGraph.edgeTail(nextStopLocDVeh);

            // Calculate leg length pVeh and dVeh
            const auto headStopPVehRank = vehCh.rank(headStopPVeh);
            const auto tailNextStopPVehRank = vehCh.rank(tailNextStopPVeh);
            const auto headStopDVehRank = vehCh.rank(headStopDVeh);
            const auto tailNextStopDVehRank = vehCh.rank(tailNextStopDVeh);

            const auto nextStopPVehLength = inputGraph.travelTime(nextStopLocPVeh);
            const auto nextStopDVehLength = inputGraph.travelTime(nextStopLocDVeh);

            vehChQuery.run(headStopPVehRank, tailNextStopPVehRank);
            const int legLenthPVeh = vehChQuery.getDistance() + nextStopPVehLength;

            vehChQuery.run(headStopDVehRank, tailNextStopDVehRank);
            const int legLenthDVeh = vehChQuery.getDistance() + nextStopDVehLength;

            const auto routeStateLengthPVeh = time_utils::calcLengthOfLegStartingAt(stopIndexPVeh, tp.pVeh->vehicleId,
                                                                                    routeState);
            const auto routeStateLengthDVeh = time_utils::calcLengthOfLegStartingAt(stopIndexDVeh, tp.dVeh->vehicleId,
                                                                                    routeState);

            unused(legLenthPVeh, legLenthDVeh, routeStateLengthPVeh, routeStateLengthDVeh);
            KASSERT(routeStateLengthPVeh == legLenthPVeh);
            KASSERT(routeStateLengthDVeh == legLenthDVeh);

            // Recalculate the distance to and from the transfer point
            const auto transferPointHead = inputGraph.edgeHead(tp.loc);
            const auto transferPointTail = inputGraph.edgeTail(tp.loc);
            const auto transferPointHeadRank = vehCh.rank(transferPointHead);
            const auto transferPointTailRank = vehCh.rank(transferPointTail);
            const auto transferLength = inputGraph.travelTime(tp.loc);

            vehChQuery.run(headStopPVehRank, transferPointTailRank);
            const auto distToTransferPVeh = stopLocPVeh == tp.loc ? 0 : vehChQuery.getDistance() + transferLength;

            vehChQuery.run(transferPointHeadRank, tailNextStopPVehRank);
            const auto distFromTransferPVeh =
                    nextStopLocPVeh == tp.loc ? 0 : vehChQuery.getDistance() + nextStopPVehLength;

            vehChQuery.run(headStopDVehRank, transferPointTailRank);
            const auto distToTransferDVeh = stopLocDVeh == tp.loc ? 0 : vehChQuery.getDistance() + transferLength;

            vehChQuery.run(transferPointHeadRank, tailNextStopDVehRank);
            const auto distFromTransferDVeh =
                    nextStopLocDVeh == tp.loc ? 0 : vehChQuery.getDistance() + nextStopDVehLength;

            if (distToTransferPVeh != tp.distancePVehToTransfer ||
                distFromTransferPVeh != tp.distancePVehFromTransfer ||
                distToTransferDVeh != tp.distanceDVehToTransfer ||
                distFromTransferDVeh != tp.distanceDVehFromTransfer) {
                KASSERT(false);
            }

            return true;
        }

        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

        CurVehLocToPickupSearchesT &searches;

        DirectTransferDistancesFinderT &pickupToTransferDistancesFinder;
        DirectTransferDistancesFinderT &transferToDropoffDistancesFinder;

        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;

        CostCalculator &calc;
        InsertionAsserterT &asserter;

        using EllipseSizeLogger = NullLogger;
        using EllipseIntersectionSizeLogger = NullLogger;
        EdgeEllipseIntersector<InputGraphT, UseCostLowerBounds, DoTransferPointParetoChecks, EllipseSizeLogger, EllipseIntersectionSizeLogger> ellipseIntersector;

        LightweightSubset edgesSubset; // Subset used to deduplicate locations in preparing any-to-any searches

        FastResetFlagArray<uint32_t> stopSeen;

        //* Statistics for the ordinary transfer assignment finder
        int64_t totalTime;

        // Stats for the tried assignments
        int64_t numPartialsTriedPickupBNS;
        int64_t numPartialsTriedPickupORD;

        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;

        int64_t numAssignmentsTriedDropoffBNS;
        int64_t numAssignmentsTriedDropoffORD;
        int64_t numAssignmentsTriedDropoffALS;

        // Stats for the transfer search itself
        int64_t numStopPairs;
        int64_t numTransferPoints;

    };
}
