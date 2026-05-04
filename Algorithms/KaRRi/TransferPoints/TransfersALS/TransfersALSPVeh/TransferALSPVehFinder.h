/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/CHStrategyALS.h"

#include <tbb/enumerable_thread_specific.h>

#pragma once

namespace karri {

    template<typename InputGraphT,
            typename VehCHEnvT,
            typename TransferALSStrategyT,
            typename TransfersPickupALSStrategyT,
            typename CurVehLocToPickupSearchesT,
            bool UseCostLowerBounds,
            bool DoTransferPointParetoChecks,
            typename InsertionAsserterT>
    class TransferALSPVehFinder {


        struct TPDistances {
            int distToTransferPVeh = INFTY;
            int distToTransferDVeh = INFTY;
            int distFromTransferDVeh = INFTY;

            static bool dominates(const TPDistances &lhs, const TPDistances &rhs) {
                return lhs.distToTransferPVeh < rhs.distToTransferPVeh &&
                       lhs.distToTransferDVeh < rhs.distToTransferDVeh &&
                       lhs.distFromTransferDVeh < rhs.distFromTransferDVeh;
            }
        };


        struct WorkUnit {
            int pVehId = INVALID_ID;
            int dVehId = INVALID_ID;
            INS_TYPES pVehType = NOT_SET;
            INS_TYPES dVehType = NOT_SET;
        };

        struct NumAsgnStats {
            int64_t numTransferPoints = 0;
            int64_t numAssignmentsTriedPickupBNS = 0;
            int64_t numAssignmentsTriedPickupORD = 0;
            int64_t numAssignmentsTriedPickupALS = 0;
            int64_t numAssignmentsTriedDropoffORD = 0;
            int64_t numAssignmentsTriedDropoffALS = 0;

            void reset() {
                numTransferPoints = 0;
                numAssignmentsTriedPickupBNS = 0;
                numAssignmentsTriedPickupORD = 0;
                numAssignmentsTriedPickupALS = 0;
                numAssignmentsTriedDropoffORD = 0;
                numAssignmentsTriedDropoffALS = 0;
            }

            NumAsgnStats &operator+=(const NumAsgnStats &other) {
                numTransferPoints += other.numTransferPoints;
                numAssignmentsTriedPickupBNS += other.numAssignmentsTriedPickupBNS;
                numAssignmentsTriedPickupORD += other.numAssignmentsTriedPickupORD;
                numAssignmentsTriedPickupALS += other.numAssignmentsTriedPickupALS;
                numAssignmentsTriedDropoffORD += other.numAssignmentsTriedDropoffORD;
                numAssignmentsTriedDropoffALS += other.numAssignmentsTriedDropoffALS;
                return *this;
            }
        };

        // The pVeh drives the detour to the transfer point
        // This implies, that the pVeh drives from its last stop to a stop of the dVeh and drops off the customer
        // The dVeh then will then perform the dropoff ORD or ALS
        // The pickup could be BNS, ORD or ALS
    public:

        using RelevantPDLoc = RelevantPDLocs::RelevantPDLoc;

        TransferALSPVehFinder(
                const InputGraphT &inputGraph,
                VehCHEnvT &vehChEnv,
                TransferALSStrategyT &strategy,
                TransfersPickupALSStrategyT &pickupALSStrategy,
                CurVehLocToPickupSearchesT &searches,
                const Fleet &fleet,
                const RouteState &routeState,
                RequestState &requestState,
                CostCalculator &calc,
                InsertionAsserterT &asserter
        ) : inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            strategy(strategy),
            pickupALSStrategy(pickupALSStrategy),
            searches(searches),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(calc),
            asserter(asserter),
            relPVehToInternalIdx(fleet.size(), INVALID_INDEX),
            dVehStopsFlags(fleet.size()),
            isEdgeRel(inputGraph.numEdges()),
            relEdgesToInternalIdx(inputGraph.numEdges()),
            threadLocalCalc([&]() { return CostCalculator(routeState, fleet); }) {}

        void init() {}

        template<typename EllipsesT>
        void findAssignments(const RelevantPDLocs &relORDPickups, const RelevantPDLocs &relBNSPickups,
                            const RelevantPDLocs &relORDDropoffs,
            const RelevantDropoffsAfterLastStop &relALSDropoffs,
            const PDDistances &pdDistances,
            const EllipsesT &ellipseContainer) {
            Timer total;
            Timer innerTimer;

            //* Collect the full ellipses of possible transfer points
            // Collect all stop ids of last stops of potential pickup vehicles (without als, as for pickup als we first have to search to the pickups and the from the pickups)

            relPVehToInternalIdx.clear(); // Reset all back to INVALID_INDEX

            std::vector<int> relevantLastStopLocs;

            int numRelPVehs = 0;
            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                relPVehToInternalIdx[pVehId] = numRelPVehs++;

                const auto numStops = routeState.numStopsOf(pVehId);
                const auto lastStopLoc = routeState.stopLocationsFor(pVehId)[numStops - 1];
                relevantLastStopLocs.push_back(lastStopLoc);
            }

            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                if (relPVehToInternalIdx[pVehId] != INVALID_INDEX)
                    continue;
                relPVehToInternalIdx[pVehId] = numRelPVehs++;

                const auto numStops = routeState.numStopsOf(pVehId);
                const auto lastStopLocs = routeState.stopLocationsFor(pVehId)[numStops - 1];
                relevantLastStopLocs.push_back(lastStopLocs);
            }

            if (dVehStopsFlags.size() <= routeState.getMaxStopId())
                dVehStopsFlags.resize(routeState.getMaxStopId() + 1);
            dVehStopsFlags.reset();
            allTransferEdges.clear();
            isEdgeRel.reset();
            for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto numStops = routeState.numStopsOf(dVehId);
                if (numStops <= 1)
                    continue;

                const auto &stopIds = routeState.stopIdsFor(dVehId);
                for (int i = 0; i < numStops; i++) {
                    const auto stopId = stopIds[i];
                    if (dVehStopsFlags.isSet(stopId))
                        continue;
                    dVehStopsFlags.set(stopId);
                    const auto &ellipse = ellipseContainer.getEdgesInEllipse(stopId);
                    for (const auto &e: ellipse) {
                        const auto loc = e.edge;
                        if (!isEdgeRel.isSet(loc)) {
                            relEdgesToInternalIdx[loc] = static_cast<int>(allTransferEdges.size());
                            isEdgeRel.set(loc);
                            allTransferEdges.push_back(loc);
                        }
                    }
                }
            }

            for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto numStops = routeState.numStopsOf(dVehId);
                if (numStops <= 1)
                    continue;

                const auto &rel = relORDDropoffs.relevantSpotsFor(dVehId);
                const int latestRelevantStopIdx = std::min(numStops - 1, rel[rel.size() - 1].stopIndex);
                const auto &stopIds = routeState.stopIdsFor(dVehId);
                for (int i = 0; i <= latestRelevantStopIdx; ++i) {
                    const auto stopId = stopIds[i];
                    if (dVehStopsFlags.isSet(stopId))
                        continue;
                    dVehStopsFlags.set(stopId);
                    const auto &ellipse = ellipseContainer.getEdgesInEllipse(stopId);
                    for (const auto &e: ellipse) {
                        const auto loc = e.edge;
                        if (!isEdgeRel.isSet(loc)) {
                            relEdgesToInternalIdx[loc] = static_cast<int>(allTransferEdges.size());
                            isEdgeRel.set(loc);
                            allTransferEdges.push_back(loc);
                        }
                    }
                }
            }

            if (allTransferEdges.empty())
                return;

            std::vector<int> pickupLocs;
            for (const auto &pickup: requestState.pickups) {
                pickupLocs.push_back(pickup.loc);
            }

            std::vector<int> dropoffLocs;
            for (const auto &dropoff: requestState.dropoffs) {
                dropoffLocs.push_back(dropoff.loc);
            }

            // Initialize the transfer strategy with the collected transfer edges.
            strategy.init(allTransferEdges);

            const auto initTime = innerTimer.elapsed<std::chrono::nanoseconds>();

            innerTimer.restart();
            const auto &pVehIdsALS = pickupALSStrategy.findPickupsAfterLastStop(pdDistances);
            const auto searchTimePickupALS = innerTimer.elapsed<std::chrono::nanoseconds>();

            if (relevantLastStopLocs.empty() && pVehIdsALS.empty())
                return;

            // Calculate the distances from all last stops (pickup ord, bns) to the potential transfers
            innerTimer.restart();
            const auto &lastStopToTransfersDistances = strategy.calculateDistancesFromLastStopToAllTransfers(
                    relevantLastStopLocs, allTransferEdges);
            const int64_t searchTimeLastStopToTransfer = innerTimer.elapsed<std::chrono::nanoseconds>();

            // Calculate the distances from all pickups to the potential transfers
            innerTimer.restart();
            // pickupToTransfersDistances[i][j] stores the distance from i-th pickup to the j-th edge in transferEdges
            const auto &pickupsToTransfersDistances = strategy.calculateDistancesFromPickupsToAllTransfers(pickupLocs,
                                                                                                           allTransferEdges);
            const int64_t searchTimePickupToTransfer = innerTimer.elapsed<std::chrono::nanoseconds>();

            // Calculate the distances from all transfers to the dropoffs
            innerTimer.restart();
            // transferToDropoffDistances[i][j] stores the distance from the j-th edge in transferEdges to the i-th dropoff
            const auto &transfersToDropoffsDistances = strategy.calculateDistancesFromAllTransfersToDropoffs(
                    allTransferEdges, dropoffLocs);
            const auto searchTimeTransferToDropoff = innerTimer.elapsed<std::chrono::nanoseconds>();

            innerTimer.restart();
            const auto workUnits = composeWorkUnits(pVehIdsALS, relORDPickups, relBNSPickups, relORDDropoffs, relALSDropoffs);


            // Hacky: Access every PALS distance that may be read once to make future reads of the underlying
            // time-stamped vector thread-safe.
            for (const auto pVehId: pVehIdsALS) {
                for (const auto &pickup: requestState.pickups) {
                    pickupALSStrategy.getDistanceToPickup(pVehId, pickup.id);
                }
            }

            globalBestCost = RequestCost::INFTY_COST();
            globalBestAssignment = AssignmentWithTransfer();

            for (auto &local: threadLocalData) {
                local.bestAsgn = AssignmentWithTransfer();
                local.bestCost = RequestCost::INFTY_COST();
                local.paretoOptimalTps.clear();
                local.postponedAssignments.clear();
                local.numAsgnStats.reset();
            }

            // Process all work units
            tbb::parallel_for(0ul, workUnits.size(), [&](const auto i) {
                const auto &wu = workUnits[i];
                auto &local = threadLocalData.local();
                auto &localCalc = threadLocalCalc.local();
                processWorkUnit(wu, relORDPickups, relBNSPickups, relORDDropoffs, relALSDropoffs, lastStopToTransfersDistances, pickupsToTransfersDistances,
                                transfersToDropoffsDistances, ellipseContainer, local.postponedAssignments,
                                local.paretoOptimalTps, localCalc, local.bestAsgn, local.bestCost, local.numAsgnStats);
            });

            std::vector<AssignmentWithTransfer> postponedPBNSAssignments;
            NumAsgnStats globalNumAsgnStats;
            for (auto &local: threadLocalData) {
                postponedPBNSAssignments.insert(postponedPBNSAssignments.end(),
                                                std::make_move_iterator(local.postponedAssignments.begin()),
                                                std::make_move_iterator(local.postponedAssignments.end()));

                if (local.bestCost.total < globalBestCost.total) {
                    globalBestCost = local.bestCost;
                    globalBestAssignment = local.bestAsgn;
                }

                globalNumAsgnStats += local.numAsgnStats;
            }

            Timer finishPbnsTimer;
            const auto numPostponedPbnsAssignments = postponedPBNSAssignments.size();
            finishPostponedPBNSAssignments(postponedPBNSAssignments, globalNumAsgnStats);
            const auto finishPbnsTime = finishPbnsTimer.elapsed<std::chrono::nanoseconds>();

            // Try best assignment found
            if (globalBestCost.total < INFTY)
                requestState.tryFinishedTransferAssignmentWithKnownCost(globalBestAssignment, globalBestCost);

            const auto tryAssignmentsTime = innerTimer.elapsed<std::chrono::nanoseconds>();

            KASSERT(globalBestCost.total >= INFTY || asserter.assertAssignment(globalBestAssignment));

            // Write the stats
            auto &stats = requestState.stats().transferALSPVehStats;
            stats.totalTime = total.elapsed<std::chrono::nanoseconds>();
            stats.initTime = initTime;

            stats.numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesPickupALS += pVehIdsALS.size();
            stats.numCandidateVehiclesDropoffORD += relORDDropoffs.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesDropoffALS += relALSDropoffs.getVehiclesWithRelevantPDLocs().size();

            stats.numAssignmentsTriedPickupBNS += globalNumAsgnStats.numAssignmentsTriedPickupBNS;
            stats.numAssignmentsTriedPickupORD += globalNumAsgnStats.numAssignmentsTriedPickupORD;
            stats.numAssignmentsTriedPickupALS += globalNumAsgnStats.numAssignmentsTriedPickupALS;
            stats.numAssignmentsTriedDropoffORD += globalNumAsgnStats.numAssignmentsTriedDropoffORD;
            stats.numAssignmentsTriedDropoffALS += globalNumAsgnStats.numAssignmentsTriedDropoffALS;

            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.tryPostponedAssignmentsTime += finishPbnsTime;
            stats.numPostponedAssignments += numPostponedPbnsAssignments;

            stats.numInputTransferPoints += allTransferEdges.size();
            stats.numNonPrunedTransferPoints += globalNumAsgnStats.numTransferPoints;

            stats.searchTimePickupALS += searchTimePickupALS;
            stats.searchTimeLastStopToTransfer += searchTimeLastStopToTransfer;
            stats.searchTimePickupToTransfer += searchTimePickupToTransfer;
            stats.searchTimeTransferToDropoff += searchTimeTransferToDropoff;
        }

    private:

        std::vector<WorkUnit> composeWorkUnits(const LightweightSubset &pVehIdsALS,
            const RelevantPDLocs &relORDPickups, const RelevantPDLocs &relBNSPickups,
            const RelevantPDLocs &relORDDropoffs,
                                               const RelevantDropoffsAfterLastStop &relALSDropoffs) {
            std::vector<WorkUnit> workUnits;

            // Ordinary pickups
            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (dVehId == pVehId)
                        continue;
                    workUnits.emplace_back(pVehId, dVehId, INS_TYPES::ORDINARY, INS_TYPES::ORDINARY);
                }
                for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (dVehId == pVehId)
                        continue;
                    workUnits.emplace_back(pVehId, dVehId, INS_TYPES::ORDINARY, INS_TYPES::AFTER_LAST_STOP);
                }
            }

            // BNS pickups
            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (dVehId == pVehId)
                        continue;
                    workUnits.emplace_back(pVehId, dVehId, INS_TYPES::BEFORE_NEXT_STOP, INS_TYPES::ORDINARY);
                }
                for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (dVehId == pVehId)
                        continue;
                    workUnits.emplace_back(pVehId, dVehId, INS_TYPES::BEFORE_NEXT_STOP, INS_TYPES::AFTER_LAST_STOP);
                }
            }

            // ALS pickups
            for (const auto pVehId: pVehIdsALS) {
                for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (dVehId == pVehId)
                        continue;
                    workUnits.emplace_back(pVehId, dVehId, INS_TYPES::AFTER_LAST_STOP, INS_TYPES::ORDINARY);
                }
                for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (dVehId == pVehId)
                        continue;
                    workUnits.emplace_back(pVehId, dVehId, INS_TYPES::AFTER_LAST_STOP, INS_TYPES::AFTER_LAST_STOP);
                }
            }

            return workUnits;
        }

        void processWorkUnit(const WorkUnit &wu,
            const RelevantPDLocs &relORDPickups, const RelevantPDLocs &relBNSPickups, const RelevantPDLocs &relORDDropoffs,
                             const RelevantDropoffsAfterLastStop &relALSDropoffs,
                             const FlatRegular2DDistanceArray &lastStopToTransfersDistances,
                             const FlatRegular2DDistanceArray &pickupsToTransfersDistances,
                             const FlatRegular2DDistanceArray &transfersToDropoffsDistances,
                             const EdgeEllipseContainer &ellipseContainer,
                             std::vector<AssignmentWithTransfer> &postponedPBNSAssignments,
                             std::vector<TPDistances> &localParetoOptimalTps,
                             CostCalculator &localCalc,
                             AssignmentWithTransfer &localBestAssignment,
                             RequestCost &localBestCost,
                             NumAsgnStats &localNumAsgnStats) {
            static std::vector<AssignmentWithTransfer> placeholderPostponedAssignments;
            KASSERT(placeholderPostponedAssignments.empty());
            static std::vector<int> placeholderLastStopDistances;
            static ConstantVectorRange<int> placeholderLastStopDistanceRange(
                    placeholderLastStopDistances.begin(), placeholderLastStopDistances.end());
            static int placeholderMinLastStopDistance = INFTY;

            if (wu.pVehType == ORDINARY || wu.pVehType == BEFORE_NEXT_STOP) {

                const auto &thisLastStopToTransfersDistances = lastStopToTransfersDistances.getDistancesFor(
                        relPVehToInternalIdx[wu.pVehId]);
                const int minThisLastStopToTransferDistance = lastStopToTransfersDistances.getMinDistanceFor(
                        relPVehToInternalIdx[wu.pVehId]);

                const auto &pickupEntries = wu.pVehType == ORDINARY ?
                                            relORDPickups.relevantSpotsFor(wu.pVehId) :
                                            relBNSPickups.relevantSpotsFor(wu.pVehId);
                auto &postponedToUse =
                        wu.pVehType == ORDINARY ? placeholderPostponedAssignments : postponedPBNSAssignments;

                for (const auto &pickupEntry: pickupEntries) {
                    const auto &thisPickupToTransfersDistances = pickupsToTransfersDistances.getDistancesFor(
                            pickupEntry.pdId);
                    const int minThisPickupToTransferDistance = pickupsToTransfersDistances.getMinDistanceFor(
                            pickupEntry.pdId);
                    if (wu.dVehType == ORDINARY) {
                        tryDropoffORD(wu.pVehId, pickupEntry, wu.dVehId, relORDDropoffs, postponedToUse,
                                      thisLastStopToTransfersDistances, minThisLastStopToTransferDistance,
                                      thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                                      transfersToDropoffsDistances, ellipseContainer, localParetoOptimalTps,
                                      localCalc, localBestAssignment, localBestCost, localNumAsgnStats);
                    } else if (wu.dVehType == AFTER_LAST_STOP) {
                        tryDropoffALS(wu.pVehId, pickupEntry, wu.dVehId, relALSDropoffs, postponedToUse,
                                      thisLastStopToTransfersDistances, minThisLastStopToTransferDistance,
                                      thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                                      transfersToDropoffsDistances, ellipseContainer, localParetoOptimalTps,
                                      localCalc, localBestAssignment, localBestCost, localNumAsgnStats);
                    }
                }
            } else if (wu.pVehType == AFTER_LAST_STOP) {

                for (const auto &pickup: requestState.pickups) {
                    // Get the distance from the last stop of the pVeh to the pickup
                    const int distanceToPickup = pickupALSStrategy.getDistanceToPickup(wu.pVehId, pickup.id);
                    if (distanceToPickup >= INFTY)
                        continue; // Pickup is not reachable

                    const int minThisPickupToTransferDistance = pickupsToTransfersDistances.getMinDistanceFor(
                            pickup.id);
                    if (minThisPickupToTransferDistance >= INFTY)
                        continue; // No transfer is reachable from this pickup

                    KASSERT(!relALSDropoffs.getVehiclesWithRelevantPDLocs().empty() ||
                            !relORDDropoffs.getVehiclesWithRelevantPDLocs().empty());
                    const auto &thisPickupToTransfersDistances = pickupsToTransfersDistances.getDistancesFor(
                            pickup.id);

                    if (wu.dVehType == ORDINARY) {
                        tryDropoffORDForPickupALS(wu.pVehId, pickup, distanceToPickup, wu.dVehId, relORDDropoffs,
                                                  placeholderLastStopDistanceRange, placeholderMinLastStopDistance,
                                                  thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                                                  transfersToDropoffsDistances, ellipseContainer, localParetoOptimalTps,
                                                  localCalc, localBestAssignment, localBestCost,
                                                  localNumAsgnStats);
                    } else if (wu.dVehType == AFTER_LAST_STOP) {
                        tryDropoffALSForPickupALS(wu.pVehId, pickup, distanceToPickup, wu.dVehId,
                                                  placeholderLastStopDistanceRange, placeholderMinLastStopDistance,
                                                  thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                                                  transfersToDropoffsDistances, relALSDropoffs, ellipseContainer,
                                                  localParetoOptimalTps, localCalc, localBestAssignment, localBestCost,
                                                  localNumAsgnStats);
                    }
                }
            }
        }

        // Checks if the new transfer point distance is dominated by any of the existing optimal distances.
        // If yes, returns false. If no, adds the new distance to the list of optimal distances, removes any
        // distances that are dominated by the new distance, and returns true.
        static bool checkPareto(const TPDistances &newTpDist, std::vector<TPDistances> &optimalDistances) {
            // Check if newTpDist is dominated by any of the existing distances
            for (const auto &existingDist: optimalDistances) {
                if (TPDistances::dominates(existingDist, newTpDist)) {
                    return false; // newTpDist is dominated, do not add it
                }
            }

            // Add newTpDist to the list of optimal distances
            optimalDistances.push_back(newTpDist);

            // Remove any distances that are dominated by the newTpDist
            auto numOpt = optimalDistances.size();
            for (int i = 0; i < numOpt;) {
                if (TPDistances::dominates(newTpDist, optimalDistances[i])) {
                    std::swap(optimalDistances[i], optimalDistances.back());
                    optimalDistances.pop_back();
                    --numOpt;
                    continue;
                }
                ++i;
            }

            return true;
        }

        void tryDropoffORD(const int pVehId, const RelevantPDLoc &pickupEntry, const int dVehId,
            const RelevantPDLocs &relORDDropoffs,
                           std::vector<AssignmentWithTransfer> &postponedAssignments,
                           const ConstantVectorRange<int> &thisLastStopToTransfersDistances,
                           const int minThisLastStopToTransferDistance,
                           const ConstantVectorRange<int> &thisPickupToTransfersDistances,
                           const int minThisPickupToTransferDistance,
                           const FlatRegular2DDistanceArray &transfersToDropoffsDistances,
                           const EdgeEllipseContainer &ellipseContainer,
                           std::vector<TPDistances> &localParetoOptimalTps,
                           CostCalculator &localCalc,
                           AssignmentWithTransfer &localBestAssignment,
                           RequestCost &localBestCost,
                           NumAsgnStats &localNumAsgnStats) {

            for (const auto &dropoffEntry: relORDDropoffs.relevantSpotsFor(dVehId)) {
                if (dropoffEntry.stopIndex == routeState.numStopsOf(dVehId) - 1)
                    continue;

                const auto &transfersToThisDropoffDistances = transfersToDropoffsDistances.getDistancesFor(
                        dropoffEntry.pdId);
                const int minTransferToThisDropoffDistance =
                        transfersToDropoffsDistances.getMinDistanceFor(dropoffEntry.pdId);

                tryTransfers(pVehId, pickupEntry, dVehId, dropoffEntry,
                             thisLastStopToTransfersDistances, minThisLastStopToTransferDistance,
                             thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                             transfersToThisDropoffDistances, minTransferToThisDropoffDistance,
                             ellipseContainer, localParetoOptimalTps, postponedAssignments, localCalc,
                             localBestAssignment, localBestCost, localNumAsgnStats);
            }

        }

        void tryDropoffALS(const int pVehId, const RelevantPDLoc &pickupEntry, const int dVehId,
                           const RelevantDropoffsAfterLastStop &relALSDropoffs,
                           std::vector<AssignmentWithTransfer> &postponedAssignments,
                           const ConstantVectorRange<int> &thisLastStopToTransfersDistances,
                           const int minThisLastStopToTransferDistance,
                           const ConstantVectorRange<int> &thisPickupToTransfersDistances,
                           const int minThisPickupToTransferDistance,
                           const FlatRegular2DDistanceArray &transfersToDropoffsDistances,
                           const EdgeEllipseContainer &ellipseContainer,
                           std::vector<TPDistances> &localParetoOptimalTps,
                           CostCalculator &localCalc,
                           AssignmentWithTransfer &localBestAssignment,
                           RequestCost &localBestCost,
                           NumAsgnStats &localNumAsgnStats) {

            const auto numStopsDVeh = routeState.numStopsOf(dVehId);
            for (const auto &dropoffEntry: relALSDropoffs.relevantSpotsFor(dVehId)) {
                const auto &transfersToThisDropoffDistances = transfersToDropoffsDistances.getDistancesFor(
                        dropoffEntry.dropoffId);
                const int minTransferToThisDropoffDistance =
                        transfersToDropoffsDistances.getMinDistanceFor(dropoffEntry.dropoffId);
                const auto dropoffEntryExtended = RelevantPDLoc(numStopsDVeh - 1, dropoffEntry.dropoffId,
                                                                dropoffEntry.distToDropoff, 0);
                tryTransfers(pVehId, pickupEntry, dVehId, dropoffEntryExtended,
                             thisLastStopToTransfersDistances, minThisLastStopToTransferDistance,
                             thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                             transfersToThisDropoffDistances, minTransferToThisDropoffDistance,
                             ellipseContainer, localParetoOptimalTps, postponedAssignments, localCalc,
                             localBestAssignment, localBestCost, localNumAsgnStats);
            }
        }

        void tryDropoffORDForPickupALS(const int pVehId, const PDLoc &pickup, const int distanceToPickup,
                                       const int dVehId,
                                       const RelevantPDLocs &relORDDropoffs,
                                       const ConstantVectorRange<int> &thisLastStopToTransfersDistances,
                                       const int minThisLastStopToTransferDistance,
                                       const ConstantVectorRange<int> &thisPickupToTransfersDistances,
                                       const int minThisPickupToTransferDistance,
                                       const FlatRegular2DDistanceArray &transfersToDropoffsDistances,
                                       const EdgeEllipseContainer &ellipseContainer,
                                       std::vector<TPDistances> &localParetoOptimalTps,
                                       CostCalculator &localCalc,
                                       AssignmentWithTransfer &localBestAssignment,
                                       RequestCost &localBestCost,
                                       NumAsgnStats &localNumAsgnStats) {

            if (distanceToPickup >= INFTY)
                return;

            const auto numStopsPVeh = routeState.numStopsOf(pVehId);
            const RelevantPDLoc pickupEntry(numStopsPVeh - 1, pickup.id, distanceToPickup, 0);
            static std::vector<AssignmentWithTransfer> placeholder;
            KASSERT(placeholder.empty());
            const auto numStopsDVeh = routeState.numStopsOf(dVehId);

            // Calculate the distances from the pickup to the stops of the dropoff vehicle
            for (const auto &dropoffEntry: relORDDropoffs.relevantSpotsFor(dVehId)) {
                // Try all possible transfer points
                if (dropoffEntry.stopIndex == numStopsDVeh - 1)
                    continue;

                const auto &transfersToThisDropoffDistances = transfersToDropoffsDistances.getDistancesFor(
                        dropoffEntry.pdId);
                const int minTransferToThisDropoffDistance =
                        transfersToDropoffsDistances.getMinDistanceFor(dropoffEntry.pdId);

                tryTransfers(pVehId, pickupEntry, dVehId, dropoffEntry,
                             thisLastStopToTransfersDistances, minThisLastStopToTransferDistance,
                             thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                             transfersToThisDropoffDistances, minTransferToThisDropoffDistance,
                             ellipseContainer, localParetoOptimalTps, placeholder, localCalc,
                             localBestAssignment, localBestCost, localNumAsgnStats);
            }
        }

        void tryDropoffALSForPickupALS(const int pVehId, const PDLoc &pickup, const int distanceToPickup,
                                       const int dVehId,
                                       const ConstantVectorRange<int> &thisLastStopToTransfersDistances,
                                       const int minThisLastStopToTransferDistance,
                                       const ConstantVectorRange<int> &thisPickupToTransfersDistances,
                                       const int minThisPickupToTransferDistance,
                                       const FlatRegular2DDistanceArray &transfersToDropoffsDistances,
                                       const RelevantDropoffsAfterLastStop &relALSDropoffs,
                                       const EdgeEllipseContainer &ellipseContainer,
                                       std::vector<TPDistances> &localParetoOptimalTps,
                                       CostCalculator &localCalc,
                                       AssignmentWithTransfer &localBestAssignment,
                                       RequestCost &localBestCost,
                                       NumAsgnStats &localNumAsgnStats) {
            KASSERT(distanceToPickup < INFTY);

            const auto numStopsPVeh = routeState.numStopsOf(pVehId);
            const RelevantPDLoc pickupEntry(numStopsPVeh - 1, pickup.id, distanceToPickup, 0);
            static std::vector<AssignmentWithTransfer> placeholder;
            KASSERT(placeholder.empty());

            const auto numStopsDVeh = routeState.numStopsOf(dVehId);

            // Calculate the distances from the pickup to the stops of the dropoff vehicle
            for (const auto dropoffEntry: relALSDropoffs.relevantSpotsFor(dVehId)) {

                const auto &transfersToThisDropoffDistances = transfersToDropoffsDistances.getDistancesFor(
                        dropoffEntry.dropoffId);
                const int minTransferToThisDropoffDistance =
                        transfersToDropoffsDistances.getMinDistanceFor(dropoffEntry.dropoffId);
                const auto dropoffEntryExtended = RelevantPDLoc(numStopsDVeh - 1, dropoffEntry.dropoffId,
                                                                dropoffEntry.distToDropoff, 0);
                tryTransfers(pVehId, pickupEntry, dVehId, dropoffEntryExtended,
                             thisLastStopToTransfersDistances, minThisLastStopToTransferDistance,
                             thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                             transfersToThisDropoffDistances, minTransferToThisDropoffDistance,
                             ellipseContainer, localParetoOptimalTps, placeholder, localCalc,
                             localBestAssignment, localBestCost, localNumAsgnStats);
            }

        }

        void tryTransfers(const int pVehId,
                          const RelevantPDLoc &pickupEntry,
                          const int dVehId,
                          const RelevantPDLoc &dropoffEntry,
                          const ConstantVectorRange<int> &pVehLastStopToTransfersDistances,
                          const int minPVehLastStopToTransferDistance,
                          const ConstantVectorRange<int> &thisPickupToTransfersDistances,
                          const int minThisPickupToTransferDistance,
                          const ConstantVectorRange<int> &transfersToThisDropoffDistances,
                          const int minTransferToThisDropoffDistance,
                          const EdgeEllipseContainer &ellipseContainer,
                          std::vector<TPDistances> &localParetoOptimalTps,
                          std::vector<AssignmentWithTransfer> &localPostponedAssignments,
                          CostCalculator &localCalc,
                          AssignmentWithTransfer &localBestAssignment,
                          RequestCost &localBestCost,
                          NumAsgnStats &localNumAsgnStats) {
            const auto *pVeh = &fleet[pVehId];
            const auto numStopsPVeh = routeState.numStopsOf(pVehId);
            const auto schedArrTimesPVeh = routeState.schedArrTimesFor(pVehId);
            const auto schedDepTimesPVeh = routeState.schedDepTimesFor(pVehId);
            const auto *dVeh = &fleet[dVehId];
            const auto numStopsDVeh = routeState.numStopsOf(dVehId);
            const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
            const auto schedDepTimesDVeh = routeState.schedDepTimesFor(dVehId);
            const auto schedArrTimesDVeh = routeState.schedArrTimesFor(dVehId);
            const auto occupanciesDVeh = routeState.occupanciesFor(dVehId);

            const auto &pickup = requestState.pickups[pickupEntry.pdId];
            const auto &dropoff = requestState.dropoffs[dropoffEntry.pdId];

            int distanceToPickup = pickupEntry.distToPDLoc;
            bool bnsLowerBoundUsed = false;

            if (pickupEntry.stopIndex == 0 && pickupEntry.stopIndex < numStopsPVeh - 1) {
                bnsLowerBoundUsed = !searches.knowsDistance(pVeh->vehicleId, pickupEntry.pdId);
                distanceToPickup = bnsLowerBoundUsed ? pickupEntry.distToPDLoc : searches.getDistance(pVeh->vehicleId,
                                                                                                      pickupEntry.pdId);
            }

            using namespace time_utils;
            // Compute part of cost lower bound that depends only on the pickup
            const int depTimeAtPickup = getActualDepTimeAtPickup(pVehId, pickupEntry.stopIndex, distanceToPickup,
                                                                 pickup, requestState, routeState);
            const int minInitialPickupDetour =
                    calcInitialPickupDetour(pVehId, pickupEntry.stopIndex, numStopsPVeh - 1, depTimeAtPickup,
                                            pickupEntry.distFromPDLocToNextStop, requestState, routeState);
            const auto pVehVehWaitTimeFromPickupToEndOfRoute =
                    getTotalVehWaitTimeInInterval(pVehId, pickupEntry.stopIndex, numStopsPVeh - 1, routeState);
            const auto minResPickupDetour =
                    std::max(minInitialPickupDetour - pVehVehWaitTimeFromPickupToEndOfRoute, 0) +
                    (pickupEntry.stopIndex == numStopsPVeh - 1 ? minThisPickupToTransferDistance
                                                               : minPVehLastStopToTransferDistance);
            const int minTripTimeUntilTransfer =
                    pickupEntry.stopIndex == numStopsPVeh - 1 ? minThisPickupToTransferDistance :
                    pickupEntry.distFromPDLocToNextStop + minPVehLastStopToTransferDistance +
                    (pickupEntry.stopIndex + 1 < numStopsPVeh - 1 ? schedArrTimesPVeh[numStopsPVeh - 1] -
                                                                    schedDepTimesPVeh[pickupEntry.stopIndex + 1] : 0);

            // Compute part of cost lower bound that depends only on the dropoff
            const bool dropoffAtExistingStop = isDropoffAtExistingStop(dVehId, INVALID_INDEX,
                                                                       dropoffEntry.stopIndex,
                                                                       dropoff.loc, routeState);
            const int minInitialDropoffDetour =
                    calcInitialDropoffDetour(dVehId, dropoffEntry.stopIndex, dropoffEntry.distToPDLoc,
                                             dropoffEntry.distFromPDLocToNextStop,
                                             dropoffAtExistingStop, routeState);
            const int minDropoffCostWithoutTrip =
                    calc.calcMinKnownDropoffSideCostWithoutTripTime(*dVeh, dropoffEntry.stopIndex,
                                                                    minInitialDropoffDetour,
                                                                    dropoff.walkingDist, requestState);

            for (int i = dropoffEntry.stopIndex; i > 0; i--) {
                if (i >= numStopsDVeh - 1)
                    continue;

                // If occupancy exceeds capacity at this leg, we do not have to consider this or earlier stops
                // of the route, as this leg needs to be traversed.
                if (occupanciesDVeh[i] + requestState.originalRequest.numRiders > dVeh->capacity)
                    break;

                // Compute lower bound for costs with transfer between i and i + 1, considering trip time
                // from stop i + 1 to the dropoff and detour to the dropoff. This lower bound also holds for
                // any stop before i so if it is worse than the best known cost, we can stop for this vehicle.
                const auto minTripTimeTransferToDropoffViaStops =
                        i + 1 >= dropoffEntry.stopIndex ? 0 : dropoffEntry.distToPDLoc +
                                                              schedArrTimesDVeh[dropoffEntry.stopIndex] -
                                                              schedDepTimesDVeh[i + 1];
                const int minTripTimeFromTransfer =
                        std::max(minTransferToThisDropoffDistance, minTripTimeTransferToDropoffViaStops +
                                                                   dropoffEntry.distToPDLoc) + dropoff.walkingDist;
                if constexpr (UseCostLowerBounds) {
                    using F = CostCalculator::CostFunction;
                    const int minCostFromHere =
                            F::calcTripCost(minTripTimeUntilTransfer + minTripTimeFromTransfer, requestState) +
                            F::calcVehicleCost(minResPickupDetour) + minDropoffCostWithoutTrip;
                    if (minCostFromHere > std::min(localBestCost.total, requestState.getBestCost()))
                        break;
                }

                const int stopId = stopIdsDVeh[i];
                const auto &transferPoints = ellipseContainer.getEdgesInEllipse(stopId);

                localParetoOptimalTps.clear();

                // Loop over all possible transfer points
                for (auto edge: transferPoints) {
                    // Build the transfer point
                    const int transferLoc = edge.edge;
                    KASSERT(isEdgeRel.isSet(transferLoc));

                    // If the pickup or dropoff coincides with the transfer, we skip it
                    if (pickup.loc == transferLoc || transferLoc == dropoff.loc)
                        continue;

                    const auto edgeOffset = inputGraph.travelTime(transferLoc);
                    const int distPickupToTransfer = thisPickupToTransfersDistances[relEdgesToInternalIdx[transferLoc]];
                    const int distTransferToDropoff = transfersToThisDropoffDistances[relEdgesToInternalIdx[transferLoc]];

                    const int distPVehToTransfer = pickupEntry.stopIndex == numStopsPVeh - 1 ? distPickupToTransfer
                                                                                             : pVehLastStopToTransfersDistances[relEdgesToInternalIdx[transferLoc]];

                    if constexpr (DoTransferPointParetoChecks) {
                        if (i > 0) { // Cannot pareto check for transfer BNS because we only know lower bounds
                            const bool transferAtStopDVeh = transferLoc == stopLocationsDVeh[i];
                            const int distToTransferDVeh = transferAtStopDVeh ? 0 : edge.distToTail + edgeOffset;
                            const int distNextLegAfterTransferDVeh = i == dropoffEntry.stopIndex ?
                                                                     transfersToThisDropoffDistances[relEdgesToInternalIdx[transferLoc]]
                                                                                                 : edge.distFromHead;
                            const bool notDominated = checkPareto(
                                    TPDistances(distPVehToTransfer, distToTransferDVeh,
                                                distNextLegAfterTransferDVeh), localParetoOptimalTps);
                            if (!notDominated)
                                continue;
                        }
                    }

                    TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh - 1, i,
                                                     distPVehToTransfer, 0, edge.distToTail + edgeOffset,
                                                     edge.distFromHead);

                    ++localNumAsgnStats.numTransferPoints;

                    // Build the assignment
                    AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                    asgn.pickupIdx = pickupEntry.stopIndex;
                    asgn.dropoffIdx = dropoffEntry.stopIndex;
                    asgn.transferIdxPVeh = numStopsPVeh - 1;
                    asgn.transferIdxDVeh = i;

                    asgn.pickupBNSLowerBoundUsed = bnsLowerBoundUsed;

                    asgn.pickup = &pickup;
                    asgn.dropoff = &dropoff;

                    KASSERT(asgn.transferIdxDVeh < numStopsDVeh - 1 ||
                            (transferLoc == stopLocationsDVeh[numStopsDVeh - 1]));

                    asgn.distToPickup = distanceToPickup;
                    asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;
                    asgn.distToDropoff = dropoffEntry.distToPDLoc;
                    asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;

                    asgn.pickupType =
                            pickupEntry.stopIndex == numStopsPVeh - 1 ? AFTER_LAST_STOP : pickupEntry.stopIndex == 0
                                                                                          ? BEFORE_NEXT_STOP : ORDINARY;
                    asgn.dropoffType = dropoffEntry.stopIndex == numStopsDVeh - 1 ? AFTER_LAST_STOP : ORDINARY;
                    asgn.transferTypePVeh = AFTER_LAST_STOP;
                    asgn.transferTypeDVeh = ORDINARY;

                    const int alsDistancePVeh =
                            pickupEntry.stopIndex == numStopsPVeh - 1 ? distanceToPickup : distPVehToTransfer;
                    finishDistances(asgn, distPickupToTransfer, alsDistancePVeh,
                                    distTransferToDropoff, dropoffEntry.distToPDLoc);

                    // Try the assignment with ORD dropoff
                    tryPotentiallyUnfinishedAssignment(asgn, localPostponedAssignments, localCalc, localBestAssignment,
                                                       localBestCost, localNumAsgnStats);
                }
            }
        }


        void finishDistances(AssignmentWithTransfer &asgn, const int pairedDistancePVeh, const int alsDistancePVeh,
                             const int pairedDistanceDVeh, const int alsDistanceDVeh) {
            const int pickupIdx = asgn.pickupIdx;
            const int transferIdxPVeh = asgn.transferIdxPVeh;
            const int transferIdxDVeh = asgn.transferIdxDVeh;
            const int dropoffIdx = asgn.dropoffIdx;

            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);

            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);

            const auto schedDepTimesPVeh = routeState.schedDepTimesFor(asgn.pVeh->vehicleId);
            const auto schedArrTimesPVeh = routeState.schedArrTimesFor(asgn.pVeh->vehicleId);
            const auto schedDepTimesDVeh = routeState.schedDepTimesFor(asgn.dVeh->vehicleId);
            const auto schedArrTimesDVeh = routeState.schedArrTimesFor(asgn.dVeh->vehicleId);

            const int legPickup =
                    pickupIdx < numStopsPVeh - 1 ? schedArrTimesPVeh[pickupIdx + 1] - schedDepTimesPVeh[pickupIdx] : 0;
            const int legTransferPVeh = transferIdxPVeh < numStopsPVeh - 1 ? schedArrTimesPVeh[transferIdxPVeh + 1] -
                                                                             schedDepTimesPVeh[transferIdxPVeh] : 0;
            const int legTransferDVeh = transferIdxDVeh < numStopsDVeh - 1 ? schedArrTimesDVeh[transferIdxDVeh + 1] -
                                                                             schedDepTimesDVeh[transferIdxDVeh] : 0;
            const int legDropoff =
                    dropoffIdx < numStopsDVeh - 1 ? schedArrTimesDVeh[dropoffIdx + 1] - schedDepTimesDVeh[dropoffIdx]
                                                  : 0;

            const bool pickupAtStop = asgn.pickup->loc == stopLocationsPVeh[pickupIdx];
            const bool transferAtStopPVeh =
                    asgn.transfer.loc == stopLocationsPVeh[transferIdxPVeh] && asgn.transferIdxPVeh > asgn.pickupIdx;
            const bool transferAtStopDVeh = asgn.transfer.loc == stopLocationsDVeh[transferIdxDVeh];
            const bool dropoffAtStop =
                    asgn.dropoff->loc == stopLocationsDVeh[dropoffIdx] && asgn.dropoffIdx > asgn.transferIdxDVeh;

            const bool pairedPVeh = pickupIdx == transferIdxPVeh;
            const bool pairedDVeh = transferIdxDVeh == dropoffIdx;

            const bool pickupAfterLastStop = pickupIdx == numStopsPVeh - 1;
            const bool transferAfterLastStopPVeh = transferIdxPVeh == numStopsPVeh - 1;
            const bool transferAfterLastStopDVeh = transferIdxDVeh == numStopsDVeh - 1;
            const bool dropoffAfterLastStop = dropoffIdx == numStopsDVeh - 1;

            //* Pickup distances
            // Distance to pickup
            if (pickupAtStop)
                asgn.distToPickup = 0;

            if (!pickupAtStop && pickupAfterLastStop)
                asgn.distToPickup = alsDistancePVeh;

            // Distance from pickup
            KASSERT(!pickupAfterLastStop || pairedPVeh);
            if (pairedPVeh)
                asgn.distFromPickup = 0;

            if (!pairedPVeh && pickupAtStop)
                asgn.distFromPickup = legPickup;

            // Distance to transfer pVeh
            if (pairedPVeh)
                asgn.distToTransferPVeh = pairedDistancePVeh;

            if (!pickupAfterLastStop && transferAfterLastStopPVeh)
                asgn.distToTransferPVeh = alsDistancePVeh;

            if (transferAtStopPVeh)
                asgn.distToTransferPVeh = 0;

            // Distance from transfer pVeh
            if (transferAtStopPVeh)
                asgn.distFromTransferPVeh = legTransferPVeh;

            if (transferAfterLastStopPVeh)
                asgn.distFromTransferPVeh = 0;

            //* Dropoff Distances
            // Distance to transfer dVeh
            if (transferAtStopDVeh)
                asgn.distToTransferDVeh = 0;

            if (!transferAtStopDVeh && transferAfterLastStopDVeh)
                asgn.distToTransferDVeh = alsDistanceDVeh;

            // Distance from transfer dVeh
            if (pairedDVeh || transferAfterLastStopDVeh)
                asgn.distFromTransferDVeh = 0;

            if (!pairedDVeh && transferAtStopDVeh) {
                KASSERT(legTransferDVeh > 0);
                asgn.distFromTransferDVeh = legTransferDVeh;
            }

            // Distance to dropoff
            if (pairedDVeh && !dropoffAfterLastStop)
                asgn.distToDropoff = pairedDistanceDVeh;

            if ((!transferAfterLastStopDVeh && dropoffAfterLastStop) ||
                (transferAfterLastStopDVeh && transferAtStopDVeh))
                asgn.distToDropoff = alsDistanceDVeh;

            if (dropoffAtStop)
                asgn.distToDropoff = 0;

            // Distance from dropoff
            if (dropoffAtStop)
                asgn.distFromDropoff = legDropoff;

            if (dropoffAfterLastStop)
                asgn.distFromDropoff = 0;

            // These assertions do not work due to edges with travel time 0
//            KASSERT(asgn.distFromDropoff > 0 || dropoffAfterLastStop);
//            KASSERT(asgn.distFromTransferPVeh > 0 || transferAfterLastStopPVeh);
//            KASSERT(asgn.distFromPickup > 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
//            KASSERT(asgn.distFromTransferDVeh > 0 || asgn.transferIdxDVeh == asgn.dropoffIdx);
//            KASSERT(asgn.distToPickup > 0 || asgn.distFromPickup > 0 || asgn.distToTransferPVeh > 0 ||
//                   asgn.distFromTransferPVeh > 0);
//            KASSERT(asgn.distToTransferDVeh > 0 || asgn.distFromTransferDVeh > 0 || asgn.distToDropoff > 0 ||
//                   asgn.distFromDropoff > 0);
        }

        // Skip unecessary assignments (e.g. if the pickup or dropoff is already at the next stop)
        bool canSkipAssignment(const AssignmentWithTransfer &asgn) const {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);
            return ((asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
                    || (asgn.transferIdxDVeh < numStopsDVeh - 1 &&
                        asgn.transfer.loc == stopLocationsDVeh[asgn.transferIdxDVeh + 1])
                    || (asgn.dropoffIdx < numStopsDVeh - 1 &&
                        asgn.dropoff->loc == stopLocationsDVeh[asgn.dropoffIdx + 1]));
        }

        void trackAssignmentTypeStatistic(const AssignmentWithTransfer &asgn,
                                          NumAsgnStats &stats) {
            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    stats.numAssignmentsTriedPickupBNS++;
                    break;

                case ORDINARY:
                    stats.numAssignmentsTriedPickupORD++;
                    break;

                case AFTER_LAST_STOP:
                    stats.numAssignmentsTriedPickupALS++;
                    break;
                default:
                    KASSERT(false);
            }

            switch (asgn.dropoffType) {
                case ORDINARY:
                    stats.numAssignmentsTriedDropoffORD++;
                    break;
                case AFTER_LAST_STOP:
                    stats.numAssignmentsTriedDropoffALS++;
                    break;
                default:
                    KASSERT(false);
            }
        }

        void tryFinishedAssignment(AssignmentWithTransfer &asgn,
                                   CostCalculator &localCalc,
                                   AssignmentWithTransfer &localBestAssignment,
                                   RequestCost &localBestCost,
                                   NumAsgnStats &localNumAsgnStats) {
            KASSERT(asgn.isFinished());
            if (canSkipAssignment(asgn))
                return;
            trackAssignmentTypeStatistic(asgn, localNumAsgnStats);
            const auto cost = localCalc.calc(asgn, requestState);
            if (cost.total < std::min(localBestCost.total, requestState.getBestCost())) {
                localBestAssignment = asgn;
                localBestCost = cost;
            }
        }

        void tryPotentiallyUnfinishedAssignment(AssignmentWithTransfer &asgn,
                                                std::vector<AssignmentWithTransfer> &postponedAssignments,
                                                CostCalculator &localCalc,
                                                AssignmentWithTransfer &localBestAssignment,
                                                RequestCost &localBestCost,
                                                NumAsgnStats &localNumAsgnStats) {

            if (canSkipAssignment(asgn))
                return;

            if (!asgn.isFinished()) {
//                const auto lowerBound = calc.calcLowerBound(asgn, requestState);
                const auto lowerBound = localCalc.calc(asgn, requestState);
                if (lowerBound.total >= std::min(localBestCost.total, requestState.getBestCost()))
                    return;

                postponedAssignments.push_back(asgn);
            } else {
                tryFinishedAssignment(asgn, localCalc, localBestAssignment, localBestCost, localNumAsgnStats);
            }
        }

        void finishPostponedPBNSAssignments(std::vector<AssignmentWithTransfer> &postponedAssignments,
                                            NumAsgnStats &globalNumAsgnStats) {

            // Group postponed assignments by vehicle ID and finish them
            std::sort(postponedAssignments.begin(), postponedAssignments.end(), [&](const AssignmentWithTransfer &a,
                                                                                    const AssignmentWithTransfer &b) {
                return a.pVeh->vehicleId < b.pVeh->vehicleId;
            });

            auto startOfCurrentVeh = postponedAssignments.begin();
            for (auto it = postponedAssignments.begin(); it != postponedAssignments.end(); ++it) {
                if (it->pVeh->vehicleId != startOfCurrentVeh->pVeh->vehicleId) {
                    finishAssignmentsWithPickupBNSLowerBound(startOfCurrentVeh->pVeh,
                                                             IteratorRange(startOfCurrentVeh, it), globalNumAsgnStats);
                    startOfCurrentVeh = it;
                }
            }
            if (startOfCurrentVeh != postponedAssignments.end()) {
                // Finish the last group of assignments
                finishAssignmentsWithPickupBNSLowerBound(startOfCurrentVeh->pVeh,
                                                         IteratorRange(startOfCurrentVeh, postponedAssignments.end()),
                                                         globalNumAsgnStats);
            }
        }


        void finishAssignmentsWithPickupBNSLowerBound(const Vehicle *pVeh,
                                                      auto &&postponedAssignmentsForPVeh,
                                                      NumAsgnStats &globalNumAsgnStats) {

            if (postponedAssignmentsForPVeh.empty())
                return;

            for (const auto &asgn: postponedAssignmentsForPVeh) {
                KASSERT(asgn.pickupBNSLowerBoundUsed && !asgn.dropoffPairedLowerBoundUsed);
                searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
            }

            searches.computeExactDistancesVia(*pVeh);

            for (auto &asgn: postponedAssignmentsForPVeh) {
                KASSERT(searches.knowsCurrentLocationOf(pVeh->vehicleId));
                KASSERT(searches.knowsDistance(pVeh->vehicleId, asgn.pickup->id));

                const int distance = searches.getDistance(pVeh->vehicleId, asgn.pickup->id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;

                KASSERT(asgn.isFinished());
                tryFinishedAssignment(asgn, calc, globalBestAssignment, globalBestCost, globalNumAsgnStats);
            }
        }

        const InputGraphT &inputGraph;

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const CH &vehCh;

        TransferALSStrategyT &strategy;
        TransfersPickupALSStrategyT &pickupALSStrategy;

        CurVehLocToPickupSearchesT &searches;

        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        CostCalculator &calc;
        InsertionAsserterT &asserter;

        std::vector<int> allTransferEdges;

        TimestampedVector<int> relPVehToInternalIdx; // Maps vehicle IDs of relevant pVehs to consecutive indices
        FastResetFlagArray<> dVehStopsFlags; // Helper to deduplicate stops of dropoff vehicles
        FastResetFlagArray<> isEdgeRel; // Helper structure to deduplicate transfer edges
        std::vector<int> relEdgesToInternalIdx; // Maps transfer edges to consecutive indices

        struct ThreadLocalData {
            AssignmentWithTransfer bestAsgn = AssignmentWithTransfer();
            RequestCost bestCost = RequestCost::INFTY_COST();
            std::vector<TPDistances> paretoOptimalTps;
            std::vector<AssignmentWithTransfer> postponedAssignments;
            NumAsgnStats numAsgnStats;
        };

        tbb::enumerable_thread_specific<ThreadLocalData> threadLocalData;
        tbb::enumerable_thread_specific<CostCalculator> threadLocalCalc;

        AssignmentWithTransfer globalBestAssignment;
        RequestCost globalBestCost;
    };
}
