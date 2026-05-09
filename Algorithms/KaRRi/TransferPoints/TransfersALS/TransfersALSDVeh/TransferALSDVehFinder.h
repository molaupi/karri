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

#pragma once

namespace karri {
    template<typename InputGraphT,
        typename VehCHEnvT,
        typename TransferALSStrategyT,
        typename CurVehLocToPickupSearchesT,
        bool UseCostLowerBounds,
        bool DoTransferPointParetoChecks,
        typename InsertionAsserterT>
    class TransferALSDVehFinder {
        struct TPDistances {
            int detourPVeh = INFTY;
            int detourDVeh = INFTY;
            int trip = INFTY;

            static bool dominates(const TPDistances &lhs, const TPDistances &rhs) {
                return lhs.detourPVeh < rhs.detourPVeh &&
                       lhs.detourDVeh < rhs.detourDVeh &&
                       lhs.trip < rhs.trip;
            }
        };

        struct WorkUnit {
            int pVehId = INVALID_ID;
            int dVehIdxInRelAlsSet = INVALID_ID;
            INS_TYPES pVehType = NOT_SET;
        };

        struct NumAsgnStats {
            int64_t numTransferPoints = 0;
            int64_t numAssignmentsTriedPickupBNS = 0;
            int64_t numAssignmentsTriedPickupORD = 0;
            int64_t numAssignmentsTriedDropoffALS = 0;

            void reset() {
                numTransferPoints = 0;
                numAssignmentsTriedPickupBNS = 0;
                numAssignmentsTriedPickupORD = 0;
                numAssignmentsTriedDropoffALS = 0;
            }

            NumAsgnStats &operator+=(const NumAsgnStats &other) {
                numTransferPoints += other.numTransferPoints;
                numAssignmentsTriedPickupBNS += other.numAssignmentsTriedPickupBNS;
                numAssignmentsTriedPickupORD += other.numAssignmentsTriedPickupORD;
                numAssignmentsTriedDropoffALS += other.numAssignmentsTriedDropoffALS;
                return *this;
            }
        };

        // The dVeh drives the detour to the transfer point
        // This implies, that the dVeh drives from its last stop to a stop of the pVeh and picks up the customer
        // The dVeh then will drive to the dropoff and go into idle mode (dropoff ALS)
        // The pickup has to be BNS or ORD
    public:
        TransferALSDVehFinder(
            const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv,
            TransferALSStrategyT &strategy,
            CurVehLocToPickupSearchesT &searches,
            const Fleet &fleet,
            const RouteState &routeState,
            InsertionAsserterT &asserter
        ) : inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            vehChQuery(vehChEnv.template getFullCHQuery<>()),
            strategy(strategy),
            searches(searches),
            fleet(fleet),
            routeState(routeState),
            calc(routeState, fleet),
            pVehStopsFlags(fleet.size()),
            isEdgeRel(inputGraph.numEdges()),
            relEdgesToInternalIdx(inputGraph.numEdges()),
            asserter(asserter),
            threadLocalCalc([&] { return CostCalculator(routeState, fleet); }) {
        }

        template<typename EllipsesT>
        void findAssignments(const RelevantPDLocs &relORDPickups, const RelevantPDLocs &relBNSPickups,
                             const RelevantDropoffsAfterLastStop &relALSDropoffs, const EllipsesT &ellipseContainer,
                             RequestState &requestState,
                             const PDLocs &pdLocs,
                             stats::DispatchingPerformanceStats &stats) {
            Timer total;
            Timer innerTimer;

            if (relALSDropoffs.getVehiclesWithRelevantPDLocs().empty())
                return;

            if (relORDPickups.getVehiclesWithRelevantPDLocs().empty() &&
                relBNSPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            // Collect the relevant last stops
            std::vector<int> relevantLastStopLocs;
            for (const int dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                const int numStops = routeState.numStopsOf(dVehId);
                const int lastStopLoc = routeState.stopLocationsFor(dVehId)[numStops - 1];
                relevantLastStopLocs.push_back(lastStopLoc);
            }

            // Collect the relevant transfer locs
            if (pVehStopsFlags.size() <= routeState.getMaxStopId())
                pVehStopsFlags.resize(routeState.getMaxStopId() + 1);
            pVehStopsFlags.reset();
            allTransferEdges.clear();
            isEdgeRel.reset();
            for (const int pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                const int numStops = routeState.numStopsOf(pVehId);
                const auto stopIds = routeState.stopIdsFor(pVehId);
                const int earliestRelevantStopIdx = relORDPickups.relevantSpotsFor(pVehId)[0].stopIndex;
                for (int i = earliestRelevantStopIdx; i < numStops; ++i) {
                    const auto stopId = stopIds[i];
                    if (pVehStopsFlags.isSet(stopId))
                        continue;
                    pVehStopsFlags.set(stopId);
                    const auto &ellipse = ellipseContainer.getEdgesInEllipse(stopId);
                    for (const auto &e: ellipse) {
                        const auto loc = e.edge;
                        if (isEdgeRel.isSet(loc))
                            continue;
                        relEdgesToInternalIdx[loc] = static_cast<int>(allTransferEdges.size());
                        isEdgeRel.set(loc);
                        allTransferEdges.push_back(loc);
                    }
                }
            }

            for (const int pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                const int numStops = routeState.numStopsOf(pVehId);
                const auto stopIds = routeState.stopIdsFor(pVehId);

                for (int i = 0; i < numStops; i++) {
                    const auto stopId = stopIds[i];
                    if (pVehStopsFlags.isSet(stopId))
                        continue;
                    pVehStopsFlags.set(stopId);
                    const auto &ellipse = ellipseContainer.getEdgesInEllipse(stopId);
                    for (const auto &e: ellipse) {
                        const auto loc = e.edge;
                        if (isEdgeRel.isSet(loc))
                            continue;
                        relEdgesToInternalIdx[loc] = static_cast<int>(allTransferEdges.size());
                        isEdgeRel.set(loc);
                        allTransferEdges.push_back(loc);
                    }
                }
            }

            std::vector<int> pickupLocs;
            for (const auto &pickup: pdLocs.pickups) {
                pickupLocs.push_back(pickup.loc);
            }

            std::vector<int> dropoffLocs;
            for (const auto &dropoff: pdLocs.dropoffs) {
                dropoffLocs.push_back(dropoff.loc);
            }

            // Initialize the transfer strategy with the collected transfer edges.
            strategy.init(allTransferEdges);

            const auto initTime = innerTimer.elapsed<std::chrono::nanoseconds>();


            // lastStopToTransferDistances[i][j] stores the distance from the last stop of the i-th vehicle in
            // relALSDropoffs.getVehiclesWithRelevantPDLocs() to the j-th edge in transferEdges
            innerTimer.restart();
            const auto &lastStopToTransfersDistances = strategy.calculateDistancesFromLastStopToAllTransfers(
                relevantLastStopLocs, allTransferEdges);
            const auto searchTimeLastStopToTransfer = innerTimer.elapsed<std::chrono::nanoseconds>();

            // Calculate the distances from all pickups to the potential transfers
            innerTimer.restart();
            // pickupToTransfersDistances[i][j] stores the distance from i-th pickup to the j-th edge in transferEdges
            const auto &pickupsToTransfersDistances = strategy.calculateDistancesFromPickupsToAllTransfers(pickupLocs,
                allTransferEdges);
            const int64_t searchTimePickupToTransfer = innerTimer.elapsed<std::chrono::nanoseconds>();

            innerTimer.restart();
            // transferToDropoffDistances[i][j] stores the distance from i-th dropoff to the j-th edge in transferEdges
            const auto &transferToDropoffDistances = strategy.calculateDistancesFromAllTransfersToDropoffs(
                allTransferEdges, dropoffLocs);
            const auto searchTimeTransferToDropoff = innerTimer.elapsed<std::chrono::nanoseconds>();

            innerTimer.restart();

            const auto workUnits = composeWorkUnits(relORDPickups, relBNSPickups, relALSDropoffs);

            globalBestCost = RequestCost::INFTY_COST();
            globalBestAssignment = AssignmentWithTransfer();

            for (auto &local: threadLocalData) {
                local.bestAsgn = AssignmentWithTransfer();
                local.bestCost = RequestCost::INFTY_COST();
                local.paretoOptimalTps.clear();
                local.postponedAssignments.clear();
                local.numAsgnStats.reset();
            }

            // tbb::parallel_for(0ul, workUnits.size(), [&](const auto i) {
            for (size_t i = 0; i < workUnits.size(); ++i) {
                const auto &wu = workUnits[i];
                auto &local = threadLocalData.local();
                auto &localCalc = threadLocalCalc.local();
                processWorkUnit(wu, relORDPickups, relBNSPickups, relALSDropoffs, lastStopToTransfersDistances,
                                pickupsToTransfersDistances,
                                transferToDropoffDistances,
                                ellipseContainer, requestState, pdLocs, local.postponedAssignments, local.paretoOptimalTps, localCalc,
                                local.bestAsgn, local.bestCost, local.numAsgnStats);
            }

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
            finishPostponedPBNSAssignments(postponedPBNSAssignments, globalNumAsgnStats, requestState, pdLocs);
            const auto finishPbnsTime = finishPbnsTimer.elapsed<std::chrono::nanoseconds>();

            // Try best assignment found
            if (globalBestCost.total < INFTY)
                requestState.tryFinishedTransferAssignmentWithKnownCost(globalBestAssignment, globalBestCost);

            const auto tryAssignmentsTime = innerTimer.elapsed<std::chrono::nanoseconds>();

            KASSERT(globalBestCost.total >= INFTY || asserter.assertAssignment(globalBestAssignment));

            // Write the stats
            auto &alsDVehStats = stats.transferALSDVehStats;
            alsDVehStats.totalTime = total.elapsed<std::chrono::nanoseconds>();
            alsDVehStats.initTime = initTime;
            alsDVehStats.numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            alsDVehStats.numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            alsDVehStats.numCandidateVehiclesDropoffALS += relALSDropoffs.getVehiclesWithRelevantPDLocs().size();
            alsDVehStats.numPickups += pdLocs.numPickups();
            alsDVehStats.numDropoffs += pdLocs.numDropoffs();
            alsDVehStats.numAssignmentsTriedPickupBNS += globalNumAsgnStats.numAssignmentsTriedPickupBNS;
            alsDVehStats.numAssignmentsTriedPickupORD += globalNumAsgnStats.numAssignmentsTriedPickupORD;
            alsDVehStats.numAssignmentsTriedDropoffALS += globalNumAsgnStats.numAssignmentsTriedDropoffALS;
            alsDVehStats.tryAssignmentsTime += tryAssignmentsTime;
            alsDVehStats.tryPostponedAssignmentsTime += finishPbnsTime;
            alsDVehStats.numPostponedAssignments += numPostponedPbnsAssignments;
            alsDVehStats.numInputTransferPoints += allTransferEdges.size();
            alsDVehStats.numNonPrunedTransferPoints += globalNumAsgnStats.numTransferPoints;

            alsDVehStats.searchTimeLastStopToTransfer += searchTimeLastStopToTransfer;
            alsDVehStats.searchTimePickupToTransfer += searchTimePickupToTransfer;
            alsDVehStats.searchTimeTransferToDropoff += searchTimeTransferToDropoff;
        }

        void init() {
        }

    private:
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

        std::vector<WorkUnit> composeWorkUnits(const RelevantPDLocs &relORDPickups, const RelevantPDLocs &relBNSPickups,
                                               const RelevantDropoffsAfterLastStop &relALSDropoffs) {
            std::vector<WorkUnit> workUnits;

            const auto &dVehIds = relALSDropoffs.getVehiclesWithRelevantPDLocs();
            for (int dVehIdxInSet = 0; dVehIdxInSet < dVehIds.size(); ++dVehIdxInSet) {
                const auto dVehId = *(dVehIds.begin() + dVehIdxInSet);

                // Pickup BNS
                for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                    // pVeh an dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;
                    workUnits.emplace_back(pVehId, dVehIdxInSet, INS_TYPES::BEFORE_NEXT_STOP);
                }

                // Pickup ORD
                for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                    // pVeh an dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;
                    workUnits.emplace_back(pVehId, dVehIdxInSet, INS_TYPES::ORDINARY);
                }
            }

            return workUnits;
        }

        void processWorkUnit(const WorkUnit &wu,
                             const RelevantPDLocs &relORDPickups,
                             const RelevantPDLocs &relBNSPickups,
                             const RelevantDropoffsAfterLastStop &relALSDropoffs,
                             const FlatRegular2DDistanceArray &lastStopToTransfersDistances,
                             const FlatRegular2DDistanceArray &pickupsToTransfersDistances,
                             const FlatRegular2DDistanceArray &transferToDropoffDistances,
                             const EdgeEllipseContainer &ellipseContainer,
                                   const RequestState &requestState,
                             const PDLocs &pdLocs,
                             std::vector<AssignmentWithTransfer> &localPostponedPBNSAssignments,
                             std::vector<TPDistances> &localParetoOptimalTps,
                             CostCalculator &localCalc,
                             AssignmentWithTransfer &localBestAssignment,
                             RequestCost &localBestCost,
                             NumAsgnStats &localNumAsgnStats) {
            const auto &lastStopDVehToTransfersDistances = lastStopToTransfersDistances.getDistancesFor(
                wu.dVehIdxInRelAlsSet);
            const int minThisLastStopDVehToTransferDistance = lastStopToTransfersDistances.getMinDistanceFor(
                wu.dVehIdxInRelAlsSet);
            const auto dVehId = *(relALSDropoffs.getVehiclesWithRelevantPDLocs().begin() + wu.dVehIdxInRelAlsSet);
            KASSERT(dVehId != wu.pVehId);

            static std::vector<AssignmentWithTransfer> placeholderPostponedAssignments;
            KASSERT(placeholderPostponedAssignments.empty());

            auto &postponedAssignmentsToUse =
                    wu.pVehType == BEFORE_NEXT_STOP ? localPostponedPBNSAssignments : placeholderPostponedAssignments;
            const auto &pickupEntries = wu.pVehType == BEFORE_NEXT_STOP
                                            ? relBNSPickups.relevantSpotsFor(wu.pVehId)
                                            : relORDPickups.relevantSpotsFor(wu.pVehId);

            for (const auto &dropoff: pdLocs.dropoffs) {
                const auto &transfersToThisDropoffDistances = transferToDropoffDistances.getDistancesFor(dropoff.id);
                const int minTransferToThisDropoffDistance = transferToDropoffDistances.getMinDistanceFor(dropoff.id);

                for (const auto &pickupEntry: pickupEntries) {
                    const auto &thisPickupToTransfersDistances = pickupsToTransfersDistances.getDistancesFor(
                        pickupEntry.pdId);
                    const int minThisPickupToTransferDistance = pickupsToTransfersDistances.getMinDistanceFor(
                        pickupEntry.pdId);

                    tryTransfers(wu.pVehId, pickupEntry, dVehId, dropoff,
                                 lastStopDVehToTransfersDistances, minThisLastStopDVehToTransferDistance,
                                 thisPickupToTransfersDistances, minThisPickupToTransferDistance,
                                 transfersToThisDropoffDistances, minTransferToThisDropoffDistance,
                                 ellipseContainer, requestState, pdLocs, postponedAssignmentsToUse, localParetoOptimalTps, localCalc,
                                 localBestAssignment, localBestCost, localNumAsgnStats);
                }
            }
        }

        void finishPostponedPBNSAssignments(std::vector<AssignmentWithTransfer> &postponedAssignments,
                                            NumAsgnStats &globalNumAsgnStats, const RequestState &requestState, const PDLocs &pdLocs) {
            // Group postponed assignments by vehicle ID and finish them
            std::sort(postponedAssignments.begin(), postponedAssignments.end(), [&](const AssignmentWithTransfer &a,
                  const AssignmentWithTransfer &b) {
                          return a.pVeh->vehicleId < b.pVeh->vehicleId;
                      });

            auto startOfCurrentVeh = postponedAssignments.begin();
            for (auto it = postponedAssignments.begin(); it != postponedAssignments.end(); ++it) {
                if (it->pVeh->vehicleId != startOfCurrentVeh->pVeh->vehicleId) {
                    finishAssignmentsWithPickupBNSLowerBound(*startOfCurrentVeh->pVeh,
                                                             IteratorRange(startOfCurrentVeh, it), globalNumAsgnStats,
                                                             requestState, pdLocs);
                    startOfCurrentVeh = it;
                }
            }
            if (startOfCurrentVeh != postponedAssignments.end()) {
                // Finish the last group of assignments
                finishAssignmentsWithPickupBNSLowerBound(*startOfCurrentVeh->pVeh,
                                                         IteratorRange(startOfCurrentVeh, postponedAssignments.end()),
                                                         globalNumAsgnStats, requestState, pdLocs);
            }
        }

        using RelevantPDLoc = RelevantPDLocs::RelevantPDLoc;

        void tryTransfers(const int pVehId, const RelevantPDLoc &pickupEntry,
                          const int dVehId, const PDLoc &dropoff,
                          const ConstantVectorRange<int> &lastStopDVehToTransferDistances,
                          const int minThisLastStopDVehToTransferDistance,
                          const ConstantVectorRange<int> &thisPickupToTransferDistances,
                          const int minThisPickupToTransferDistance,
                          const ConstantVectorRange<int> &transfersToThisDropoffDistances,
                          const int minTransferToThisDropoffDistance,
                          const EdgeEllipseContainer &ellipseContainer,
                                   const RequestState &requestState,
                          const PDLocs &pdLocs,
                          std::vector<AssignmentWithTransfer> &postponedPBNSAssignments,
                          std::vector<TPDistances> &localParetoOptimalTps,
                          CostCalculator &localCalc,
                          AssignmentWithTransfer &localBestAssignment,
                          RequestCost &localBestCost,
                          NumAsgnStats &localNumAsgnStats) {
            static const int stopTime = InputConfig::getInstance().stopTime;

            const auto &pVeh = fleet[pVehId];
            const auto &dVeh = fleet[dVehId];
            const int numStopsPVeh = routeState.numStopsOf(pVehId);
            const auto stopIdsPVeh = routeState.stopIdsFor(pVehId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);
            const int numStopsDVeh = routeState.numStopsOf(dVehId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
            const auto &pickup = pdLocs.pickups[pickupEntry.pdId];
            const auto pickupBns = pickupEntry.stopIndex < numStopsPVeh - 1 && pickupEntry.stopIndex == 0;

            using namespace time_utils;
            const auto pVehVehWaitTimeFromPickupToEndOfRoute =
                    getTotalVehWaitTimeInInterval(pVehId, pickupEntry.stopIndex, numStopsPVeh - 1, routeState);
            const auto pickupLegLength =
                    calcLengthOfLegStartingAt(pickupEntry.stopIndex, pVehId, routeState);
            const auto minPickupInitialDetour =
                    pickupEntry.distToPDLoc + pickupEntry.distFromPDLocToNextStop - pickupLegLength;
            const auto minResPickupDetour = std::max(minPickupInitialDetour - pVehVehWaitTimeFromPickupToEndOfRoute, 0);

            const auto minResDVehDetour =
                    minThisLastStopDVehToTransferDistance + minTransferToThisDropoffDistance + stopTime;
            const int minTripTimeAfterTransfer = minTransferToThisDropoffDistance;

            for (int i = pickupEntry.stopIndex; i < numStopsPVeh; i++) {
                const int stopId = stopIdsPVeh[i];
                const auto &transferPoints = ellipseContainer.getEdgesInEllipse(stopId);

                using F = CostCalculator::CostFunction;
                const auto minTripTimeToStopI = i <= pickupEntry.stopIndex + 1
                                                    ? 0
                                                    : pickupEntry.distFromPDLocToNextStop +
                                                      routeState.schedArrTimesFor(pVehId)[i] -
                                                      routeState.schedDepTimesFor(pVehId)[pickupEntry.stopIndex + 1];

                // Compute a lower bound cost that only depends on information independent of the actual transfer
                // location. This lower bound monotonously increases with growing i, so once the lower bound exceeds the
                // best cost found so far, we can stop the search for this pickup entry.
                const auto minTripTimeForLowerBoundStopI = std::max(minTripTimeToStopI,
                                                                    minThisPickupToTransferDistance) +
                                                           minTripTimeAfterTransfer;
                if constexpr (UseCostLowerBounds) {
                    const auto minCostToStopI = F::calcVehicleCost(minResPickupDetour + minResDVehDetour) +
                                                F::calcTripCost(minTripTimeForLowerBoundStopI, requestState);
                    if (minCostToStopI > std::min(localBestCost.total, requestState.getBestCost()))
                        break;
                }

                const auto vehWaitTimeFromStopIToEndOfRoute =
                        getTotalVehWaitTimeInInterval(pVehId, i, numStopsPVeh - 1, routeState);


                const auto transferLegLength = calcLengthOfLegStartingAt(i, pVehId, routeState);
                localParetoOptimalTps.clear();
                for (auto edge: transferPoints) {
                    const int tpLoc = edge.edge;
                    KASSERT(isEdgeRel.isSet(tpLoc));

                    // If the pickup or dropoff coincides with the transfer, we skip the assignment
                    if (pickup.loc == tpLoc || tpLoc == dropoff.loc)
                        continue;

                    const int tpOffset = inputGraph.travelTime(tpLoc);

                    const bool transferAtLastStopDVeh = tpLoc == stopLocationsDVeh[numStopsDVeh - 1];
                    const bool transferAtStopPVeh = tpLoc == stopLocationsPVeh[i] && pickupEntry.stopIndex != i;
                    const auto paired = pickupEntry.stopIndex == i;
                    const int distToTransferDVeh = transferAtLastStopDVeh
                                                       ? 0
                                                       : lastStopDVehToTransferDistances[relEdgesToInternalIdx[tpLoc]];

                    const int distToTransferPVeh = paired
                                                       ? thisPickupToTransferDistances[relEdgesToInternalIdx[tpLoc]]
                                                       : (transferAtStopPVeh ? 0 : edge.distToTail + tpOffset);
                    const int distFromTransferPVeh = edge.distFromHead;
                    const int distNextLegAfterTransferDVeh = transfersToThisDropoffDistances[relEdgesToInternalIdx[
                        tpLoc]];
                    const int detourPVeh =
                            distToTransferPVeh + !transferAtStopPVeh * stopTime + distFromTransferPVeh;
                    const int detourDVeh = distToTransferDVeh + !transferAtLastStopDVeh * stopTime +
                                           distNextLegAfterTransferDVeh;

                    const auto minTripTimeForTp =
                            minTripTimeToStopI + distToTransferPVeh + distNextLegAfterTransferDVeh;
                    const auto minResDetourPVehForTp = std::max(
                        detourPVeh - transferLegLength - vehWaitTimeFromStopIToEndOfRoute, 0);

                    if constexpr (UseCostLowerBounds) {
                        const auto minCostForTp = F::calcVehicleCost(minResDetourPVehForTp + detourDVeh) +
                                                  F::calcTripCost(minTripTimeForTp, requestState);
                        if (minCostForTp > std::min(localBestCost.total, requestState.getBestCost()))
                            continue;
                    }


                    if constexpr (DoTransferPointParetoChecks) {
                        const int trip = distToTransferPVeh + distNextLegAfterTransferDVeh;
                        const bool notDominated = checkPareto(TPDistances(detourPVeh, detourDVeh, trip),
                                                              localParetoOptimalTps);
                        if (!notDominated)
                            continue;
                    }


                    // Construct the transfer point
                    TransferPoint tp = TransferPoint(tpLoc, &pVeh, &dVeh);
                    localNumAsgnStats.numTransferPoints++;

                    tp.distancePVehToTransfer = edge.distToTail + tpOffset;
                    tp.distancePVehFromTransfer = edge.distFromHead;

                    KASSERT(
                        distToTransferDVeh > 0 || stopLocationsDVeh[numStopsDVeh - 1] == tpLoc || inputGraph.travelTime(
                            tpLoc) == 0);
                    tp.distanceDVehToTransfer = distToTransferDVeh;
                    tp.distanceDVehFromTransfer = 0;

                    tp.stopIdxPVeh = i;
                    tp.stopIdxDVeh = numStopsDVeh - 1;

                    // Build the resulting assignment
                    AssignmentWithTransfer asgn = AssignmentWithTransfer(&pVeh, &dVeh, tp);
                    asgn.pickup = pickup;
                    asgn.dropoff = dropoff;

                    asgn.pickupIdx = pickupEntry.stopIndex;
                    asgn.dropoffIdx = numStopsDVeh - 1;
                    asgn.transferIdxPVeh = i;
                    asgn.transferIdxDVeh = numStopsDVeh - 1;

                    asgn.distToPickup = pickupEntry.distToPDLoc;
                    asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;
                    asgn.distToDropoff = transfersToThisDropoffDistances[relEdgesToInternalIdx[tpLoc]];
                    asgn.distFromDropoff = 0;

                    // Check transfer at stop pVeh
                    if (!paired && transferAtStopPVeh) {
                        asgn.distToTransferPVeh = 0;

                        const int nextLeg = asgn.transferIdxPVeh < numStopsPVeh - 1
                                                ? routeState.schedArrTimesFor(pVehId)[asgn.transferIdxPVeh +
                                                      1] -
                                                  routeState.schedDepTimesFor(pVehId)[asgn.transferIdxPVeh]
                                                : 0;
                        asgn.distFromTransferPVeh = nextLeg;
                    }

                    if (paired) {
                        asgn.distToTransferPVeh = thisPickupToTransferDistances[relEdgesToInternalIdx[tpLoc]];
                        asgn.distFromPickup = 0;
                    }

                    if (pickupBns) {
                        if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup.id)) {
                            asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup.id);
                        } else {
                            asgn.pickupBNSLowerBoundUsed = true;
                        }
                    }

                    asgn.distToTransferDVeh = distToTransferDVeh;
                    asgn.distFromTransferDVeh = 0;

                    asgn.pickupType = pickupBns ? BEFORE_NEXT_STOP : ORDINARY;
                    asgn.transferTypePVeh = i == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                    asgn.transferTypeDVeh = AFTER_LAST_STOP;
                    asgn.dropoffType = AFTER_LAST_STOP;

                    // Try the potentially unfinished assignment
                    tryPotentiallyUnfinishedAssignment(asgn, postponedPBNSAssignments, localCalc, localBestAssignment,
                                                       localBestCost, localNumAsgnStats, requestState);
                }
            }
        }

        // Skip unnecessary assignments (e.g. if the pickup or dropoff is already at the next stop)
        bool canSkipAssignment(const AssignmentWithTransfer &asgn) const {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            return (asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup.loc == stopLocationsPVeh[asgn.pickupIdx + 1])
                   || (asgn.transferIdxPVeh < numStopsPVeh - 1 &&
                       asgn.transfer.loc == stopLocationsPVeh[asgn.transferIdxPVeh + 1]);
        }

        void trackAssignmentTypeStatistic(const AssignmentWithTransfer &asgn, NumAsgnStats &numAsgnStats) {
            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    numAsgnStats.numAssignmentsTriedPickupBNS++;
                    break;

                case ORDINARY:
                    numAsgnStats.numAssignmentsTriedPickupORD++;
                    break;

                default:
                    KASSERT(false);
            }
            numAsgnStats.numAssignmentsTriedDropoffALS++;
        }

        void tryFinishedAssignment(AssignmentWithTransfer &asgn,
                                   CostCalculator &localCalc,
                                   AssignmentWithTransfer &localBestAssignment,
                                   RequestCost &localBestCost,
                                   NumAsgnStats &numAsgnStats,
                                   const RequestState &requestState) {
            KASSERT(asgn.isFinished());

            if (canSkipAssignment(asgn))
                return;

            trackAssignmentTypeStatistic(asgn, numAsgnStats);

            const auto cost = localCalc.calc(asgn, requestState);

            if (cost.total < localBestCost.total) {
                localBestCost = cost;
                localBestAssignment = asgn;
            }
        }

        void tryPotentiallyUnfinishedAssignment(AssignmentWithTransfer &asgn,
                                                std::vector<AssignmentWithTransfer> &postponedAssignments,
                                                CostCalculator &localCalc,
                                                AssignmentWithTransfer &localBestAssignment,
                                                RequestCost &localBestCost,
                                                NumAsgnStats &numAsgnStats,
                                   const RequestState &requestState) {
            if (canSkipAssignment(asgn))
                return;

            if (!asgn.isFinished()) {
                const auto cost = localCalc.calc(asgn, requestState);
                if (cost.total <= std::min(requestState.getBestCost(), localBestCost.total)) {
                    postponedAssignments.push_back(asgn);
                }
            } else {
                tryFinishedAssignment(asgn, localCalc, localBestAssignment, localBestCost, numAsgnStats, requestState);
            }
        }

        void finishAssignmentsWithPickupBNSLowerBound(const Vehicle &pVeh,
                                                      auto &&postponedAssignments,
                                                      NumAsgnStats &numAsgnStats,
                                   const RequestState &requestState,
                                                      const PDLocs &pdLocs) {
            for (const auto &asgn: postponedAssignments) {
                KASSERT(asgn.pickupBNSLowerBoundUsed && !asgn.pickupPairedLowerBoundUsed);
                searches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
            }

            searches.computeExactDistancesVia(pVeh, pdLocs);

            for (auto &asgn: postponedAssignments) {
                KASSERT(searches.knowsCurrentLocationOf(pVeh.vehicleId));
                KASSERT(searches.knowsDistance(pVeh.vehicleId, asgn.pickup.id));

                const int distance = searches.getDistance(pVeh.vehicleId, asgn.pickup.id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;

                tryFinishedAssignment(asgn, calc, globalBestAssignment, globalBestCost, numAsgnStats, requestState);
            }
        }

        const InputGraphT &inputGraph;

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const CH &vehCh;
        VehCHQuery vehChQuery;

        TransferALSStrategyT &strategy;

        CurVehLocToPickupSearchesT &searches;

        const Fleet &fleet;
        const RouteState &routeState;
        CostCalculator calc;

        std::vector<int> allTransferEdges;

        FastResetFlagArray<> pVehStopsFlags; // Helper structure to deduplicate stops of pickup vehicles

        FastResetFlagArray<> isEdgeRel; // Helper structure to deduplicate transfer edges
        std::vector<int> relEdgesToInternalIdx; // Maps transfer edges to consecutive indices

        InsertionAsserterT &asserter;

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
