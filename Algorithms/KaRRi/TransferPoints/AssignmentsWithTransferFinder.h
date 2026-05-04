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

#include "Algorithms/KaRRi/TransferPoints/OrdinaryTransfers/OrdinaryTransferFinder.h"

#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"

namespace karri {

    template<typename OrdinaryTransferFinderT,
            typename TransferALSPVehFinderT,
            typename TransferALSDVehFinderT,
            typename DropoffALSStrategyT,
            typename EllipseReconstructorT,
            typename InsertionAsserterT
            >
    class AssignmentsWithTransferFinder {

    public:
        AssignmentsWithTransferFinder(
                OrdinaryTransferFinderT &ordinaryTransfers,
                TransferALSPVehFinderT &transfersALSPVeh,
                TransferALSDVehFinderT &transfersALSDVeh,
                DropoffALSStrategyT &dropoffALSStrategy,
                EllipseReconstructorT &ellipseReconstructor,
                RequestState &requestState,
                const RouteState &routeState,
                InsertionAsserterT &asserter)
                : ordinaryTransfers(ordinaryTransfers),
                  transfersALSPVeh(transfersALSPVeh),
                  transfersALSDVeh(transfersALSDVeh),
                  dropoffALSStrategy(dropoffALSStrategy),
                  ellipseReconstructor(ellipseReconstructor),
                  requestState(requestState),
                  routeState(routeState),
                  asserter(asserter) {}

        void init() {
            ordinaryTransfers.init();
            transfersALSPVeh.init();
            transfersALSDVeh.init();
        }

        void findBestAssignment(const RelevantPDLocs &relORDPickups,
                                const RelevantPDLocs &relBNSPickups,
                                const RelevantPDLocs &relORDDropoffs,
                                const RelevantPDLocs &relBNSDropoffs,
                                const PDDistances &pdDistances) {
            // Method to find the best assignment with exactly one transfer, i. e. the best possible
            // single transfer journey for the given request

            const auto relALSDropoffs = dropoffALSStrategy.findDropoffsAfterLastStop();
            std::vector<int> pVehStopIds;
            std::vector<int> dVehStopIds;
            getPVehAndDVehStopIdsForOrdinaryTransfers(relORDPickups, relBNSPickups, relORDDropoffs, relBNSDropoffs,
                                                      relALSDropoffs, pVehStopIds, dVehStopIds);


            std::vector<int> allStopIds;
            allStopIds.reserve(pVehStopIds.size() + dVehStopIds.size());
            stopSeen.reset();
            for (const auto &stopId: pVehStopIds) {
                if (!stopSeen.isSet(stopId)) {
                    stopSeen.set(stopId);
                    allStopIds.push_back(stopId);
                }
            }
            for (const auto &stopId: dVehStopIds) {
                if (!stopSeen.isSet(stopId)) {
                    stopSeen.set(stopId);
                    allStopIds.push_back(stopId);
                }
            }
            const auto ellipseContainer = ellipseReconstructor.computeEllipses(allStopIds, requestState.stats().ellipseReconstructionStats);

            // * TRANSFER AFTER LAST STOP (PVeh)
            transfersALSPVeh.findAssignments(relORDPickups, relBNSPickups, relORDDropoffs, relALSDropoffs, pdDistances, ellipseContainer);

            // * ORDINARY TRANSFER
            ordinaryTransfers.findAssignments(pVehStopIds, dVehStopIds, relORDPickups, relBNSPickups, relORDDropoffs, relBNSDropoffs, relALSDropoffs, ellipseContainer);

            // * TRANSFER AFTER LAST STOP (DVeh)
            transfersALSDVeh.findAssignments(relORDPickups, relBNSPickups, relALSDropoffs, ellipseContainer);

            //* Test the best assignment found
            KASSERT(requestState.getBestCostWithTransfer() == INFTY || asserter.assertAssignment(requestState.getBestAssignmentWithTransfer()));
        }


    private:

        template<typename RelALSDropoffs>
        void getPVehAndDVehStopIdsForOrdinaryTransfers(const RelevantPDLocs &relORDPickups,
                                                       const RelevantPDLocs &relBNSPickups,
                                                       const RelevantPDLocs &relORDDropoffs,
                                                       const RelevantPDLocs &relBNSDropoffs,
                                                       const RelALSDropoffs &relALSDropoffs,
                                                       std::vector<int> &pVehStopIds,
                                                       std::vector<int> &dVehStopIds) {
            if (routeState.getMaxStopId() + 1 > stopSeen.size())
                stopSeen.resize(routeState.getMaxStopId() + 1);
            stopSeen.reset();

            for (const int pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                const int earliestRelevantStopIdx = 0;
                const auto stopIds = routeState.stopIdsFor(pVehId);
                for (int i = earliestRelevantStopIdx; i < routeState.numStopsOf(pVehId); ++i) {
                    if (!stopSeen.isSet(stopIds[i])) {
                        stopSeen.set(stopIds[i]);
                        pVehStopIds.push_back(stopIds[i]);
                    }
                }
            }

            for (const int pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                const int earliestRelevantStopIdx = relORDPickups.relevantSpotsFor(pVehId)[0].stopIndex;
                const auto stopIds = routeState.stopIdsFor(pVehId);
                for (int i = earliestRelevantStopIdx; i < routeState.numStopsOf(pVehId); ++i) {
                    if (!stopSeen.isSet(stopIds[i])) {
                        stopSeen.set(stopIds[i]);
                        pVehStopIds.push_back(stopIds[i]);
                    }
                }
            }

            stopSeen.reset();

            for (const int dVehId: relBNSDropoffs.getVehiclesWithRelevantPDLocs()) {
                const int latestRelevantStopIdx = 0;
                const auto stopIds = routeState.stopIdsFor(dVehId);
                for (int i = 0; i < latestRelevantStopIdx + 1; ++i) {
                    if (!stopSeen.isSet(stopIds[i])) {
                        stopSeen.set(stopIds[i]);
                        dVehStopIds.push_back(stopIds[i]);
                    }
                }
            }

            for (const int dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto numStops = routeState.numStopsOf(dVehId);
                const auto& rel = relORDDropoffs.relevantSpotsFor(dVehId);
                const int latestRelevantStopIdx = std::min(numStops - 1, rel[rel.size() - 1].stopIndex);
                const auto stopIds = routeState.stopIdsFor(dVehId);
                for (int i = 0; i < latestRelevantStopIdx + 1; ++i) {
                    if (!stopSeen.isSet(stopIds[i])) {
                        stopSeen.set(stopIds[i]);
                        dVehStopIds.push_back(stopIds[i]);
                    }
                }
            }

            for (const int dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                const int latestRelevantStopIdx = routeState.numStopsOf(dVehId) - 1;
                const auto stopIds = routeState.stopIdsFor(dVehId);
                for (int i = 0; i < latestRelevantStopIdx + 1; ++i) {
                    if (!stopSeen.isSet(stopIds[i])) {
                        stopSeen.set(stopIds[i]);
                        dVehStopIds.push_back(stopIds[i]);
                    }
                }
            }
        }

        OrdinaryTransferFinderT &ordinaryTransfers;
        TransferALSPVehFinderT &transfersALSPVeh;
        TransferALSDVehFinderT &transfersALSDVeh;
        DropoffALSStrategyT &dropoffALSStrategy;
        EllipseReconstructorT &ellipseReconstructor;

        

        RequestState &requestState;
        const RouteState &routeState;

        InsertionAsserterT &asserter;

        FastResetFlagArray<uint32_t> stopSeen;
    };
}