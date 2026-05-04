/// ******************************************************************************
/// MIT License
///
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

#pragma once

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "VertexInEllipse.h"
#include "DataStructures/Containers/TimestampedVector.h"

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

namespace karri {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using bucket entries and a CH topological downward search.
    template<typename InputGraphT,
            bool UseCostLowerBounds,
            bool DoTransferParetoChecks,
            typename EllipseSizeLoggerT = NullLogger,
            typename EllipseIntersectionSizeLoggerT = NullLogger>
    class EdgeEllipseIntersector {

    public:

        EdgeEllipseIntersector(const InputGraphT &inputGraph,
                               const Fleet &fleet,
                               const RequestState &requestState,
                               const RouteState &routeState,
                               CostCalculator &calc)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  requestState(requestState),
                  routeState(routeState),
                  calc(calc),
                  threadLocalNumTransferPointsBuilt(0),
                  ellipsesSizeLogger(LogManager<EllipseSizeLoggerT>::getLogger("ellipses.csv",
                                                                               "size\n")),
                  ellipseIntersectionSizeLogger(
                          LogManager<EllipseIntersectionSizeLoggerT>::getLogger("ellipse-intersection.csv",
                                                                                "size\n")) {}

        // Given a set of stop IDs for pickup vehicles and dropoff vehicles and the previously computed ellipses,
        // this computes the transfer points between any pair of stops in the two sets by intersecting ellipses.
        // The given stop IDs should indicate the first stop in a pair of stops of the respective vehicle.
        void computeTransferPoints(const std::vector<int> &pVehStopIds, const std::vector<int> &dVehStopIds,
                                   const EdgeEllipseContainer &ellipseContainer,
                                   int64_t& numTransferPointsBuilt) {

            numTransferPointsBuilt = 0;
            if (pVehStopIds.empty() || dVehStopIds.empty())
                return;

            for (auto& local : threadLocalNumTransferPointsBuilt)
                local = 0;

            std::fill(idxOfStopPVeh.begin(), idxOfStopPVeh.end(), INVALID_INDEX);
            idxOfStopPVeh.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);
            std::fill(idxOfStopDVeh.begin(), idxOfStopDVeh.end(), INVALID_INDEX);
            idxOfStopDVeh.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);

            numStopsPVeh = 0;
            for (const auto &pVehStopId: pVehStopIds) {
                if (idxOfStopPVeh[pVehStopId] == INVALID_INDEX) {
                    idxOfStopPVeh[pVehStopId] = numStopsPVeh++;
                    ellipsesSizeLogger << ellipseContainer.getEdgesInEllipse(pVehStopId).size() << "\n";
                }
            }

            numStopsDVeh = 0;
            for (const auto &dVehStopId: dVehStopIds) {
                if (idxOfStopDVeh[dVehStopId] == INVALID_INDEX) {
                    idxOfStopDVeh[dVehStopId] = numStopsDVeh++;
                    ellipsesSizeLogger << ellipseContainer.getEdgesInEllipse(dVehStopId).size() << "\n";
                }
            }

            std::vector<int> allStopIds;
            allStopIds.insert(allStopIds.end(), pVehStopIds.begin(), pVehStopIds.end());
            allStopIds.insert(allStopIds.end(), dVehStopIds.begin(), dVehStopIds.end());

            // Compute pairwise intersections of ellipses
            KASSERT(numStopsPVeh >= 0 && numStopsDVeh >= 0);
            if (transferPoints.size() < numStopsPVeh * numStopsDVeh)
                transferPoints.resize(numStopsPVeh * numStopsDVeh);

            // Flatten for parallelization of work
            std::vector<std::pair<int, int>> stopIdPairs;
            stopIdPairs.reserve(numStopsPVeh * numStopsDVeh);
            for (const auto &stopIdPStop: pVehStopIds) {
                const auto vehIdPStop = routeState.vehicleIdOf(stopIdPStop);

                for (const auto &stopIdDStop: dVehStopIds) {
                    const auto vehIdDStop = routeState.vehicleIdOf(stopIdDStop);

                    if (vehIdPStop == vehIdDStop)
                        continue;

                    stopIdPairs.emplace_back(stopIdPStop, stopIdDStop);
                }
            }

            tbb::parallel_for(0ul, stopIdPairs.size(), [&](const auto i) {
                const auto &[stopIdPStop, stopIdDStop] = stopIdPairs[i];
                const auto vehIdPStop = routeState.vehicleIdOf(stopIdPStop);
                const auto internalIdxPStop = idxOfStopPVeh[stopIdPStop];
                const auto &ellipsePStop = ellipseContainer.getEdgesInEllipse(stopIdPStop);

                const auto vehIdDStop = routeState.vehicleIdOf(stopIdDStop);
                const auto internalIdxDStop = idxOfStopDVeh[stopIdDStop];
                const auto &ellipseDStop = ellipseContainer.getEdgesInEllipse(stopIdDStop);

                KASSERT(vehIdPStop != vehIdDStop);

                KASSERT(internalIdxPStop * numStopsDVeh + internalIdxDStop >= 0 && internalIdxPStop * numStopsDVeh + internalIdxDStop < transferPoints.size());
                auto &transferPointsForPair = transferPoints[internalIdxPStop * numStopsDVeh + internalIdxDStop];
                transferPointsForPair.clear();
                auto& localNumTransferPointsBuilt = threadLocalNumTransferPointsBuilt.local();
                intersectEllipses(ellipsePStop, ellipseDStop, vehIdPStop, vehIdDStop,
                                  routeState.stopPositionOf(stopIdPStop),
                                  routeState.stopPositionOf(stopIdDStop), transferPointsForPair,
                                  localNumTransferPointsBuilt);
            });

            for (const auto &localNum: threadLocalNumTransferPointsBuilt) {
                numTransferPointsBuilt += localNum;
            }

            for (const auto &transferPointsForPair: transferPoints) {
                ellipseIntersectionSizeLogger << transferPointsForPair.size() << "\n";
            }
        }

        // Given the IDs of the respective first stops in a pair of stops in a pickup vehicle and a pair of stops in a
        // dropoff vehicle, this returns the feasible transfer points for these stop pairs.
        const std::vector<TransferPoint> &getTransferPoints(const int pVehStopId, const int dVehStopId) const {
            const auto internalIdxPStop = idxOfStopPVeh[pVehStopId];
            const auto internalIdxDStop = idxOfStopDVeh[dVehStopId];
            KASSERT(internalIdxPStop != INVALID_INDEX && internalIdxDStop != INVALID_INDEX);
            return transferPoints[internalIdxPStop * numStopsDVeh + internalIdxDStop];
        }

    private:

        bool transferPointDominates(const TransferPoint &tp1, const TransferPoint &tp2) {
            const auto detourPVeh1 = tp1.distancePVehToTransfer + tp1.distancePVehFromTransfer;
            const auto detourPVeh2 = tp2.distancePVehToTransfer + tp2.distancePVehFromTransfer;
            const auto detourDVeh1 = tp1.distanceDVehToTransfer + tp1.distanceDVehFromTransfer;
            const auto detourDVeh2 = tp2.distanceDVehToTransfer + tp2.distanceDVehFromTransfer;
            const auto tripTime1 = tp1.distancePVehToTransfer + tp1.distanceDVehFromTransfer;
            const auto tripTime2 = tp2.distancePVehToTransfer + tp2.distanceDVehFromTransfer;
            return detourPVeh1 < detourPVeh2 && detourDVeh1 < detourDVeh2 && tripTime1 < tripTime2;
        }

        // Computes intersection of two ellipses. Edges in ellipses must be edge IDs in the original input graph.
        // Ellipses are expected to be sorted in increasing order of edge ID.
        // The result is a vector of transfer points specified with edge IDs of the original input graph.
        // The transfer points are checked for pareto-dominance and only non-dominated points are returned.
        void intersectEllipses(const std::vector<EdgeInEllipse> &ellipsePVeh,
                               const std::vector<EdgeInEllipse> &ellipseDVeh,
                               const int pVehId, const int dVehId,
                               const int stopIdxPVeh, const int stopIdxDVeh,
                               std::vector<TransferPoint> &result,
                               int64_t &numTransferPointsBuilt) {
            result.clear();

            const auto *pVehPtr = &fleet[pVehId];
            const auto *dVehPtr = &fleet[dVehId];

            auto itEdgesPVeh = ellipsePVeh.begin();
            auto itEdgesDVeh = ellipseDVeh.begin();
            const auto endEdgesPVeh = ellipsePVeh.end();
            const auto endEdgesDVeh = ellipseDVeh.end();

            while (itEdgesPVeh < endEdgesPVeh && itEdgesDVeh < endEdgesDVeh) {
                const auto &edgePVeh = *itEdgesPVeh;
                const auto &edgeDVeh = *itEdgesDVeh;
                if (edgePVeh.edge < edgeDVeh.edge) {
                    ++itEdgesPVeh;
                    continue;
                }

                if (edgePVeh.edge > edgeDVeh.edge) {
                    ++itEdgesDVeh;
                    continue;
                }

                const int loc = edgePVeh.edge;
                const int distPVehToTransfer = edgePVeh.distToTail + inputGraph.travelTime(loc);
                const int distPVehFromTransfer = edgePVeh.distFromHead;
                const int distDVehToTransfer = edgeDVeh.distToTail + inputGraph.travelTime(loc);
                const int distDVehFromTransfer = edgeDVeh.distFromHead;


                ++itEdgesPVeh;
                ++itEdgesDVeh;

                const auto tp = TransferPoint(loc, pVehPtr, dVehPtr, stopIdxPVeh, stopIdxDVeh, distPVehToTransfer,
                                              distPVehFromTransfer, distDVehToTransfer, distDVehFromTransfer);
                ++numTransferPointsBuilt;

                if constexpr (DoTransferParetoChecks) {
                    // Check whether known transfer points dominate tp
                    bool dominated = false;
                    for (const auto &knownTP: result) {
                        if (transferPointDominates(knownTP, tp)) {
                            dominated = true;
                            break;
                        }
                    }
                    if (dominated) {
                        continue;
                    }

                    // If tp is not dominated, add it to result and remove any dominated by tp
                    result.push_back(tp);
                    auto numOpt = result.size();
                    for (int i = 0; i < numOpt;) {
                        if (transferPointDominates(tp, result[i])) {
                            std::swap(result[i], result.back());
                            result.pop_back();
                            --numOpt;
                            continue;
                        }
                        ++i;
                    }
                } else {
                    // If we do not check for pareto dominance, we simply add the transfer point
                    result.push_back(tp);
                }
            }

            if constexpr (UseCostLowerBounds) {
                for (int i = 0; i < result.size();) {
                    const auto minTpCost = calc.calcMinCostForTransferPoint(result[i], requestState);
                    if (minTpCost.total > requestState.getBestCost()) {
                        std::swap(result[i], result.back());
                        result.pop_back();
                        continue;
                    }
                    ++i;
                }
            }
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const RequestState &requestState;
        const RouteState &routeState;
        CostCalculator &calc;

        int numStopsPVeh;
        int numStopsDVeh;

        // Maps a stop ID to an internal index in the vector of stop IDs.
        std::vector<int> idxOfStopPVeh;
        std::vector<int> idxOfStopDVeh;

        // For two stop IDs i (pVeh) and j (dVeh), transferPoints[idxOfStopPVeh[i] * numStopsDVeh + idxOfStopDVeh[j]] contains the
        // transfer points between the two stops.
        std::vector<std::vector<TransferPoint>> transferPoints;

        tbb::enumerable_thread_specific<int64_t> threadLocalNumTransferPointsBuilt;

        EllipseSizeLoggerT &ellipsesSizeLogger;
        EllipseIntersectionSizeLoggerT &ellipseIntersectionSizeLogger;
    };

} // karri