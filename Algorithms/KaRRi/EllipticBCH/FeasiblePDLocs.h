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

#include <array>
#include <bit>
#include <vector>

#include "DataStructures/Utilities/IteratorRange.h"
#include "Algorithms/KaRRi/BaseObjects/PDLocs.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "Tools/Timer.h"

namespace karri {

    // Consolidates the result of the elliptic BCH searches into the PD locs that are feasible for every stop, i.e.
    // the PD locs for which the distances in both directions (from the stop and to the next stop) are known and for
    // which a detour via the PD loc does not trivially exceed the leeway of the leg starting at the stop.
    // The feasible PD locs are the basis for filtering relevant PD locs for specific assignment types (see
    // RelevantPDLocsFilter). Since Pareto-based filtering does not apply to paired assignments, the finders for
    // paired assignments work on the feasible PD locs directly.
    class FeasiblePDLocs {

    public:

        struct FeasiblePDLoc {
            int pdId = INVALID_ID;
            int distToPDLoc = INFTY;
            int distFromPDLocToNextStop = INFTY;
        };

    private:

        using FeasibleVec = std::vector<FeasiblePDLoc>;

    public:

        using It = FeasibleVec::const_iterator;

        explicit FeasiblePDLocs(const RouteState &routeState)
                : routeState(routeState),
                  stopInfo(routeState.getMaxStopId() + 1) {}

        // Constructs the feasible PD locs from the result of the elliptic BCH searches.
        template<typename EllipticBCHSearchResultT>
        void construct(const EllipticBCHSearchResultT &searchResult, const int numPDLocs,
                       int64_t &numFeasibleStops, int64_t &numFeasiblePDLocs) {
            using Label = typename EllipticBCHSearchResultT::Label;
            static constexpr int K = EllipticBCHSearchResultT::NUM_PD_LOCS_PER_LABEL;

            clear();

            if (stopInfo.size() < routeState.getMaxStopId() + 1)
                stopInfo.resize(routeState.getMaxStopId() + 1);

            const int numLabels = searchResult.getNumLabelsPerStop();
            std::array<int, K> to, from;

            for (const int &stopId: searchResult.getStopIdsWithRelevantPDLocs()) {
                const int totalNumFeasibleBefore = static_cast<int>(feasiblePdLocs.size());

                // A detour via the PD loc must not exceed the leeway of the leg starting at the stop, i.e. the
                // sum of the distances to and from the PD loc must not exceed the leeway. (The leeway may be
                // larger than the time the detour may actually take if the vehicle waits at the next stop. The
                // exact hard constraints are checked when filtering relevant PD locs.)
                // PD locs at the stop itself (distance 0 from the stop) are always feasible. (The elliptic BCH
                // search results contain only such PD locs for last stops of vehicles.)
                const Label maxDist = Label(routeState.leewayOfLegStartingAt(stopId));

                const Label *distsToPDLocs = searchResult.distanceLabelsToRelevantPDLocsFor(stopId);
                const Label *distsFromPDLocs = searchResult.distanceLabelsFromRelevantPDLocsToNextStopOf(stopId);
                Label minDistToPDLoc = INFTY;
                Label minDistFromPDLocToNextStop = INFTY;
                for (int l = 0; l < numLabels; ++l) {
                    const auto &toLabel = distsToPDLocs[l];
                    const auto &fromLabel = distsFromPDLocs[l];

                    const auto feasible = (toLabel < INFTY) & (fromLabel < INFTY) &
                                          (((toLabel + fromLabel) <= maxDist) | (toLabel == Label(0)));
                    uint64_t bits = feasible.toBits();
                    // A partial last batch of PD locs is filled with copies of the first PD loc of the batch by the
                    // elliptic BCH searches. Ignore these copies. (They do not affect the minimum distances.)
                    if (l == numLabels - 1 && numPDLocs % K != 0)
                        bits &= (uint64_t(1) << (numPDLocs % K)) - 1;
                    if (!bits)
                        continue;

                    minDistToPDLoc.min(select(feasible, toLabel, Label(INFTY)));
                    minDistFromPDLocToNextStop.min(select(feasible, fromLabel, Label(INFTY)));

                    toLabel.storeTo(to.data());
                    fromLabel.storeTo(from.data());
                    size_t idx = feasiblePdLocs.size();
                    feasiblePdLocs.resize(idx + std::popcount(bits));
                    while (bits) {
                        const int k = std::countr_zero(bits);
                        bits &= bits - 1;
                        feasiblePdLocs[idx++] = {l * K + k, to[k], from[k]};
                    }
                }

                const int totalNumFeasibleAfter = static_cast<int>(feasiblePdLocs.size());
                if (totalNumFeasibleAfter == totalNumFeasibleBefore)
                    continue;

                stopInfo[stopId] = {totalNumFeasibleBefore, totalNumFeasibleAfter, minDistToPDLoc.horizontalMin(),
                                    minDistFromPDLocToNextStop.horizontalMin()};
                stopIdsWithFeasiblePDLocs.push_back(stopId);
            }

            numFeasibleStops += static_cast<int64_t>(stopIdsWithFeasiblePDLocs.size());
            numFeasiblePDLocs += static_cast<int64_t>(feasiblePdLocs.size());
        }

        bool hasFeasiblePDLocs(const int stopId) const {
            return stopId < stopInfo.size() && stopInfo[stopId].startIdx != INVALID_INDEX;
        }

        IteratorRange<It> feasiblePdLocsFor(const int stopId) const {
            if (!hasFeasiblePDLocs(stopId))
                return {feasiblePdLocs.cbegin(), feasiblePdLocs.cbegin()};
            const auto &info = stopInfo[stopId];
            return {feasiblePdLocs.cbegin() + info.startIdx, feasiblePdLocs.cbegin() + info.endIdx};
        }

        int minDistToPDLocFor(const int stopId) const {
            return hasFeasiblePDLocs(stopId) ? stopInfo[stopId].minDistToPDLoc : INFTY;
        }

        int minDistFromPDLocToNextStopOf(const int stopId) const {
            return hasFeasiblePDLocs(stopId) ? stopInfo[stopId].minDistFromPDLocToNextStop : INFTY;
        }

        const std::vector<int> &getStopIdsWithFeasiblePDLocs() const {
            return stopIdsWithFeasiblePDLocs;
        }

        int numStopsWithFeasiblePDLocs() const {
            return static_cast<int>(stopIdsWithFeasiblePDLocs.size());
        }

    private:

        // Describes range of feasible PD locs and minimum distance values for a stop that has feasible PD locs.
        struct FeasibleForStop {
            int startIdx = INVALID_INDEX;
            int endIdx = INVALID_INDEX;
            int minDistToPDLoc = INFTY;
            int minDistFromPDLocToNextStop = INFTY;
        };

        void clear() {
            for (const auto &stopId: stopIdsWithFeasiblePDLocs)
                stopInfo[stopId] = FeasibleForStop();
            stopIdsWithFeasiblePDLocs.clear();
            feasiblePdLocs.clear();
        }

        const RouteState &routeState;

        // Iterable set of stop IDs that have feasible PD locs.
        std::vector<int> stopIdsWithFeasiblePDLocs;

        // Maps a stop id to the range of feasible PD locs for this stop in feasiblePdLocs.
        std::vector<FeasibleForStop> stopInfo;

        // Value array.
        FeasibleVec feasiblePdLocs;
    };
}
