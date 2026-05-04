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
#include "DataStructures/Containers/FastResetFlagArray.h"
#include "DataStructures/Containers/LightweightSubset.h"
#include "EllipseReconstructorStats.h"

#include "Tools/Timer.h"

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/combinable.h>

namespace karri {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using Dijkstra searches.
    template<typename InputGraphT, typename LabelSet, typename WeightT = TravelTimeAttribute>
    class DijkstraEllipseReconstructorQuery {

        using DistanceLabel = typename LabelSet::DistanceLabel;
        using LabelMask = typename LabelSet::LabelMask;
        static constexpr int K = LabelSet::K;


        struct PruneIfRadiusExceeded {
            PruneIfRadiusExceeded(const DistanceLabel &radius) : radius(radius) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                return allSet(distToV > radius);
            }

        private:
            const DistanceLabel &radius;
        };

        struct MarkAsScanned {
            MarkAsScanned(FastResetFlagArray<> &scanned) : scanned(scanned) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &, const DistLabelContainerT & /*distLabels*/) {
                scanned.set(v);
                return false;
            }

        private:
            FastResetFlagArray<> &scanned;
        };

        template<typename DistanceContainer>
        struct MemorizeIfInAnyEllipse {
            MemorizeIfInAnyEllipse(const DistanceLabel &leeway,
                                   DistanceContainer &distancesInOther,
                                   const FastResetFlagArray<> &scannedInOther,
                                   LightweightSubset &inAnyEllipse)
                    : leeway(leeway),
                      distancesInOther(distancesInOther),
                      scannedInOther(scannedInOther),
                      inAnyEllipse(inAnyEllipse) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                if (!scannedInOther.isSet(v) || inAnyEllipse.contains(v))
                    return false;

                const auto withinLeeway = distToV + distancesInOther[v] <= leeway;
                if (anySet(withinLeeway))
                    inAnyEllipse.insert(v);

                return false;
            }

        private:
            const DistanceLabel &leeway;
            DistanceContainer &distancesInOther;
            const FastResetFlagArray<> &scannedInOther;
            LightweightSubset &inAnyEllipse;
        };

        using MemorizeImpl = MemorizeIfInAnyEllipse<StampedDistanceLabelContainer<DistanceLabel>>;

    public:

        DijkstraEllipseReconstructorQuery(const InputGraphT &inputGraph,
                                       const InputGraphT &reverseGraph,
                                       const RouteState &routeState)
                : inputGraph(inputGraph),
                  reverseGraph(reverseGraph),
                  numVertices(inputGraph.numVertices()),
                  routeState(routeState),
                  curLeeways(),
                  scannedInForwardSearch(numVertices),
                  inAnyEllipse(numVertices),
                  forwardSearch(inputGraph, {}, {PruneIfRadiusExceeded(curLeeways), MarkAsScanned(scannedInForwardSearch)}),
                  reverseSearch(reverseGraph, {}, {PruneIfRadiusExceeded(curLeeways),
                                MemorizeImpl(curLeeways, forwardSearch.distanceLabels, scannedInForwardSearch,
                                             inAnyEllipse)}) {}

        void init() {}

        void run(const std::array<int, K> &stopIds,
                 const DistanceLabel &leeways,
                 const int numEllipses, // number of stopIds actually used in batch
                 std::vector<std::vector<VertexInEllipse>>::iterator firstEllipseInBatch,
                 EllipseReconstructorStats &stats) {
            KASSERT(numEllipses <= K);

            Timer timer;
            stats.initTime += timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            std::array<int, K> sources = {};
            std::array<int, K> zeroOffsets;
            zeroOffsets.fill(0);
            std::array<int, K> targets = {};
            std::array<int, K> offsets = {};
            for (int i = 0; i < numEllipses; ++i) {
                const auto stopLocations = routeState.stopLocationsFor(
                        routeState.vehicleIdOf(stopIds[i]));
                const int stopIndex = routeState.stopPositionOf(stopIds[i]);
                const auto stopLoc = stopLocations[stopIndex];
                const auto nextStopLoc = stopLocations[stopIndex + 1];
                sources[i] = inputGraph.edgeHead(stopLoc);
                targets[i] = inputGraph.edgeTail(nextStopLoc);
                offsets[i] = inputGraph.travelTime(nextStopLoc);
            }
            for (int i = numEllipses; i < K; ++i) {
                sources[i] = sources[0]; // Use first source for padding
                targets[i] = targets[0]; // Use first target for padding
                offsets[i] = offsets[0];
            }

            curLeeways = leeways;
            inAnyEllipse.clear();
            scannedInForwardSearch.reset();
            forwardSearch.runWithOffset(sources, zeroOffsets);
            reverseSearch.runWithOffset(targets, offsets);

            stats.topoSearchTime += timer.elapsed<std::chrono::nanoseconds>();
            stats.numVerticesSettled += forwardSearch.getNumVerticesSettled() + reverseSearch.getNumVerticesSettled();
            stats.numEdgesRelaxed += forwardSearch.getNumEdgeRelaxations() + reverseSearch.getNumEdgeRelaxations();

            // Accumulate result per ellipse
            timer.restart();
            auto &forwDist = forwardSearch.distanceLabels;
            auto &revDist = reverseSearch.distanceLabels;
            for (const auto &v: inAnyEllipse) {
                const auto &distToVertex = forwDist[v];
                const auto &distFromVertex = revDist[v];
                const auto breaksLeeway = distToVertex + distFromVertex > leeways;
                KASSERT(!allSet(breaksLeeway));
                for (int j = 0; j < numEllipses; ++j) {
                    if (!breaksLeeway[j]) {
                        KASSERT(distToVertex[j] < INFTY && distFromVertex[j] < INFTY);
                        firstEllipseInBatch[j].emplace_back(v, distToVertex[j], distFromVertex[j]);
                    }
                }
            }
            stats.postprocessTime += timer.elapsed<std::chrono::nanoseconds>();
        }

    private:

        const InputGraphT &inputGraph;
        const InputGraphT &reverseGraph;
        const size_t numVertices;
        const RouteState &routeState;

        DistanceLabel curLeeways;
        FastResetFlagArray<> scannedInForwardSearch; // Flags that mark whether vertex has been scanned in forward search.
        LightweightSubset inAnyEllipse; // Contains all vertices that are in at least one of the K ellipses.

        using ForwardSearch = Dijkstra<InputGraphT, WeightT, LabelSet, dij::NoCriterion, dij::CompoundCriterion<PruneIfRadiusExceeded, MarkAsScanned>, StampedDistanceLabelContainer>;
        using ReverseSearch = Dijkstra<InputGraphT, WeightT, LabelSet, dij::NoCriterion, dij::CompoundCriterion<PruneIfRadiusExceeded, MemorizeImpl>, StampedDistanceLabelContainer>;
        ForwardSearch forwardSearch;
        ReverseSearch reverseSearch;
    };

} // karri
