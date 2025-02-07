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
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "VertexInEllipse.h"

namespace karri {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using Dijkstra searches.
    template<typename InputGraphT, typename WeightT = TravelTimeAttribute>
    class DijkstraEllipseReconstructor {


    private:

        struct StopWhenLeewayExceeded {
            explicit StopWhenLeewayExceeded(const int &currentLeeway) : currentLeeway(currentLeeway) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, const DistLabelT &distToV, const DistLabelContT &) const noexcept {
                return allSet(distToV > currentLeeway);
            }

            const int &currentLeeway;
        };

        struct StoreSearchSpace {
            explicit StoreSearchSpace(std::vector<int> &searchSpace) : searchSpace(searchSpace) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &, const DistLabelContT &) const noexcept {
                searchSpace.push_back(v);
                return false;
            }

            std::vector<int> &searchSpace;
        };


    public:

        DijkstraEllipseReconstructor(const InputGraphT& inputGraph, const InputGraphT& revGraph, const RouteState& routeState)
        : inputGraph(inputGraph),
        revGraph(revGraph),
        routeState(routeState),
        forwardSearchSpace(),
        reverseSearchSpace(),
        forwardSearch(inputGraph, StopWhenLeewayExceeded(curLeeway), StoreSearchSpace(forwardSearchSpace)),
        reverseSearch(revGraph, StopWhenLeewayExceeded(curLeeway), StoreSearchSpace(reverseSearchSpace)) {}


        std::vector<VertexInEllipse> getVerticesInEllipseOfLegAfter(const int stopId, int& numVerticesSettled, int &numEdgesRelaxed) {

            const int vehId = routeState.vehicleIdOf(stopId);
            const int stopIdx = routeState.stopPositionOf(stopId);
            KASSERT(stopIdx < routeState.numStopsOf(vehId) - 1);
            const auto stopLocations = routeState.stopLocationsFor(vehId);

            forwardSearchSpace.clear();
            reverseSearchSpace.clear();
            curLeeway = routeState.leewayOfLegStartingAt(stopId);

            forwardSearch.runWithOffset(inputGraph.edgeHead(stopLocations[stopIdx]), 0);
            const int offsetNextStop = inputGraph.template get<WeightT>(stopLocations[stopIdx + 1]);
            reverseSearch.runWithOffset(inputGraph.edgeTail(stopLocations[stopIdx + 1]), offsetNextStop);

            numVerticesSettled = forwardSearch.getNumVerticesSettled() + reverseSearch.getNumVerticesSettled();
            numEdgesRelaxed = forwardSearch.getNumEdgeRelaxations() + reverseSearch.getNumEdgeRelaxations();



            std::sort(forwardSearchSpace.begin(), forwardSearchSpace.end());
            std::sort(reverseSearchSpace.begin(), reverseSearchSpace.end());

            std::vector<VertexInEllipse> verticesInEllipse;
            int iF = 0, iR = 0;
            while (iF < forwardSearchSpace.size() && iR < reverseSearchSpace.size()) {

                while (iF < forwardSearchSpace.size() && forwardSearchSpace[iF] < reverseSearchSpace[iR])
                    ++iF;
                if (iF == forwardSearchSpace.size())
                    break;
                while (iR < reverseSearchSpace.size() && reverseSearchSpace[iR] < forwardSearchSpace[iF])
                    ++iR;
                if (iR == reverseSearchSpace.size())
                    break;

                if (forwardSearchSpace[iF] == reverseSearchSpace[iR]) {
                    const auto v = forwardSearchSpace[iF];
                    if (forwardSearch.getDistance(v) + reverseSearch.getDistance(v) <= curLeeway)
                        verticesInEllipse.push_back({v, forwardSearch.getDistance(v), reverseSearch.getDistance(v)});
                    ++iF;
                    ++iR;
                }
            }
            return verticesInEllipse;
        }

        std::vector<VertexInEllipse> getVerticesInEllipseOfLegBefore(const int stopId, int& numVerticesSettled, int &numEdgesRelaxed) {
            KASSERT(routeState.stopPositionOf(stopId) > 0);
            KASSERT(routeState.stopPositionOf(stopId) < routeState.numStopsOf(routeState.vehicleIdOf(stopId)));
            return getVerticesInEllipseOfLegAfter(routeState.idOfPreviousStopOf(stopId), numVerticesSettled, numEdgesRelaxed);
        }


    private:


        const InputGraphT &inputGraph;
        const InputGraphT &revGraph;
        const RouteState &routeState;

        using Search = Dijkstra<InputGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, StopWhenLeewayExceeded, StoreSearchSpace>;
        int curLeeway;
        std::vector<int> forwardSearchSpace;
        std::vector<int> reverseSearchSpace;
        Search forwardSearch;
        Search reverseSearch;

    };

} // karri
