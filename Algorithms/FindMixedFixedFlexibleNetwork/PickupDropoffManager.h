/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInFullVehAttribute.h"
#include "DataStructures/Graph/Attributes/MapToEdgeInPsgAttribute.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "FixedLine.h"
#include "Request.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "PathStartEndInfo.h"

namespace mixfix {


    template<typename VehInputGraphT,
            typename PsgInputGraphT,
            typename WeightT = TravelTimeAttribute>
    class PickupDropoffManager {


    private:
        struct StopWhenRadiusExceeded {
            StopWhenRadiusExceeded(const int& radius) : radius(radius) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                return distToV[0] > radius;
            }

        private:
            const int& radius;
        };

        struct RememberSearchSpace {

            RememberSearchSpace(std::vector<int> &searchSpace) : searchSpace(searchSpace) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &, const DistLabelContT &) {
                searchSpace.push_back(v);
                return false;
            }

        private:
            std::vector<int> &searchSpace;

        };

        using Search = Dijkstra<PsgInputGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, StopWhenRadiusExceeded, RememberSearchSpace>;

    public:

        PickupDropoffManager(const VehInputGraphT &vehGraph,
                             const PsgInputGraphT &forwardPsgGraph,
                             const PsgInputGraphT &reversePsgGraph,
                             const int walkingRadius)
                : vehGraph(vehGraph),
                  forwardPsgGraph(forwardPsgGraph),
                  reversePsgGraph(reversePsgGraph),
                  walkingRadius(walkingRadius),
                  pickupSearch(forwardPsgGraph, {walkingRadius}, {searchSpace}),
                  dropoffSearch(reversePsgGraph, {walkingRadius}, {searchSpace}),
                  searchSpace(),
                  progressBar() {}




        void findPossiblePDLocsForRequests(const std::vector<Request> &requests,
                                           PathStartEndInfo& pdInfo) {

            progressBar.init(static_cast<int>(requests.size()));

            // TODO: SIMD-ify, parallelize
            for (const auto &req: requests) {
                pdInfo.addPathBeginningAtVertex(req.requestId, 0, vehGraph.edgeHead(req.origin));
                pdInfo.addPathEndAtVertex(req.requestId, 0, vehGraph.edgeHead(req.destination));

                const int originInPsg = vehGraph.mapToEdgeInPsg(req.origin);
                const int destInPsg = vehGraph.mapToEdgeInPsg(req.destination);

                searchSpace.clear();
                auto headOfOriginEdge = forwardPsgGraph.edgeHead(originInPsg);
                pickupSearch.run(headOfOriginEdge);
                turnSearchSpaceIntoPickupLocations(req.requestId, pdInfo);

                searchSpace.clear();
                auto headOfDestEdge = forwardPsgGraph.edgeHead(destInPsg);
                dropoffSearch.run(headOfDestEdge);
                turnSearchSpaceIntoDropoffLocations(req.requestId, pdInfo);

                ++progressBar;
            }
        }

    private:


        void turnSearchSpaceIntoPickupLocations(const int requestId, PathStartEndInfo& pdInfo) {
            for (const auto &v: searchSpace) {
                const auto distToV = pickupSearch.getDistance(v);
                assert(distToV <= walkingRadius);
                FORALL_INCIDENT_EDGES(forwardPsgGraph, v, e) {
                    const int walkingDist = distToV + forwardPsgGraph.travelTime(e);
                    const int eInVehGraph = forwardPsgGraph.mapToEdgeInFullVeh(e);
                    if (eInVehGraph == MapToEdgeInFullVehAttribute::defaultValue() || walkingDist > walkingRadius)
                        continue;

                    pdInfo.addPathBeginningAtVertex(requestId, walkingDist, vehGraph.edgeHead(eInVehGraph));
                }
            }
        }

        void turnSearchSpaceIntoDropoffLocations(const int requestId, PathStartEndInfo& pdInfo) {
            for (const auto &v: searchSpace) {
                const auto distToV = dropoffSearch.getDistance(v);
                assert(distToV <= walkingRadius);
                FORALL_INCIDENT_EDGES(reversePsgGraph, v, e) {
                    const auto eInForwGraph = reversePsgGraph.edgeId(e);
                    const int eInVehGraph = forwardPsgGraph.mapToEdgeInFullVeh(eInForwGraph);
                    if (eInVehGraph == MapToEdgeInFullVehAttribute::defaultValue())
                        continue;
                    pdInfo.addPathEndAtVertex(requestId, distToV, vehGraph.edgeHead(eInVehGraph));
                }
            }
        }


        const VehInputGraphT &vehGraph;
        const PsgInputGraphT &forwardPsgGraph;
        const PsgInputGraphT &reversePsgGraph;
        const int walkingRadius;

        Search pickupSearch;
        Search dropoffSearch;
        std::vector<int> searchSpace;

        ProgressBar progressBar;

    };
} // end namespace