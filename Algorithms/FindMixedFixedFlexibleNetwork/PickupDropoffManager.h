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
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "FixedLine.h"
#include "Request.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "InputConfig.h"
#include "Tools/CommandLine/ProgressBar.h"

namespace mixfix {


    template<typename VehInputGraphT,
            typename PsgInputGraphT,
            typename WeightT = TravelTimeAttribute>
    class PickupDropoffManager {


    private:
        struct StopWhenRadiusExceeded {
            StopWhenRadiusExceeded(const int radius) : radius(radius) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                return distToV[0] > radius;
            }

        private:
            const int radius;
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
                             const PsgInputGraphT &reversePsgGraph)
                : vehGraph(vehGraph),
                  forwardPsgGraph(forwardPsgGraph),
                  reversePsgGraph(reversePsgGraph),
                  pickupSearch(forwardPsgGraph, {InputConfig::getInstance().walkingRadius}, {searchSpace}),
                  dropoffSearch(reversePsgGraph, {InputConfig::getInstance().walkingRadius}, {searchSpace}),
                  searchSpace(),
                  progressBar() {}

        void findPossiblePDLocsForRequests(const std::vector<Request> &requests) {

            progressBar.init(requests.size());

            pickupIndex.resize(vehGraph.numEdges());
            std::fill(pickupIndex.begin(), pickupIndex.end(), ValueBlockPosition{0,0});
            dropoffIndex.resize(vehGraph.numEdges());
            std::fill(dropoffIndex.begin(), dropoffIndex.end(), ValueBlockPosition{0,0});

            possiblePickups.clear();
            pickupWalkingDists.clear();
            possibleDropoffs.clear();
            dropoffWalkingDists.clear();

            // TODO: SIMD-ify, parallelize
            for (const auto &req: requests) {
                addPickupAtEdge(req.requestId, 0, req.origin);
                addDropoffAtEdge(req.requestId, 0, req.destination);

                const int originInPsg = vehGraph.toPsgEdge(req.origin);
                const int destInPsg = vehGraph.toPsgEdge(req.destination);

                searchSpace.clear();
                auto headOfOriginEdge = forwardPsgGraph.edgeHead(originInPsg);
                pickupSearch.run(headOfOriginEdge);
                turnSearchSpaceIntoPickupLocations(req.requestId);

                searchSpace.clear();
                auto tailOfDestEdge = forwardPsgGraph.edgeTail(destInPsg);
                auto destOffset = forwardPsgGraph.travelTime(destInPsg);
                dropoffSearch.runWithOffset(tailOfDestEdge, destOffset);
                turnSearchSpaceIntoDropoffLocations(req.requestId);

                ++progressBar;
            }

            // TODO: Post-process 2D-arrays, (compression, potentially sorting)

        }

        // Returns IDs of requests that may be picked up at edge e.
        ConstantVectorRange<int> getPossiblePickupsAt(const int e) const {
            assert(e >= 0);
            assert(e < vehGraph.numEdges());
            const auto start = pickupIndex[e].start;
            const auto end = pickupIndex[e].end;
            return {possiblePickups.begin() + start, possiblePickups.begin() + end};
        }

        // Returns IDs of requests that may be picked up at edge e.
        ConstantVectorRange<int> getPickupWalkingDistsAt(const int e) const {
            assert(e >= 0);
            assert(e < vehGraph.numEdges());
            const auto start = pickupIndex[e].start;
            const auto end = pickupIndex[e].end;
            return {pickupWalkingDists.begin() + start, pickupWalkingDists.begin() + end};
        }

        // Returns IDs of requests that may be dropped off at edge e.
        ConstantVectorRange<int> getPossibleDropoffsAt(const int e) const {
            assert(e >= 0);
            assert(e < vehGraph.numEdges());
            const auto start = dropoffIndex[e].start;
            const auto end = dropoffIndex[e].end;
            return {possibleDropoffs.begin() + start, possibleDropoffs.begin() + end};
        }

        // Returns IDs of requests that may be dropped off at edge e.
        ConstantVectorRange<int> getDropoffWalkingDistsAt(const int e) const {
            assert(e >= 0);
            assert(e < vehGraph.numEdges());
            const auto start = dropoffIndex[e].start;
            const auto end = dropoffIndex[e].end;
            return {dropoffWalkingDists.begin() + start, dropoffWalkingDists.begin() + end};
        }


    private:

        void turnSearchSpaceIntoPickupLocations(const int requestId) {
            for (const auto &v: searchSpace) {
                const auto distToV = pickupSearch.getDistance(v);
                assert(distToV <= InputConfig::getInstance().walkingRadius);
                FORALL_INCIDENT_EDGES(forwardPsgGraph, v, e) {
                    const int walkingDist = distToV + forwardPsgGraph.travelTime(e);
                    const int eInVehGraph = forwardPsgGraph.toCarEdge(e);
                    if (eInVehGraph == PsgEdgeToCarEdgeAttribute::defaultValue() || walkingDist > InputConfig::getInstance().walkingRadius)
                        continue;

                    addPickupAtEdge(requestId, walkingDist, eInVehGraph);
                    KASSERT(pickupIndex[eInVehGraph].start >= 0 && pickupIndex[eInVehGraph].end <= possiblePickups.size());
                }
            }
        }

        void turnSearchSpaceIntoDropoffLocations(const int requestId) {
            for (const auto &v: searchSpace) {
                const auto distToV = dropoffSearch.getDistance(v);
                assert(distToV <= InputConfig::getInstance().walkingRadius);
                FORALL_INCIDENT_EDGES(reversePsgGraph, v, e) {
                    const auto eInForwGraph = reversePsgGraph.edgeId(e);
                    const int eInVehGraph = forwardPsgGraph.toCarEdge(eInForwGraph);
                    if (eInVehGraph == PsgEdgeToCarEdgeAttribute::defaultValue())
                        continue;
                    addDropoffAtEdge(requestId, distToV, eInVehGraph);
                    KASSERT(dropoffIndex[eInVehGraph].start >= 0 && dropoffIndex[eInVehGraph].end <= possibleDropoffs.size());
                }
            }
        }


        void addPickupAtEdge(const int requestId, const int walkingDist, const int e) {
            assert(e >= 0);
            assert(e < vehGraph.numEdges());
            const auto start = pickupIndex[e].start;
            const auto end = pickupIndex[e].end;

            // TODO: sort value arrays?
            // Do not add duplicates
            if (contains(possiblePickups.begin() + start, possiblePickups.begin() + end, requestId))
                return;

            const int idx = insertion(e, requestId, pickupIndex, possiblePickups, pickupWalkingDists);
            pickupWalkingDists[idx] = walkingDist;
        }

        void addDropoffAtEdge(const int requestId, const int walkingDist, const int e) {
            assert(e >= 0);
            assert(e < vehGraph.numEdges());
            const auto start = dropoffIndex[e].start;
            const auto end = dropoffIndex[e].end;

            // TODO: sort value arrays?
            // Do not add duplicates
            if (contains(possibleDropoffs.begin() + start, possibleDropoffs.begin() + end, requestId))
                return;

            const int idx = insertion(e, requestId, dropoffIndex, possibleDropoffs, dropoffWalkingDists);
            dropoffWalkingDists[idx] = walkingDist;
        }


        const VehInputGraphT &vehGraph;
        const PsgInputGraphT &forwardPsgGraph;
        const PsgInputGraphT &reversePsgGraph;

        Search pickupSearch;
        Search dropoffSearch;
        std::vector<int> searchSpace;

        // We use two DynamicRagged2DArray to store the subset of requests that can be picked up( dropped off at each
        // edge. For both 2D-array, there is a single index array that for each edge stores a range of indices where
        // the entries for that edge are stored in a number of value arrays.

        // Pickup Index Arra<: For each edge (of the vehicle graph) e, the according entries in the pickup value arrays
        // lie in the index interval [pickupIndex[e].start, pickupIndex[e].end).
        std::vector<ValueBlockPosition> pickupIndex;

        // IDs of requests that can be picked up at edge.
        std::vector<int> possiblePickups;

        // IDs of requests that can be picked up at edge.
        std::vector<int> pickupWalkingDists;

        // Dropoff Index Array: For each edge (of the vehicle graph) e, the according entries in the dropoff value arrays
        // lie in the index interval [dropoffIndex[e].start, dropoffIndex[e].end).
        std::vector<ValueBlockPosition> dropoffIndex;

        // IDs of requests that can be dropped off at edge.
        std::vector<int> possibleDropoffs;

        // IDs of requests that can be dropped off at edge.
        std::vector<int> dropoffWalkingDists;


        ProgressBar progressBar;

    };
} // end namespace