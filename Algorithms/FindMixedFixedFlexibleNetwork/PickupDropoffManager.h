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


        // Call defrag() first!
        void writeTo(std::ofstream &out) const {
            bio::write(out, pickupIndex);
            bio::write(out, possiblePickups);
            bio::write(out, pickupWalkingDists);
            bio::write(out, dropoffIndex);
            bio::write(out, possibleDropoffs);
            bio::write(out, dropoffWalkingDists);
        }

        void readFrom(std::ifstream &in) {
            bio::read(in, pickupIndex);
            bio::read(in, possiblePickups);
            bio::read(in, pickupWalkingDists);
            bio::read(in, dropoffIndex);
            bio::read(in, possibleDropoffs);
            bio::read(in, dropoffWalkingDists);
        }

        void findPossiblePDLocsForRequests(const std::vector<Request> &requests) {

            progressBar.init(requests.size());

            pickupIndex.resize(vehGraph.numVertices());
            std::fill(pickupIndex.begin(), pickupIndex.end(), ValueBlockPosition{0, 0});
            dropoffIndex.resize(vehGraph.numVertices());
            std::fill(dropoffIndex.begin(), dropoffIndex.end(), ValueBlockPosition{0, 0});

            possiblePickups.clear();
            pickupWalkingDists.clear();
            possibleDropoffs.clear();
            dropoffWalkingDists.clear();

            // TODO: SIMD-ify, parallelize
            for (const auto &req: requests) {
                addPickupAtVertex(req.requestId, 0, vehGraph.edgeHead(req.origin));
                addDropoffAtVertex(req.requestId, 0, vehGraph.edgeHead(req.destination));

                const int originInPsg = vehGraph.mapToEdgeInPsg(req.origin);
                const int destInPsg = vehGraph.mapToEdgeInPsg(req.destination);

                searchSpace.clear();
                auto headOfOriginEdge = forwardPsgGraph.edgeHead(originInPsg);
                pickupSearch.run(headOfOriginEdge);
                turnSearchSpaceIntoPickupLocations(req.requestId);

                searchSpace.clear();
                auto headOfDestEdge = forwardPsgGraph.edgeHead(destInPsg);
                dropoffSearch.run(headOfDestEdge);
                turnSearchSpaceIntoDropoffLocations(req.requestId);

                ++progressBar;
            }

            // TODO: Post-process 2D-arrays, (potentially sorting)
            defrag();
        }

        // Returns IDs of requests that may be picked up at vertex v.
        ConstantVectorRange<int> getPossiblePickupsAt(const int v) const {
            assert(v >= 0);
            assert(v < vehGraph.numEdges());
            const auto start = pickupIndex[v].start;
            const auto end = pickupIndex[v].end;
            return {possiblePickups.begin() + start, possiblePickups.begin() + end};
        }

        // Returns IDs of requests that may be picked up at vertex v.
        ConstantVectorRange<int> getPickupWalkingDistsAt(const int v) const {
            assert(v >= 0);
            assert(v < vehGraph.numEdges());
            const auto start = pickupIndex[v].start;
            const auto end = pickupIndex[v].end;
            return {pickupWalkingDists.begin() + start, pickupWalkingDists.begin() + end};
        }

        // Returns IDs of requests that may be dropped off at vertex v.
        ConstantVectorRange<int> getPossibleDropoffsAt(const int v) const {
            assert(v >= 0);
            assert(v < vehGraph.numEdges());
            const auto start = dropoffIndex[v].start;
            const auto end = dropoffIndex[v].end;
            return {possibleDropoffs.begin() + start, possibleDropoffs.begin() + end};
        }

        // Returns IDs of requests that may be dropped off at vertex v.
        ConstantVectorRange<int> getDropoffWalkingDistsAt(const int v) const {
            assert(v >= 0);
            assert(v < vehGraph.numEdges());
            const auto start = dropoffIndex[v].start;
            const auto end = dropoffIndex[v].end;
            return {dropoffWalkingDists.begin() + start, dropoffWalkingDists.begin() + end};
        }


    private:


        void turnSearchSpaceIntoPickupLocations(const int requestId) {
            for (const auto &v: searchSpace) {
                const auto distToV = pickupSearch.getDistance(v);
                assert(distToV <= InputConfig::getInstance().walkingRadius);
                FORALL_INCIDENT_EDGES(forwardPsgGraph, v, e) {
                    const int walkingDist = distToV + forwardPsgGraph.travelTime(e);
                    const int eInVehGraph = forwardPsgGraph.mapToEdgeInFullVeh(e);
                    if (eInVehGraph == MapToEdgeInFullVehAttribute::defaultValue() || walkingDist > InputConfig::getInstance().walkingRadius)
                        continue;

                    addPickupAtVertex(requestId, walkingDist, vehGraph.edgeHead(eInVehGraph));
                    KASSERT(pickupIndex[vehGraph.edgeHead(eInVehGraph)].start >= 0 && pickupIndex[vehGraph.edgeHead(eInVehGraph)].end <= possiblePickups.size());
                }
            }
        }

        void turnSearchSpaceIntoDropoffLocations(const int requestId) {
            for (const auto &v: searchSpace) {
                const auto distToV = dropoffSearch.getDistance(v);
                assert(distToV <= InputConfig::getInstance().walkingRadius);
                FORALL_INCIDENT_EDGES(reversePsgGraph, v, e) {
                    const auto eInForwGraph = reversePsgGraph.edgeId(e);
                    const int eInVehGraph = forwardPsgGraph.mapToEdgeInFullVeh(eInForwGraph);
                    if (eInVehGraph == MapToEdgeInFullVehAttribute::defaultValue())
                        continue;
                    addDropoffAtVertex(requestId, distToV, vehGraph.edgeHead(eInVehGraph));
                    KASSERT(dropoffIndex[vehGraph.edgeHead(eInVehGraph)].start >= 0 && dropoffIndex[vehGraph.edgeHead(eInVehGraph)].end <= possibleDropoffs.size());
                }
            }
        }


        void addPickupAtVertex(const int requestId, const int walkingDist, const int v) {
            assert(v >= 0);
            assert(v < vehGraph.numVertices());
            const auto start = pickupIndex[v].start;
            const auto end = pickupIndex[v].end;

            // TODO: sort value arrays?
            // Do not add duplicates
            if (contains(possiblePickups.begin() + start, possiblePickups.begin() + end, requestId))
                return;

            const int idx = insertion(v, requestId, pickupIndex, possiblePickups, pickupWalkingDists);
            pickupWalkingDists[idx] = walkingDist;
        }

        void addDropoffAtVertex(const int requestId, const int walkingDist, const int v) {
            assert(v >= 0);
            assert(v < vehGraph.numEdges());
            const auto start = dropoffIndex[v].start;
            const auto end = dropoffIndex[v].end;

            // TODO: sort value arrays?
            // Do not add duplicates
            if (contains(possibleDropoffs.begin() + start, possibleDropoffs.begin() + end, requestId))
                return;

            const int idx = insertion(v, requestId, dropoffIndex, possibleDropoffs, dropoffWalkingDists);
            dropoffWalkingDists[idx] = walkingDist;
        }

        // De-fragment 2D-arrays after constructing them
        void defrag() {
            defragImpl(pickupIndex, possiblePickups, pickupWalkingDists);
            defragImpl(dropoffIndex, possibleDropoffs, dropoffWalkingDists);
        }

        void defragImpl(std::vector<ValueBlockPosition> &index, std::vector<int> &possiblePD,
                        std::vector<int> &walkingDists) {
            int totalPossiblePDs = 0;
            for (const auto &pos: index)
                totalPossiblePDs += pos.end - pos.start;

            std::vector<ValueBlockPosition> newIndex(index.size());
            std::vector<int> newPossiblePD;
            std::vector<int> newWalkingDists;
            newPossiblePD.reserve(totalPossiblePDs);
            newWalkingDists.reserve(totalPossiblePDs);

            for (int e = 0; e < index.size(); ++e) {
                newIndex[e].start = newPossiblePD.size();
                newPossiblePD.insert(newPossiblePD.end(), possiblePD.begin() + index[e].start,
                                     possiblePD.begin() + index[e].end);
                newWalkingDists.insert(newWalkingDists.end(), walkingDists.begin() + index[e].start,
                                       walkingDists.begin() + index[e].end);
                newIndex[e].end = newPossiblePD.size();
            }
            index = std::move(newIndex);
            possiblePD = std::move(newPossiblePD);
            walkingDists = std::move(newWalkingDists);
        }


        const VehInputGraphT &vehGraph;
        const PsgInputGraphT &forwardPsgGraph;
        const PsgInputGraphT &reversePsgGraph;

        Search pickupSearch;
        Search dropoffSearch;
        std::vector<int> searchSpace;

        // We use two DynamicRagged2DArray to store the subset of requests that can be picked up/ dropped off at each
        // vertex. For both 2D-array, there is a single index array that for each vertex stores a range of indices where
        // the entries for that vertex are stored in a number of value arrays.

        // Pickup Index Array: For each vertex (of the vehicle graph) v, the according entries in the pickup value arrays
        // lie in the index interval [pickupIndex[v].start, pickupIndex[v].end).
        std::vector<ValueBlockPosition> pickupIndex;

        // IDs of requests that can be picked up at vertex.
        std::vector<int> possiblePickups;

        // IDs of requests that can be picked up at vertex.
        std::vector<int> pickupWalkingDists;

        // Dropoff Index Array: For each vertex (of the vehicle graph) v, the according entries in the dropoff value arrays
        // lie in the index interval [dropoffIndex[v].start, dropoffIndex[v].end).
        std::vector<ValueBlockPosition> dropoffIndex;

        // IDs of requests that can be dropped off at vertex.
        std::vector<int> possibleDropoffs;

        // IDs of requests that can be dropped off at vertex.
        std::vector<int> dropoffWalkingDists;


        ProgressBar progressBar;

    };
} // end namespace