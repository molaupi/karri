/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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


// Ways to map a vertex in the target graph to a desired output location (vertex or edge).
// Each mapping may be given an output location that has to be avoided.
namespace target_vertex_to_output_location {

    template<typename CloseEligibleVertexChooserT>
    struct AvoidVertex {

        AvoidVertex(CloseEligibleVertexChooserT& chooser) : closeEligibleVertexChooser(chooser) {}

        template<typename T>
        int operator()(const int v,  const T&, const int vertexToAvoid = INVALID_ID) const {
            if (v == vertexToAvoid) {
                return closeEligibleVertexChooser.findOtherVertex(v);
            }
            return v;
        }

        CloseEligibleVertexChooserT &closeEligibleVertexChooser;
    };


    template<typename GraphT,
            typename IsEdgeEligibleT,
            typename CloseEligibleVertexChooserT>
    struct RandomEligibleIncidentEdge {

        RandomEligibleIncidentEdge(const GraphT &revTargetGraph,
                                   const IsEdgeEligibleT &isEdgeEligible,
                                   CloseEligibleVertexChooserT &closeEligibleVertexChooser,
                                   const int seed = 0)
                : revTargetGraph(revTargetGraph),
                  isEdgeEligible(isEdgeEligible),
                  closeEligibleVertexChooser(closeEligibleVertexChooser),
                  rand(seed + 1) {}

        template<typename T>
        int operator()(const int v, const T&, const int edgeToAvoid = INVALID_ID) {

            // If the only eligible incoming edge to the chosen head is avoidLocInTar, then we have to choose a different
            // head. We find a new vertex in the vicinity and pick its best incoming edge.
            std::vector<int> incEdges = getEligibleIncomingEdges(v);
            if (incEdges.size() == 1 && incEdges[0] == edgeToAvoid) {
                incEdges = getEligibleIncomingEdges(closeEligibleVertexChooser.findOtherVertex(v));
            }

            return chooseElementUniformlyAtRandom(incEdges);
        }

    private:

        // Gets all eligible incident edges in target graph.
        std::vector<int> getEligibleIncomingEdges(const int v) const {
            std::vector<int> eligibleEdges;

            // Eligible incoming edges
            FORALL_INCIDENT_EDGES(revTargetGraph, v, e) {
                const auto edgeId = revTargetGraph.edgeId(e);
                if (isEdgeEligible(edgeId))
                    eligibleEdges.push_back(edgeId);
            }
            return eligibleEdges;
        }

        template<typename ElementsT>
        int chooseElementUniformlyAtRandom(const ElementsT &elements) {
            assert(!elements.empty());
            const auto idx = std::uniform_int_distribution<>(0, static_cast<int>(elements.size()) - 1)(rand);
            return elements[idx];
        }

        const GraphT& revTargetGraph;
        const IsEdgeEligibleT &isEdgeEligible;
        CloseEligibleVertexChooserT &closeEligibleVertexChooser;
        std::minstd_rand rand;

    };


    template<typename GraphT,
            typename IsEdgeEligibleT,
            typename CloseEligibleVertexChooserT>
    struct EligibleIncidentEdgeWithClosestTail {

        EligibleIncidentEdgeWithClosestTail(const GraphT &sourceGraph,
                                            const GraphT &targetGraph,
                                            const GraphT &revTargetGraph,
                                            const IsEdgeEligibleT &isEdgeEligible,
                                            CloseEligibleVertexChooserT &closeEligibleVertexChooser)
                : sourceGraph(sourceGraph),
                  targetGraph(targetGraph),
                  revTargetGraph(revTargetGraph),
                  isEdgeEligible(isEdgeEligible),
                  closeEligibleVertexChooser(closeEligibleVertexChooser) {}


        int operator()(const int v, const int& srcEdge, const int edgeToAvoid = INVALID_ID) const {

            const auto srcTailLatLng = sourceGraph.latLng(sourceGraph.edgeTail(srcEdge));

            // Find best fitting edge (the one with the tail closest to the tail in src):
            auto pickBestEligibleIncEdge = [&](const int v) -> int {
                double bestTailDist = std::numeric_limits<double>::max();
                int bestTarEdge = INVALID_EDGE;
                auto eligibleIncEdgesToHead = getEligibleIncomingEdges(v);
                for (const auto &e: eligibleIncEdgesToHead) {
                    if (e == edgeToAvoid)
                        continue;
                    const auto tarTailLatLng = targetGraph.latLng(targetGraph.edgeTail(e));
                    const auto dist = srcTailLatLng.getGreatCircleDistanceTo(tarTailLatLng);
                    if (dist < bestTailDist) {
                        bestTailDist = dist;
                        bestTarEdge = e;
                    }
                }
                return bestTarEdge;
            };

            int tarEdge = pickBestEligibleIncEdge(v);
            if (tarEdge != INVALID_EDGE)
                return tarEdge;

            // If the only edge incident to head was edgeToAvoid, then we need to consider the incoming edges of a
            // different vertex. We find a new vertex close and pick its best incoming edge.
            int newV = closeEligibleVertexChooser.findOtherVertex(v);
            tarEdge = pickBestEligibleIncEdge(newV);
            assert(tarEdge != edgeToAvoid && tarEdge != INVALID_EDGE);
            return tarEdge;
        }


    private:

        // Gets all eligible incident edges in target graph.
        std::vector<int> getEligibleIncomingEdges(const int v) const {
            std::vector<int> eligibleEdges;

            // Eligible incoming edges
            FORALL_INCIDENT_EDGES(revTargetGraph, v, e) {
                const auto edgeId = revTargetGraph.edgeId(e);
                if (isEdgeEligible(edgeId))
                    eligibleEdges.push_back(edgeId);
            }
            return eligibleEdges;
        }

        const GraphT &sourceGraph;
        const GraphT &targetGraph;
        const GraphT& revTargetGraph;
        const IsEdgeEligibleT &isEdgeEligible;
        CloseEligibleVertexChooserT &closeEligibleVertexChooser;

    };


}
