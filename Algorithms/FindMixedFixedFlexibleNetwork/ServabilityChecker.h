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
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "FixedLine.h"
#include "Request.h"

namespace mixfix {


    template<typename VehInputGraphT,
            typename PsgInputGraphT,
            typename WeightT = TravelTimeAttribute>
    class ServabilityChecker {


        struct StopWhenLineHitOrRadiusExceeded {
            StopWhenLineHitOrRadiusExceeded(const int radius, const PsgInputGraphT &graph, const FixedLine &line,
                                            bool &lineHit)
                    : radius(radius), graph(graph), line(line), lineHit(lineHit) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                if (distToV[0] > radius) {
                    lineHit = false;
                    return true;
                }

                FORALL_INCIDENT_EDGES(graph, v, e) {
                    if (distToV[0] + graph.template get<WeightT>(e) > radius)
                        continue;
                    const auto& eInForwVehGraph = graph.toCarEdge(e);
                    if (eInForwVehGraph == INVALID_EDGE)
                        continue;
                    if (contains(line.begin(), line.end(), eInForwVehGraph)) {
                        lineHit = true;
                        return true;
                    }
                }

                return false;
            }

        private:
            const int radius;
            const PsgInputGraphT &graph;
            const FixedLine &line;
            bool &lineHit;
        };

        using Search = Dijkstra<PsgInputGraphT, WeightT, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, StopWhenLineHitOrRadiusExceeded>;

    public:

        bool canLineServeRider(const FixedLine &line, const Request &req) {
            // todo: implement
        }


    private:

        Search pickupSearch;
        Search dropoffSearch;


    };
} // end namespace