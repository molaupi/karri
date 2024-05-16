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

#include "DataStructures/Containers/FastResetFlagArray.h"

namespace karri {

// Finds the vehicle distances between the origin/destination and pickups/dropoffs using a Dijkstra search on
// the vehicle graph.
    template<typename VehGraphT, typename WeightT = TravelTimeAttribute>
    class VehicleToPDLocQuery {

        struct StopWhenAllFound {

            explicit StopWhenAllFound(const VehicleToPDLocQuery &query) : query(query) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &, const DistLabelContT & /* distanceLabels */) {
                return query.numVerticesToFind == 0;
            }

            const VehicleToPDLocQuery &query;
        };

        struct CheckForPDLoc {

            explicit CheckForPDLoc(VehicleToPDLocQuery &query) : query(query) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &, const DistLabelContT & /* distanceLabels */) {
                if (query.vertexHasPDLoc.isSet(v))
                    --query.numVerticesToFind;
                return false;
            }

            VehicleToPDLocQuery &query;
        };

        using LabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;
        using Search = Dijkstra<VehGraphT, WeightT, LabelSet, StopWhenAllFound, CheckForPDLoc>;

    public:

        VehicleToPDLocQuery(const VehGraphT &graph, const VehGraphT &revGraph)
                : forwardGraph(graph),
                  forwardSearch(graph, StopWhenAllFound(*this), CheckForPDLoc(*this)),
                  reverseSearch(revGraph, StopWhenAllFound(*this), CheckForPDLoc(*this)),
                  vertexHasPDLoc(graph.numVertices()), runTime(0) {}

        // Takes a vector of PDLoc and a center point and finds the vehicle distances from the center to
        // every PD loc. Stores the found distances in the vehDistFromCenter field of each PD loc.
        template<typename VectorT>
        void runForward(const int center, VectorT &pdLocs) {

            Timer timer;

            const auto s = forwardGraph.edgeHead(center);

            vertexHasPDLoc.reset();
            numVerticesToFind = 0;

            for (int i = 0; i < pdLocs.size(); ++i) {
                const auto &pdLoc = pdLocs[i];
                const auto vertexOfSpot = forwardGraph.edgeTail(pdLoc.fullVehLoc);
                if (!vertexHasPDLoc.isSet(vertexOfSpot))
                    ++numVerticesToFind;
                vertexHasPDLoc.set(vertexOfSpot);
            }

            forwardSearch.run(s);

            for (int i = 0; i < pdLocs.size(); ++i) {
                auto &pdLoc = pdLocs[i];
                const auto vertexOfSpot = forwardGraph.edgeTail(pdLoc.fullVehLoc);
                pdLoc.vehDistFromCenter =
                        forwardSearch.getDistance(vertexOfSpot) + forwardGraph.travelTime(pdLoc.fullVehLoc);
                assert(pdLoc.vehDistFromCenter < INFTY);
            }


            runTime = timer.elapsed<std::chrono::nanoseconds>();
        }

        // Takes a vector of PDLoc and a center point and finds the vehicle distances from every PD loc
        // to the center. Stores the found distances in the vehDistToCenter field of each PD loc.
        template<typename VectorT>
        void runReverse(const int center, VectorT &pdLocs) {

            Timer timer;

            const auto t = forwardGraph.edgeTail(center);
            const auto offset = forwardGraph.travelTime(center);

            vertexHasPDLoc.reset();
            numVerticesToFind = 0;

            for (int i = 0; i < pdLocs.size(); ++i) {
                const auto &pdLoc = pdLocs[i];
                const auto vertexOfSpot = forwardGraph.edgeHead(pdLoc.fullVehLoc);
                if (!vertexHasPDLoc.isSet(vertexOfSpot))
                    ++numVerticesToFind;
                vertexHasPDLoc.set(vertexOfSpot);
            }

            reverseSearch.runWithOffset(t, offset);

            for (int i = 0; i < pdLocs.size(); ++i) {
                auto &pdLoc = pdLocs[i];
                const auto vertexOfSpot = forwardGraph.edgeHead(pdLoc.fullVehLoc);
                pdLoc.vehDistToCenter = reverseSearch.getDistance(vertexOfSpot);
                assert(pdLoc.vehDistToCenter < INFTY);
            }

            runTime = timer.elapsed<std::chrono::nanoseconds>();
        }

        const int64_t &getRunTime() const {
            return runTime;
        }

        int getNumEdgeRelaxationsInLastForwardRun() const {
            return forwardSearch.getNumEdgeRelaxations();
        }

        int getNumEdgeRelaxationsInLastReverseRun() const {
            return reverseSearch.getNumEdgeRelaxations();
        }

        int getNumVerticesSettledInLastForwardRun() const {
            return forwardSearch.getNumVerticesSettled();
        }

        int getNumVerticesSettledInLastReverseRun() const {
            return reverseSearch.getNumVerticesSettled();
        }


    private:

        const VehGraphT &forwardGraph;
        Search forwardSearch;
        Search reverseSearch;

        int numVerticesToFind;
        FastResetFlagArray<> vertexHasPDLoc;

        int64_t runTime;
    };
}