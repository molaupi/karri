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

#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Labels/ParentInfo.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/CH/CHQuery.h"
#include "Algorithms/CH/CH.h"
#include "Tools/Timer.h"

namespace karri {


// Given a set H of pdDistances locs, for each vehicle veh, this search finds the closest pdDistances loc
// argmin_{h \in H} dist(lastStop(veh), h).
    template<typename InputGraphT, typename CHEnvT, typename LastStopBucketsT, typename PruningCriterionT = dij::NoCriterion, typename LabelSetT = BasicLabelSet<0, ParentInfo::PARENT_VERTICES_ONLY>>
    class ClosestPDLocToLastStopBCHQuery {

        static_assert(LabelSetT::KEEP_PARENT_VERTICES,
                      "ClosestPDLocToLastStopBCHQuery needs parent vertex pointers to propagate id of closest PD loc through search tree.");

        using LabelSet = LabelSetT;
        using PruningCriterion = PruningCriterionT; // Criterion to prune the Dijkstra search that constitutes the BCH.

    public:

        ClosestPDLocToLastStopBCHQuery(const InputGraphT &inputGraph, const int fleetSize,
                                       const CHEnvT &chEnv,
                                       const LastStopBucketsT &lastStopBuckets,
                                       PruningCriterion pruningCriterion = {}) :
                inputGraph(inputGraph),
                lastStopBuckets(lastStopBuckets),
                ch(chEnv.getCH()),
                search(ch.downwardGraph(), {}, pruningCriterion),
                idOfClosestSpotToRank(ch.downwardGraph().numVertices()),
                idOfClosestSpotToVeh(fleetSize),
                distVehToClosestSpot(fleetSize) {}

        template<typename PDLocsT>
        void run(const PDLocsT &pdLocs) {

            Timer timer;

            init(pdLocs);

            // todo stopping criterion and pruning for bucket scans based on max tentative distance across all vehicles
            while (!search.queue.empty()) {
                const auto v = search.settleNextVertex();
                idOfClosestSpotToRank[v] = idOfClosestSpotToRank[search.parent.getVertex(v)];
                scanVehicleBucketAtRank(v);
            }

            runTime = timer.elapsed<std::chrono::nanoseconds>();
        }

        unsigned int getIdOfSpotClosestToVeh(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < idOfClosestSpotToVeh.size());
            return idOfClosestSpotToVeh[vehId];
        }

        // Get distance from last stop of given vehicle to the closest PDLoc as calculated in the last call to run.
        int getDistToClosestPDLocFromVeh(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < distVehToClosestSpot.size());
            return distVehToClosestSpot[vehId];
        }

        int getNumEdgeRelaxations() const {
            return search.getNumEdgeRelaxations();
        }

        int getNumVerticesSettled() const {
            return search.getNumVerticesSettled();
        }

        int getNumEntriesScanned() const {
            return numEntriesScanned;
        }

        int64_t getRunTime() const {
            return runTime;
        }

    private:

        template<typename PDLocsT>
        void init(const PDLocsT &pdLocs) {
            numEntriesScanned = 0;

            search.distanceLabels.init();
            search.queue.clear();
            std::fill(idOfClosestSpotToRank.begin(), idOfClosestSpotToRank.end(), INVALID_ID);
            std::fill(idOfClosestSpotToVeh.begin(), idOfClosestSpotToVeh.end(), INVALID_ID);
            std::fill(distVehToClosestSpot.begin(), distVehToClosestSpot.end(), INFTY);

            for (const auto &pdLoc: pdLocs) {
                const auto tailRank = ch.rank(inputGraph.edgeTail(pdLoc.loc));
                const auto offset = inputGraph.travelTime(pdLoc.loc);

                if (offset < search.distanceLabels[tailRank][0]) {
                    search.distanceLabels[tailRank][0] = offset;
                    idOfClosestSpotToRank[tailRank] = pdLoc.id;

                    search.parent.setVertex(tailRank, tailRank, true);
                    if (!search.queue.contains(tailRank)) {
                        search.queue.insert(tailRank, offset);
                    } else {
                        search.queue.decreaseKey(tailRank, offset);
                    }
                }
            }
        }

        void scanVehicleBucketAtRank(const int v) {
            const auto idOfSpotClosestToV = idOfClosestSpotToRank[v];
            const auto distVToSpot = search.getDistance(v);
            auto bucket = lastStopBuckets.getUnsortedBucketOf(v);
            for (const auto &entry: bucket) {
                ++numEntriesScanned;
                const auto vehId = entry.targetId;
                const auto distToSpotViaV = entry.distToTarget + distVToSpot;
                if (distToSpotViaV < distVehToClosestSpot[vehId]) {
                    idOfClosestSpotToVeh[vehId] = idOfSpotClosestToV;
                    distVehToClosestSpot[vehId] = distToSpotViaV;
                }
            }
        }

        const InputGraphT &inputGraph;
        const LastStopBucketsT &lastStopBuckets;
        const CH &ch;

        using Search = Dijkstra<typename CH::SearchGraph, typename CH::Weight, LabelSet, dij::NoCriterion, PruningCriterion>;
        Search search;

        std::vector<unsigned int> idOfClosestSpotToRank;
        std::vector<unsigned int> idOfClosestSpotToVeh;
        std::vector<int> distVehToClosestSpot;

        int numEntriesScanned;
        int64_t runTime;

    };

    template<typename InputGraphT, typename CHEnvT, typename LastStopBucketsT>
    using ClosestPDLocToLastStopBCHQueryWithStallOnDemand = ClosestPDLocToLastStopBCHQuery<
            InputGraphT,
            CHEnvT,
            LastStopBucketsT,
            typename CHQuery<BasicLabelSet<0, ParentInfo::PARENT_VERTICES_ONLY>>::PruningCriterion
    >;

}