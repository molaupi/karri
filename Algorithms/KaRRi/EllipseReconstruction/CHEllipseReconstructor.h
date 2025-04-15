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
#include "CHEllipseReconstructorQuery.h"

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

namespace karri {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using bucket entries and a CH topological downward search.
    template<typename CHEnvT, typename EllipticBucketsEnvironmentT, typename WeightT = TraversalCostAttribute,
            typename LoggerT = NullLogger>
    class CHEllipseReconstructor {

        using LabelSet = SimdLabelSet<3, ParentInfo::NO_PARENT_INFO>;
//        using LabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using DistanceLabel = typename LabelSet::DistanceLabel;
        using LabelMask = typename LabelSet::LabelMask;
        static constexpr int K = LabelSet::K;

        using Query = CHEllipseReconstructorQuery<EllipticBucketsEnvironmentT, LabelSet, WeightT>;

        struct QueryStats {
            int numVerticesSettled = 0;
            int numEdgesRelaxed = 0;
            int64_t initTime = 0;
            int64_t topoSearchTime = 0;
            int64_t postprocessTime = 0;
        };
    public:

        CHEllipseReconstructor(const CHEnvT &chEnv, const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                               const RouteState &routeState)
                  : ch(chEnv.getCH()),
                  downGraph(chEnv.getCH().downwardGraph()),
                  upGraph(chEnv.getCH().upwardGraph()),
                  topDownRankPermutation(chEnv.getCH().downwardGraph().numVertices()),
                  query([&](){return Query(ch, downGraph, upGraph, topDownRankPermutation,
                        ellipticBucketsEnv, routeState);}),
                  logger(LogManager<LoggerT>::getLogger("ch_ellipse_reconstruction.csv",
                                                        "num_ellipses,"
                                                        "init_time,"
                                                        "topo_search_time,"
                                                        "postprocess_time,"
                                                        "total_time,"
                                                        "topo_search_num_vertices_settled,"
                                                        "topo_search_num_edges_relaxed\n")) {
            KASSERT(downGraph.numVertices() == upGraph.numVertices());
            const int numVertices = downGraph.numVertices();
            for (int r = 0; r < numVertices; ++r)
                topDownRankPermutation[r] = numVertices - r - 1;

            downGraph.permuteVertices(topDownRankPermutation);
            upGraph.permuteVertices(topDownRankPermutation);
        }

        std::vector<std::vector<VertexInEllipse>>
        getVerticesInEllipsesOfLegsAfterStops(const std::vector<int> &stopIds, int &totalNumVerticesSettled,
                                              int &totalNumEdgesRelaxed) {

            totalNumVerticesSettled = 0;
            totalNumEdgesRelaxed = 0;
            if (stopIds.empty())
                return {};

            Timer timer;

            const size_t numEllipses = stopIds.size();

            const size_t numBatches = numEllipses / K + (numEllipses % K != 0);

            std::vector<std::vector<VertexInEllipse>> ellipses;
            ellipses.resize(numEllipses);
            tbb::parallel_for(0ul, numBatches, [&](size_t i) {
                auto& localQuery = query.local();
                auto& localStats = queryStats.local();
                std::vector<int> batchStopIds;
                for (int j = 0; j < K && i * K + j < numEllipses; ++j) {
                    batchStopIds.push_back(stopIds[i * K + j]);
                }
                auto batchResult = localQuery.run(batchStopIds, localStats.numVerticesSettled, localStats.numEdgesRelaxed, localStats.initTime,
                                                  localStats.topoSearchTime, localStats.postprocessTime);
                for (int j = 0; j < K && i * K + j < numEllipses; ++j) {
                    ellipses[i * K + j].swap(batchResult[j]);
                }
            });

            int64_t totalInitTime = 0;
            int64_t totalTopoSearchTime = 0;
            int64_t totalPostprocessTime = 0;
            for (auto& localStats : queryStats) {
                totalNumVerticesSettled += localStats.numVerticesSettled;
                totalNumEdgesRelaxed += localStats.numEdgesRelaxed;
                totalInitTime += localStats.initTime;
                totalTopoSearchTime += localStats.topoSearchTime;
                totalPostprocessTime += localStats.postprocessTime;
                localStats.numVerticesSettled = 0;
                localStats.numEdgesRelaxed = 0;
                localStats.initTime = 0;
                localStats.topoSearchTime = 0;
                localStats.postprocessTime = 0;
            }

            const auto totalTime = timer.elapsed<std::chrono::nanoseconds>();

            logger << numEllipses << "," << totalInitTime << "," << totalTopoSearchTime << ","
                   << totalPostprocessTime << "," << totalTime << "," << totalNumVerticesSettled << "," << totalNumEdgesRelaxed << "\n";

            return ellipses;
        }


    private:

        const CH& ch;
        CH::SearchGraph downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        CH::SearchGraph upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.

        Permutation topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.

        tbb::enumerable_thread_specific<QueryStats> queryStats;
        tbb::enumerable_thread_specific<Query> query;

        LoggerT &logger;
    };

} // karri
