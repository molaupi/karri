/// *******************************************************************************
/// MIT License
///
/// Copyright (c) 2026 Copied/Adapted
///
/// *******************************************************************************

#pragma once

#include <type_traits>
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Tools/Timer.h"
#include "DataStructures/Containers/Subset.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"

namespace karri {

    template<typename InputGraphT, typename CHEnvT>
    class RepositioningBucketsEnvironment {

        using RepositionEntry = BucketEntry;

        struct CompareEntries {
            bool operator()(const RepositionEntry &e1, const RepositionEntry &e2) const {
                return e1.distToTarget < e2.distToTarget;
            }
        };

    public:
        static constexpr bool SORTED = true;

        using BucketContainer = SortedBucketContainer<RepositionEntry, CompareEntries>;

    private:

        struct GenerateRepositionEntry {
            explicit GenerateRepositionEntry(BucketContainer &bucketContainer, int &curVehId, int &verticesVisited)
                    : bucketContainer(bucketContainer), curVehId(curVehId), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                // The topological multi-root search guarantees that the first time v is settled
                // we have the minimal upward distance from any root on the path to v. Just insert it.
                const int newDist = distToV[0];
                auto entry = RepositionEntry(curVehId, newDist);
                bucketContainer.insert(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &bucketContainer;
            int &curVehId;
            int &verticesVisited;
        };

    public:

        RepositioningBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv,
                                        karri::stats::UpdatePerformanceStats &stats)
            : inputGraph(inputGraph), ch(chEnv.getCH()), searchGraph(ch.upwardGraph()),
              bucketContainer(searchGraph.numVertices()), vehicleId(INVALID_ID), verticesVisitedInSearch(0),
              entriesVisitedInSearch(0),
              genSearch(chEnv.getForwardTopologicalSearch(
                  GenerateRepositionEntry(bucketContainer, vehicleId, verticesVisitedInSearch))),
              deleteSearchSpace(searchGraph.numVertices()),
              stats(stats) {
        }

        const BucketContainer &getBuckets() const {
            return bucketContainer;
        }

        // Generate bucket entries for a vehicle along the given path (vector of vertices).
        // For each vertex settled by the upward topological search(s), create/update a bucket entry
        // that stores the minimal upward distance from any vertex on the path to that settled vertex.
        void generateRepositioningBucketEntries(const Vehicle &veh, const std::vector<int> &path) {
            Timer timer;
            vehicleId = veh.vehicleId;
            verticesVisitedInSearch = 0;

            // Run a single topological upward search rooted at all vertices in the path. Initialize
            // a vector of root ranks and run the multi-root topological search so each vertex is
            // settled at most once.
            std::vector<int> roots;
            roots.reserve(path.size());
            for (const int vertex : path)
                roots.push_back(ch.rank(vertex));

            genSearch.runWithMultipleRoots(roots);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsGenerateEntriesTime += time;
        }

        // Remove bucket entries for a vehicle for the union of upward search spaces of all vertices on path.
        // We perform a BFS-like traversal over the upward graph starting from all path roots and remove entries.
        void removeRepositioningBucketEntries(const Vehicle &veh, const std::vector<int> &path) {
            Timer timer;
            vehicleId = veh.vehicleId;
            verticesVisitedInSearch = 0;
            entriesVisitedInSearch = 0;

            deleteSearchSpace.clear();
            for (const int vertex : path)
                deleteSearchSpace.insert(ch.rank(vertex));

            for (auto iter = deleteSearchSpace.begin(); iter < deleteSearchSpace.end(); ++iter) {
                const auto v = *iter;
                if (bucketContainer.remove(v, vehicleId)) {
                    FORALL_INCIDENT_EDGES(searchGraph, v, e) {
                        const auto w = searchGraph.edgeHead(e);
                        deleteSearchSpace.insert(w);
                    }
                }
                ++verticesVisitedInSearch;
                entriesVisitedInSearch += bucketContainer.getNumEntriesVisitedInLastUpdateOrRemove();
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.lastStopBucketsDeleteEntriesTime += time;
        }

    private:

        using GenSearch = typename CHEnvT::template TopologicalUpwardSearch<GenerateRepositionEntry>;

        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;

        BucketContainer bucketContainer;

        int vehicleId;
        int verticesVisitedInSearch;
        int entriesVisitedInSearch;

        GenSearch genSearch;

        Subset deleteSearchSpace;

        karri::stats::UpdatePerformanceStats &stats;
    };

} // namespace karri
