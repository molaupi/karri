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

#include <vector>
#include "Algorithms/Buckets/SortedBucketContainer.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "DataStructures/Containers/Subset.h"

namespace karri {

    // Maintains bucket entries for vehicles that are currently repositioning. For a repositioning vehicle, the
    // bucket entry at a vertex v stores the minimum upward distance from any vertex on the vehicle's repositioning
    // path to v. Thus, a BCH search from a pickup yields lower bounds on the distance from the current location of a
    // repositioning vehicle to the pickup.
    // Bucket entries must only be generated or removed while no searches are run on the buckets.
    template<typename InputGraphT, typename CHEnvT>
    class RepositioningBucketsEnvironment {

        struct CompareEntries {
            bool operator()(const BucketEntry &e1, const BucketEntry &e2) const {
                return e1.distToTarget < e2.distToTarget;
            }
        };

    public:

        using BucketContainer = SortedBucketContainer<BucketEntry, CompareEntries>;

    private:

        struct GenerateRepositioningEntry {
            explicit GenerateRepositioningEntry(BucketContainer &bucketContainer, int &curVehId)
                    : bucketContainer(bucketContainer), curVehId(curVehId) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                // The multi-root search settles v with the minimum upward distance from any root to v.
                bucketContainer.insert(v, BucketEntry(curVehId, distToV[0]));
                return false;
            }

            BucketContainer &bucketContainer;
            int &curVehId;
        };

        using GenSearch = typename CHEnvT::template TopologicalUpwardSearch<GenerateRepositioningEntry>;

    public:

        RepositioningBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv)
                : inputGraph(inputGraph), ch(chEnv.getCH()), searchGraph(ch.upwardGraph()),
                  bucketContainer(searchGraph.numVertices()), curVehId(INVALID_ID),
                  genSearch(chEnv.getForwardTopologicalSearch(GenerateRepositioningEntry(bucketContainer, curVehId))),
                  deleteSearchSpace(searchGraph.numVertices()) {}

        const BucketContainer &getBuckets() const {
            return bucketContainer;
        }

        // Generate bucket entries for a vehicle along the given path (vector of vertices).
        void generateRepositioningBucketEntries(const Vehicle &veh, const std::vector<int> &path) {
            curVehId = veh.vehicleId;

            roots.clear();
            for (const int vertex: path)
                roots.push_back(ch.rank(vertex));
            genSearch.runWithMultipleRoots(roots);
        }

        // Remove bucket entries for a vehicle for the union of the upward search spaces of all vertices on the path.
        void removeRepositioningBucketEntries(const Vehicle &veh, const std::vector<int> &path) {
            int64_t numEntriesVisited = 0;

            deleteSearchSpace.clear();
            for (const int vertex: path)
                deleteSearchSpace.insert(ch.rank(vertex));

            // Traverse the whole upward search space (we cannot prune at vertices without an entry since the search
            // used for generating the entries may use stall-on-demand).
            for (auto iter = deleteSearchSpace.begin(); iter < deleteSearchSpace.end(); ++iter) {
                const auto v = *iter;
                bucketContainer.remove(v, veh.vehicleId, numEntriesVisited);
                FORALL_INCIDENT_EDGES(searchGraph, v, e) {
                    deleteSearchSpace.insert(searchGraph.edgeHead(e));
                }
            }
        }

    private:

        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;

        BucketContainer bucketContainer;

        int curVehId;
        GenSearch genSearch;
        std::vector<int> roots;

        Subset deleteSearchSpace;
    };


    // No-op facility that satisfies interface for generating and removing repositioning bucket entries (though the
    // methods should never be called if repositioning is disabled).
    class NoOpRepositioningBucketsEnvironment {

    public:

        struct BucketContainer {
        };

        const BucketContainer &getBuckets() const {
            return buckets;
        }

        void generateRepositioningBucketEntries(const Vehicle &, const std::vector<int> &) {}

        void removeRepositioningBucketEntries(const Vehicle &, const std::vector<int> &) {}

    private:
        BucketContainer buckets;
    };

} // namespace karri
