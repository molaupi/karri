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

#include "Tools/Constants.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"
#include <vector>

namespace mixfix {

    template<typename InputGraphT,
            typename VehChEnvT>
    class SPPreliminaryPaths {
    public:

        struct Path {
            using EdgeIt = typename std::vector<int>::const_iterator;

            EdgeIt begin() const {
                return edges.cbegin();
            }

            EdgeIt end() const {
                return edges.cend();
            }

            int& operator[](const int idx) {
                return edges[idx];
            }

            const int& operator[](const int idx) const {
                return edges[idx];
            }

            int& front() {
                return edges.front();
            }

            const int& front() const {
                return edges.front();
            }

            int& back() {
                return edges.back();
            }

            const int& back() const {
                return edges.back();
            }

            int size() const {
                return edges.size();
            }

            const int& getRequestId() const {
                return requestId;
            }

        private:
            friend SPPreliminaryPaths;

            Path(const int requestId) : requestId(requestId), edges() {}

            int requestId;
            std::vector<int> edges;
        };

    private:
        using Paths = std::vector<Path>;
        using PathIt = typename Paths::const_iterator;
        using Query = typename VehChEnvT::template FullCHQuery<>;

    public:

        SPPreliminaryPaths(const InputGraphT &inputGraph, const VehChEnvT &vehChEnv)
                : inputGraph(inputGraph),
                  ch(vehChEnv.getCH()),
                  query(vehChEnv.template getFullCHQuery<>()),
                  pathUnpacker(ch) {}

        void findPathsAndDirectDists(std::vector<Request> &requests) {

            std::fill(reqIdToPathIdx.begin(), reqIdToPathIdx.end(), INVALID_INDEX);
            reqIdToPathIdx.resize(requests.size(), INVALID_INDEX);
            paths.clear();

            for (auto &req: requests) {
                const int srcRank = ch.rank(inputGraph.edgeHead(req.origin));
                const int destRank = ch.rank(inputGraph.edgeTail(req.destination));
                const int offset = inputGraph.travelTime(req.destination);
                query.run(srcRank, destRank);

                // TODO: Finding the direct distances should technically be separate from finding preliminary
                //  paths. If we use a different method for finding preliminary paths, we need to separate them.
                req.directDist = query.getDistance() + offset;

                reqIdToPathIdx[req.requestId] = paths.size();
                Path newPath(req.requestId);
                pathUnpacker.unpackUpDownPath(query.getUpEdgePath(), query.getDownEdgePath(), newPath.edges);
                newPath.edges.push_back(req.destination);
                paths.push_back(std::move(newPath));
            }

        }

        int numPaths() const {
            return paths.size();
        }

        bool empty() const {
            return paths.empty();
        }

        bool hasPathFor(const int reqId) const {
            return reqIdToPathIdx[reqId] != INVALID_INDEX;
        }

        const Path& getPathFor(const int reqId) const {
            KASSERT(hasPathFor(reqId),
                    "Request " << reqId << " does not have a path.", kassert::assert::light);
            return paths[reqIdToPathIdx[reqId]];
        }

        void removePathForRequest(const int requestId) {
            const int idx = reqIdToPathIdx[requestId];
            KASSERT(hasPathFor(requestId),
                    "Request " << requestId << " already does not have a path.", kassert::assert::light);

            std::swap(paths[idx], paths.back());
            paths.pop_back();

            reqIdToPathIdx[requestId] = INVALID_INDEX;
            reqIdToPathIdx[paths[idx].requestId] = idx;
        }

        PathIt begin() const {
            return paths.cbegin();
        }

        PathIt end() const {
            return paths.cend();
        }

    private:

        const InputGraphT &inputGraph;
        const CH &ch;
        Query query;
        CHPathUnpacker pathUnpacker;

        std::vector<int> reqIdToPathIdx;
        std::vector<Path> paths;

    };


} // end namespace