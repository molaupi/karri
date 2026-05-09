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
#include "VertexInEllipse.h"
#include <kassert/kassert.hpp>

namespace karri {
    struct EdgeEllipseContainer {

        const std::vector<EdgeInEllipse> &getEdgesInEllipse(const int stopId) const {
            KASSERT(stopId < static_cast<int>(idxOfStop.size()));
            const auto idx = idxOfStop[stopId];
            // KASSERT(idx != INVALID_INDEX);
            if (idx == INVALID_INDEX)
                return empty;

            return edgeEllipses[idx];
        }

    private:

        EdgeEllipseContainer() = default;

        template<typename, typename, typename, bool, int, typename, typename>
        friend class PHASTEllipseReconstructor;

        template<typename, typename, typename, typename>
        friend class DijkstraEllipseReconstructor;

        friend class OnlyAtStopEllipseReconstructor;

        // Maps a stop ID to an internal index in the vector of stop IDs.
        std::vector<int> idxOfStop;
        std::vector<std::vector<EdgeInEllipse>> edgeEllipses;

        std::vector<EdgeInEllipse> empty;
    };

    struct NoOpEdgeEllipseContainer {
        std::vector<EdgeInEllipse> getEdgesInEllipse(const int) const {
            return {};
        }
    };
}