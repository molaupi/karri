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

#include <string>
#include "Tools/Logging/NullLogger.h"
#include "Constants.h"
#include "DataStructures/Containers/Subset.h"
#include "DataStructures/Containers/TimestampedVector.h"

template<typename LoggerT = NullLogger>
class BundledBCHSearchTracker {
public:

    BundledBCHSearchTracker(const int numEdges, const int numBuckets, LoggerT &logger)
            : searchSpace(numEdges),
              minEffectiveness(numEdges, INFTY),
              distAtMinEffectiveness(numEdges, INFTY),
              numBuckets(numBuckets),
              logger(logger) {}

    void clear() {
        searchSpace.clear();
        minEffectiveness.clear();
        distAtMinEffectiveness.clear();
    }

    template<typename DistanceLabelT>
    void registerEdgeRelaxation(const int e, const DistanceLabelT &distToTail) {
        searchSpace.insert(e);
        const auto reachedTail = distToTail < DistanceLabelT(INFTY);
        const auto effectiveness = countSet(reachedTail);
        if (effectiveness < minEffectiveness[e]) {
            minEffectiveness[e] = effectiveness;
            distAtMinEffectiveness[e] = distToTail.horizontalMin();
        }
    }

    void writeOutputForSearch(const std::string &searchId) {

        logger << searchId;

        std::vector<int> edges = searchSpace.extractElementsAndClear();
        std::sort(edges.begin(), edges.end(), [&](const int &e1, const int &e2) {
            return distAtMinEffectiveness[e1] < distAtMinEffectiveness[e2];
        });


//        int curBucket = 0;

        const auto bucketWidth = edges.size() / numBuckets;
        for (int curBucket = 0; curBucket < numBuckets; ++curBucket) {
            int sumOfEffInBucket = 0;
            for (int i = curBucket * bucketWidth; i < edges.size() && i < (curBucket + 1) * bucketWidth; ++i)
                sumOfEffInBucket += minEffectiveness[edges[i]];
            const auto numRelaxationsInBucket = std::min(bucketWidth, edges.size() - curBucket * bucketWidth);
            const double avgEffInBucket = (double) sumOfEffInBucket / (double) numRelaxationsInBucket;
//            const auto minDistInBucket = curBucket * bucketWidth < edges.size()? distAtMinEffectiveness[edges[curBucket * bucketWidth]] : 0;
//            logger << ",(" << avgEffInBucket << "|" << minDistInBucket << ")";
            logger << "," << avgEffInBucket;
        }
        logger << "\n";

        // Full output with edge ids:
//        logger << searchId << ",";
//        for (auto it = searchSpace.begin(); it != searchSpace.end(); ++it) {
//            if (it != searchSpace.begin()) logger << ":";
//            const auto &e = *it;
//            logger << "(" << e << "|" << distAtMinEffectiveness[e] << "|" << minEffectiveness[e] << ")";
//        }
//        logger << "\n";
    }


private:

    // Contains all edges that have been relaxed during a bundled search.
    Subset searchSpace;

    // Let the effectiveness of a bundled edge relaxation of edge e=(v,w) denote the number of searches that the
    // relaxation advanced (i.e. the number of searches that reached v before relaxing e).
    // The possible values range from 1 to K.
    // For each edge e, this stores the smallest effectiveness among all relaxations of e performed during a bundled
    // search.
    TimestampedVector<int> minEffectiveness;

    // For each edge e=(v,w), this stores the smallest distance from any of the K sources to v at the time of the
    // relaxation of e with the smallest effectiveness.
    TimestampedVector<int> distAtMinEffectiveness;


    const int numBuckets;
    LoggerT &logger;
};

template<typename LoggerT = NullLogger>
class NoOpBundledBCHSearchTracker {

public:
    NoOpBundledBCHSearchTracker(const int, LoggerT &) {}

    void clear() {}

    template<typename DistanceLabelContainerT>
    void operator()(const int, const int, const int, const DistanceLabelContainerT &) {
    }

    void writeOutputForSearch(const std::string &) {}

};