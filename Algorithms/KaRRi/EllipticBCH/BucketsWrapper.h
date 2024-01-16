//
// Created by tim on 16.01.24.
//

#pragma once


#include <type_traits>
#include "BucketEntryWithLeeway.h"
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "Algorithms/Buckets/SortedBucketContainer.h"

namespace karri {

    template<bool SORTED_BUCKETS>
    class BucketsWrapper {
        using Entry = BucketEntryWithLeeway;

        struct DoesEntryHaveLargerRemainingLeeway {
            bool operator()(const Entry &e1, const Entry &e2) const {
                return e1.leeway - e1.distToTarget > e2.leeway - e2.distToTarget;
            }
        };

    public:
        static constexpr bool SORTED_BY_REM_LEEWAY = SORTED_BUCKETS;
        using BucketContainer = std::conditional_t<SORTED_BUCKETS,
                SortedBucketContainer<Entry, DoesEntryHaveLargerRemainingLeeway>,
                DynamicBucketContainer<Entry>
        >;


        BucketsWrapper(const int numVertices): sourceBuckets(numVertices), targetBuckets(numVertices) {}

        BucketContainer &getSourceBuckets() {
            return sourceBuckets;
        }

        BucketContainer &getTargetBuckets() {
            return targetBuckets;
        }

    private:

        BucketContainer sourceBuckets;
        BucketContainer  targetBuckets;

    };
}