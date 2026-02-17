#pragma once
#include "Algorithms/Buckets/BucketEntry.h"

namespace karri {
    // .targetId is vehicle ID
    // .distToTarget is dist from last stop to vertex for idle vehicles or arrival time at vertex for non-idle vehicles
    using LastStopBucketEntry  = BucketEntry;

    struct CompareLastStopBucketEntries {
        bool operator()(const LastStopBucketEntry &e1, const LastStopBucketEntry &e2) const {
            return e1.distToTarget < e2.distToTarget;
        }
    };
}
