#pragma once
#include "BucketEntryWithLeeway.h"

namespace karri {
    using EllipticBucketEntry = BucketEntryWithLeeway;

    struct DoesEntryHaveLargerRemainingLeeway {
        bool operator()(const EllipticBucketEntry &e1, const EllipticBucketEntry &e2) const {
            return e1.leeway - e1.distToTarget > e2.leeway - e2.distToTarget;
        }
    };
}
