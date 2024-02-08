#pragma once

enum LastStopBucketsSortingType {
    FULL, // Separate idle and non-idle vehicles
    ONLY_DIST, // Sort all entries only by distances
    UNSORTED // Do not sort buckets
};