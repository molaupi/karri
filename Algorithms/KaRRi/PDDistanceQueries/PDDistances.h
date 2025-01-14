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

#include "Tools/Simd/AlignedVector.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {

// Representation of PD-distances, i.e. distances from pickups to dropoffs.
    template<typename LabelSetT>
    struct PDDistances {
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        static constexpr int K = LabelSetT::K;


        PDDistances(const RequestState &requestState) : requestState(requestState) {}

        void clear() {
            minCost = INFTY;
            minTravelTime = INFTY;
            const int numLabelsPerDropoff = (requestState.numPickups() / K + (requestState.numPickups() % K != 0));
            const int numNeededLabels = numLabelsPerDropoff * requestState.numDropoffs();
            costs.clear();
            travelTimes.clear();
            costs.resize(numNeededLabels, DistanceLabel(INFTY));
            travelTimes.resize(numNeededLabels, DistanceLabel(INFTY));


            // minCostPerPickup has one entry per pickup. Initialize to INFTY.
            minCostPerPickup.clear();
            minCostPerPickup.resize(numLabelsPerDropoff, DistanceLabel(INFTY));
        }

        // Get total traversal cost of shortest path between pickup and dropoff according to traversal costs.
        // IDs refer to the indices in the vectors of pickups/dropoffs given at the last initialize() call.
        int getCost(const unsigned int pickupId, const unsigned int dropoffId) const {
            assert(pickupId < requestState.numPickups());
            assert(dropoffId < requestState.numDropoffs());
            const int res = costLabelFor(pickupId, dropoffId)[pickupId % K];
            assert(res < INFTY);
            return res;
        }

        // Get total traversal cost of shortest path between pickup and dropoff according to traversal costs.
        int getCost(const PDLoc &pickup, const PDLoc &dropoff) const {
            return getCost(pickup.id, dropoff.id);
        }

        // Get total travel time of shortest path between pickup and dropoff according to traversal costs.
        int getTravelTime(const unsigned int pickupId, const unsigned int dropoffId) const {
            assert(pickupId < requestState.numPickups());
            assert(dropoffId < requestState.numDropoffs());
            const int res = travelTimeLabelFor(pickupId, dropoffId)[pickupId % K];
            assert(res < INFTY);
            return res;
        }

        // Get total travel time of shortest path between pickup and dropoff according to traversal costs.
        int getTravelTime(const PDLoc &pickup, const PDLoc &dropoff) const {
            return getCost(pickup.id, dropoff.id);
        }

        const DistanceLabel &getCostsForBatchOfPickups(const unsigned int firstPickupIdInBatch,
                                                                 const unsigned int &dropoffId) const {
            return costLabelFor(firstPickupIdInBatch, dropoffId);
        }

        const DistanceLabel &getTravelTimesForBatchOfPickups(const unsigned int firstPickupIdInBatch,
                                                       const unsigned int &dropoffId) const {
            return travelTimeLabelFor(firstPickupIdInBatch, dropoffId);
        }

        const int &getMinCost() const {
            return minCost;
        }

        const int &getMinTravelTime() const {
            return minTravelTime;
        }

        int getMinCostForPickup(const int pickupId) const {
            return minCostPerPickup[pickupId / K][pickupId % K];
        }

        // Compares a current PD-distance to the given distance and updates the distances if the given distances are smaller.
        void updateDistanceIfSmaller(const unsigned int pickupId, const unsigned int dropoffId, const int cost, const int travelTime) {
            const auto offsetInBatch = pickupId % K;
            auto &label = costLabelFor(pickupId, dropoffId);
            if (cost < label[offsetInBatch]) {
                label[offsetInBatch] = cost;
                travelTimeLabelFor(pickupId, dropoffId)[offsetInBatch] = travelTime;
                minCost = std::min(minCost, cost);
                minTravelTime = std::min(minTravelTime, travelTimeLabelFor(pickupId, dropoffId)[offsetInBatch]);

                if (cost < minCostPerPickup[pickupId / K][offsetInBatch]) {
                    minCostPerPickup[pickupId / K][offsetInBatch] = cost;
                }
                assert(minDirectDist <= minCostPerPickup[pickupId / K].horizontalMin());
            }
        }

        // Compares the current PD-distances for a batch of pickups to one dropoff to the given distances and updates the
        // ones for which the given distances are smaller.
        // Batch consists of K subsequent pickup IDs and is defined by the first ID in the batch.
        void updateDistanceBatchIfSmaller(const unsigned int firstPickupId, const unsigned int dropoffId,
                                          const DistanceLabel &cost, const DistanceLabel& travelTime) {
            auto &label = costLabelFor(firstPickupId, dropoffId);
            const auto smaller = cost < label;
            if (anySet(smaller)) {
                label.setIf(cost, smaller);
                travelTimeLabelFor(firstPickupId, dropoffId).setIf(travelTime, smaller);
                minCost = std::min(minCost, cost.horizontalMin());
                minTravelTime = std::min(minTravelTime, travelTimeLabelFor(firstPickupId, dropoffId).horizontalMin());
                minCostPerPickup[firstPickupId / K].min(cost);
                assert(minDirectDist <= minCostPerPickup[firstPickupId / K].horizontalMin());
            }
        }


    private:

        inline int idxFor(const unsigned int pickupId, const unsigned int dropoffId) const {
            return (pickupId / K) * requestState.numDropoffs() + dropoffId;
        }

        DistanceLabel &costLabelFor(const unsigned int pickupId, const unsigned int dropoffId) {
            return costs[idxFor(pickupId, dropoffId)];
        }

        const DistanceLabel &costLabelFor(const unsigned int pickupId, const unsigned int dropoffId) const {
            return costs[idxFor(pickupId, dropoffId)];
        }

        DistanceLabel &travelTimeLabelFor(const unsigned int pickupId, const unsigned int dropoffId) {
            return travelTimes[idxFor(pickupId, dropoffId)];
        }

        const DistanceLabel &travelTimeLabelFor(const unsigned int pickupId, const unsigned int dropoffId) const {
            return travelTimes[idxFor(pickupId, dropoffId)];
        }

        const RequestState &requestState;

        // Distances are stored as vectors of size K (DistanceLabel).
        // ceil(numPickups / K) labels per dropoff, sequentially arranged by increasing dropoffId.
        AlignedVector<DistanceLabel> costs; // Total traversal costs of shortest paths according to traversal costs.
        AlignedVector<DistanceLabel> travelTimes; // Total travel times of shortest paths according to traversal costs.
        int minCost;
        int minTravelTime;
        AlignedVector<DistanceLabel> minCostPerPickup;
    };
}