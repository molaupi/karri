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

#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Containers/FastResetFlagArray.h"
#include "Tools/Logging/LogManager.h"
#include "DataStructures/Queues/AddressableKHeap.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/KaRRi/LastStopSearches/LabelBucketContainer.h"

namespace karri::PickupAfterLastStopStrategies {

    struct PDPairAfterLastStopLabel {
        int pickupId = INVALID_ID;
        int dropoffId = INVALID_ID;
        int directCost = INFTY; // Stored so it only has to be retrieved from direct distances once
        int directTravelTime = INFTY;
        int costToPickup = INFTY;
        int travelTimeToPickup = INFTY;

        friend bool operator==(const PDPairAfterLastStopLabel &label1, const PDPairAfterLastStopLabel &label2) {
            return label1.pickupId == label2.pickupId && label1.dropoffId == label2.dropoffId &&
                   label1.costToPickup == label2.costToPickup;
        }
    };


    // A query which finds the best pair of pickup and (promising) dropoff to insert after the last stop of each vehicle
    // ignoring the service time constraint of the vehicle. If this search finds a PD-pair for a vehicle that holds the
    // service time constraint of that vehicle, then that PD-pair is guaranteed to have the smallest cost for an
    // assignment after the last stop of that vehicle. If the PD-pair found for a vehicle violates the service time
    // constraint, this is an indicator that it is unlikely that a different PD-pair will hold the constraint but not a
    // guarantee. In that case, other PD-pairs need to be evaluated for those vehicles outside the scope of this search.
    template<typename InputGraphT,
            typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename DirectSearchesT,
            typename QueueT = AddressableQuadHeap,
            bool STALL_LABELS = false>
    class MinCostPairAfterLastStopQuery {

        using BucketContainer = LabelBucketContainer<PDPairAfterLastStopLabel>;

        using PDDistanceLabel = typename DirectSearchesT::DistanceLabel;
        using PDLabelMask = typename DirectSearchesT::LabelMask;
        static constexpr int PD_K = DirectSearchesT::K;
        static constexpr int INVALID_DIST = -1;

    public:

        MinCostPairAfterLastStopQuery(const InputGraphT &inputGraph, const Fleet &fleet,
                                      const CHEnvT &chEnv,
                                      const RouteState &routeState, DirectSearchesT &directSearches,
                                      const CostCalculator &calculator,
                                      const LastStopBucketsEnvT &lastStopBucketsEnv,
                                      const RequestState &requestState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  queryGraph(ch.downwardGraph()),
                  oppositeGraph(ch.upwardGraph()),
                  fleet(fleet),
                  routeState(routeState),
                  calculator(calculator),
                  lastStopBuckets(lastStopBucketsEnv.getBuckets()),
                  directSearches(directSearches),
                  requestState(requestState),
                  reverseLabelBuckets(inputGraph.numVertices()),
                  reverseQueue(queryGraph.numVertices()),
                  markedIndices(),
                  upperBoundCostWithConstraints(INFTY),
                  bestCostWithoutConstraints(INFTY),
                  bestAsgn() {}

        void run(const int &bestKnownCost) {

            Timer timer;

            initQueryForRun(bestKnownCost);

            initializationTime = timer.elapsed<std::chrono::nanoseconds>();
            timer.restart();

            int v;
            PDPairAfterLastStopLabel label;
            while (!stopSearch()) {
                const bool unpruned = settleNextLabel(v, label);
                if (unpruned) {
                    ++numLabelsRelaxed;
                    scanVehicleBucket(v, label);
                }
            }

            runTime = timer.elapsed<std::chrono::nanoseconds>() + initializationTime;
        }

        const int &getBestCostWithoutConstraints() const {
            return bestCostWithoutConstraints;
        }

        const Assignment &getBestAssignment() const {
            return bestAsgn;
        }

        const int &getUpperBoundCostWithHardConstraints() const {
            return upperBoundCostWithConstraints;
        }

        int getNumInitialLabelsGenerated() const {
            return numInitialLabelsGenerated;
        }

        int getNumInitialLabelsNotPruned() const {
            return numInitialLabelsNotPruned;
        }

        int getNumLabelsRelaxed() const {
            return numLabelsRelaxed;
        }

        int getNumEntriesScanned() const {
            return numEntriesScanned;
        }

        // Returns how often the search propagated a label along an edge in the last run.
        int getNumEdgeRelaxations() const {
            return numEdgeRelaxations;
        }

        int getNumDominationRelationTests() const {
            return numDominationRelationTests;
        }

        int64_t getInitializationTime() const {
            return initializationTime;
        }

        int64_t getRunTime() const {
            return runTime;
        }

    private:

        inline bool stopSearch() const {

            if (reverseQueue.empty()) return true;

            int v, minCostLowerBound;
            reverseQueue.min(v, minCostLowerBound);
            return minCostLowerBound > bestCostWithoutConstraints;
        }

        void initQueryForRun(const int &bestKnownCost) {
            numLabelsRelaxed = 0;
            numEntriesScanned = 0;
            numInitialLabelsGenerated = 0;
            numInitialLabelsNotPruned = 0;
            numEdgeRelaxations = 0;
            numDominationRelationTests = 0;

            reverseLabelBuckets.clear();
            reverseQueue.clear();

            upperBoundCostWithConstraints = bestKnownCost;
            bestCostWithoutConstraints = bestKnownCost;
            bestAsgn = Assignment();

            // Generate initial labels. Processes pickups batch-wise according to batches used in searches for direct
            // PD-distances.
            const int numPickupBatches = requestState.numPickups() / PD_K + (requestState.numPickups() % PD_K != 0);
            for (int pickupBatchIdx = 0; pickupBatchIdx < numPickupBatches; ++pickupBatchIdx) {

                // For each pickup in batch (using batched operations): Find dropoffs for which an initial label with
                // this pickup and dropoff needs to be created.
                dropoffIdsForInitialLabels.clear();
                directCostsForInitialLabels.clear();
                directTravelTimesForInitialLabels.clear();
                for (const auto &dropoff: requestState.dropoffs) {
                    const auto &directCostBatch = directSearches.getCostsForBatchOfPickups(
                            pickupBatchIdx * PD_K, dropoff.id);
                    const auto &directTravelTimeBatch = directSearches.getTravelTimesForBatchOfPickups(
                            pickupBatchIdx * PD_K, dropoff.id);
                    checkDropoffForInitialLabelWithGivenPickupBatch(dropoff, directCostBatch, directTravelTimeBatch);
                }

                // For each pickup in batch (non-batched): Generate necessary initial labels.
                for (int idxInBatch = 0; idxInBatch < PD_K &&
                                         pickupBatchIdx * PD_K + idxInBatch < requestState.numPickups(); ++idxInBatch) {
                    const auto &pickup = requestState.pickups[pickupBatchIdx * PD_K + idxInBatch];
                    const auto tail = ch.rank(inputGraph.edgeTail(pickup.loc));
                    const auto pickupCostOffset = inputGraph.traversalCost(pickup.loc);
                    const auto pickupTravelTimeOffset = inputGraph.travelTime(pickup.loc);

                    for (int i = 0; i < dropoffIdsForInitialLabels.size(); ++i) {
                        const int directCost = directCostsForInitialLabels[i][idxInBatch];
                        if (directCost == INVALID_DIST)
                            continue;

                        const int directTravelTime = directTravelTimesForInitialLabels[i][idxInBatch];

                        const auto &dropoff = requestState.dropoffs[dropoffIdsForInitialLabels[i]];
                        PDPairAfterLastStopLabel initialLabel = {pickup.id, dropoff.id, directCost, directTravelTime,
                                                                 pickupCostOffset, pickupTravelTimeOffset};
                        ++numInitialLabelsGenerated;

                        // Check for domination between pairs with pickup at the same vertex and insert into the bucket at the
                        // right spot.
                        const auto minCostOfLabel = lowerBoundCostOfLabel(initialLabel);

                        if (minCostOfLabel > upperBoundCostWithConstraints)
                            continue;
                        if (insertLabelAtVertexAndClean(tail, reverseLabelBuckets, initialLabel, minCostOfLabel)) {
                            ++numInitialLabelsNotPruned;
                            if (!reverseQueue.contains(tail)) {
                                reverseQueue.insert(tail, minCostOfLabel);
                            } else {
                                const auto &minCostLabel = reverseLabelBuckets.minOpenLabel(tail);
                                reverseQueue.decreaseKey(tail, lowerBoundCostOfLabel(minCostLabel));
                            }
                        }
                    }
                }
            }
        }

        void checkDropoffForInitialLabelWithGivenPickupBatch(const PDLoc &dropoff,
                                                             const PDDistanceLabel &costsToDropoff,
                                                             const PDDistanceLabel &travelTimesToDropoff) {
            static const PDDistanceLabel INVALID_DIST_LABEL = PDDistanceLabel(INVALID_DIST);
            static const auto inftyLabel = PDDistanceLabel(INFTY);
            PDLabelMask isNewDominated = costsToDropoff >= inftyLabel;
            for (int i = 0; i < dropoffIdsForInitialLabels.size(); ++i) {
                isNewDominated |= batchInitialLabelDominates(dropoffIdsForInitialLabels[i],
                                                             directCostsForInitialLabels[i],
                                                             directTravelTimesForInitialLabels[i],
                                                             dropoff.id,
                                                             costsToDropoff,
                                                             travelTimesToDropoff);
                if (allSet(isNewDominated))
                    return;
            }

            int i = 0;
            while (i < dropoffIdsForInitialLabels.size()) {
                const auto newDominates = batchInitialLabelDominates(dropoff.id, costsToDropoff,
                                                                     travelTimesToDropoff,
                                                                     dropoffIdsForInitialLabels[i],
                                                                     directCostsForInitialLabels[i],
                                                                     directTravelTimesForInitialLabels[i]);
                if (allSet(newDominates)) {
                    // If new label dominates existing label for every pickup in the batch, remove the entire existing
                    // label by swapping it to the end and popping it.
                    std::swap(dropoffIdsForInitialLabels[i], dropoffIdsForInitialLabels.back());
                    dropoffIdsForInitialLabels.pop_back();
                    std::swap(directCostsForInitialLabels[i], directCostsForInitialLabels.back());
                    directCostsForInitialLabels.pop_back();
                    std::swap(directTravelTimesForInitialLabels[i], directTravelTimesForInitialLabels.back());
                    directTravelTimesForInitialLabels.pop_back();
                } else {
                    // Otherwise, set label to invalid values everywhere where new label dominates existing label and
                    // continue with next label.
                    directCostsForInitialLabels[i].setIf(INVALID_DIST_LABEL, newDominates);
                    directTravelTimesForInitialLabels[i].setIf(INVALID_DIST_LABEL, newDominates);
                    ++i;
                }
            }
            dropoffIdsForInitialLabels.push_back(dropoff.id);
            auto costsWhereNotDominated = costsToDropoff;
            costsWhereNotDominated.setIf(INVALID_DIST_LABEL, isNewDominated);
            directCostsForInitialLabels.push_back(costsWhereNotDominated);
            auto travelTimesWhereNotDominated = travelTimesToDropoff;
            travelTimesWhereNotDominated.setIf(INVALID_DIST_LABEL, isNewDominated);
            directTravelTimesForInitialLabels.push_back(travelTimesWhereNotDominated);
        }

        bool initialLabelDominates(const PDPairAfterLastStopLabel &label1, const PDPairAfterLastStopLabel &label2) {
            ++numDominationRelationTests;
            const auto &pickup1 = requestState.pickups[label1.pickupId];
            const auto &dropoff1 = requestState.dropoffs[label1.dropoffId];
            const auto &pickup2 = requestState.pickups[label2.pickupId];
            const auto &dropoff2 = requestState.dropoffs[label2.dropoffId];

            assert(dropoff1.id != dropoff2.id);
            if (pickup1.id != pickup2.id || label1.costToPickup != label2.costToPickup)
                return false;

            // pickup1 == pickup2 and label1.costToPickup == label2.costToPickup => domination only depends on the part
            // after the pickup, i.e. direct distance and walking distance from dropoff:

            using F = CostCalculator::CostFunction;
            const auto maxDetourDiff = label1.directTravelTime - label2.directTravelTime;
            const auto maxTripDiff = maxDetourDiff + dropoff1.walkingDist - dropoff2.walkingDist;
            const auto maxTripVioDiff = F::TRIP_VIO_WEIGHT * std::max(maxTripDiff, 0);

            const auto maxCostDiff = F::VEH_WEIGHT * (label1.directCost - label2.directCost) +
                                     maxTripVioDiff;
            return maxCostDiff < 0;
        }

        PDLabelMask batchInitialLabelDominates(const unsigned int dropoffId1,
                                               const PDDistanceLabel &costsToDropoff1,
                                               const PDDistanceLabel &travelTimesToDropoff1,
                                               const unsigned int dropoffId2,
                                               const PDDistanceLabel &costsToDropoff2,
                                               const PDDistanceLabel &travelTimesToDropoff2) {
            static const PDDistanceLabel INVALID_DIST_LABEL = PDDistanceLabel(INVALID_DIST);
            static const PDDistanceLabel ZERO_DIST_LABEL = PDDistanceLabel(0);

            ++numDominationRelationTests;
            const auto &dropoff1 = requestState.dropoffs[dropoffId1];
            const auto &dropoff2 = requestState.dropoffs[dropoffId2];
            auto walkDiff = PDDistanceLabel(dropoff1.walkingDist - dropoff2.walkingDist);

            using F = CostCalculator::CostFunction;
            PDDistanceLabel maxDetourDiff = travelTimesToDropoff1 - travelTimesToDropoff2;
            PDDistanceLabel maxTripDiff = maxDetourDiff + walkDiff;
            PDDistanceLabel maxTripVioDiff = maxTripDiff;
            maxTripVioDiff.max(ZERO_DIST_LABEL);
            maxTripVioDiff.multiplyWithScalar(F::TRIP_VIO_WEIGHT);

            PDDistanceLabel maxTraversalCostDiff = costsToDropoff1 - costsToDropoff2;
            maxTraversalCostDiff.multiplyWithScalar(F::VEH_WEIGHT);

            const auto maxCostDiff = maxTraversalCostDiff + maxTripVioDiff;

            // Construct mask that is set wherever costsToDropoff1 is INVALID_DIST
            const PDLabelMask invalidMask = costsToDropoff1 == INVALID_DIST_LABEL;
            return ~invalidMask & (maxCostDiff < ZERO_DIST_LABEL);
        }


        int lowerBoundCostOfLabel(const PDPairAfterLastStopLabel &label) const {
//            const int minWaitVioCost = CostCalculator::CostFunction::calcWaitViolationCost(
//                    requestState.originalRequest.requestTime + label.travelTimeToPickup, requestState);
//            const int minTripTime =
//                    label.travelTimeToPickup + InputConfig::getInstance().stopTime + label.directTravelTime +
//                    requestState.dropoffs[label.dropoffId].walkingDist;
//            const int minTripVioCost = CostCalculator::CostFunction::calcTripTimeViolationCost(minTripTime,
//                                                                                               requestState);
            return label.costToPickup + label.directCost;
        }

        // Settles the global label with minimum cost lower bound. Sets labelAtV to the closed label and v to the vertex
        // where the label was closed. Returns true if the label was propagated to the neighbors of v or false if the
        // label was pruned at v.
        bool settleNextLabel(int &v, PDPairAfterLastStopLabel &labelAtV) {
            int costLowerBound;
            reverseQueue.min(v, costLowerBound);
            labelAtV = reverseLabelBuckets.closeMinOpenLabel(v);
            assert(lowerBoundCostOfLabel(labelAtV) == costLowerBound);

            // Check if this label can be pruned at v
            const bool pruned = STALL_LABELS && pruneLabel(v, labelAtV);

            if (!pruned) {
                // Push minTripTimeLabelAtV along all edges out of v.
                FORALL_INCIDENT_EDGES(queryGraph, v, e) {
                    ++numEdgeRelaxations;
                    const auto w = queryGraph.edgeHead(e);
                    if (w == v) continue;
                    PDPairAfterLastStopLabel labelViaV = labelAtV;
                    labelViaV.costToPickup += queryGraph.template get<CH::Weight>(e);
                    labelViaV.travelTimeToPickup += queryGraph.travelTime(e);

                    // Check whether the lower bound of this label exceeds the current upper bound for the cost of any
                    // assignment
                    const auto minCostOfLabelViaV = lowerBoundCostOfLabel(labelViaV);
                    if (minCostOfLabelViaV > bestCostWithoutConstraints)
                        continue;


                    bool inserted = insertLabelAtVertexAndClean(w, reverseLabelBuckets, labelViaV, minCostOfLabelViaV);

                    if (inserted) {
                        // Update PQ of buckets for change at w.
                        if (!reverseQueue.contains(w)) {
                            reverseQueue.insert(w, minCostOfLabelViaV);
                        } else {
                            const auto minLabelAtW = reverseLabelBuckets.minOpenLabel(w);
                            const auto lowerBoundCost = lowerBoundCostOfLabel(minLabelAtW);
                            reverseQueue.decreaseKey(w, lowerBoundCost);
                        }
                    }
                }
            }

            // Update PQ of buckets for closing the min label at v.
            if (reverseLabelBuckets.getBucketOf(v).open().size() == 0) {
                int deletedV;
                reverseQueue.deleteMin(deletedV, costLowerBound);
                assert(v == deletedV && costLowerBound == lowerBoundCostOfLabel(labelAtV));
            } else {
                reverseQueue.increaseKey(v, lowerBoundCostOfLabel(reverseLabelBuckets.minOpenLabel(v)));
            }

            return !pruned;
        }

        // Checks whether newLabel is dominated by existing labels in the bucket at vertex. If newLabel is not dominated,
        // newLabel is inserted into the bucket, other open labels that are dominated by newLabel are removed and true is
        // returned. If newLabel is dominated by existing labels, no change to the bucket is made and false is returned.
        // Sets the given flag to true iff previously all open labels at this vertex had the same pickup, the new label has
        // this pickup, too, and the new label replaces all existing open labels (this happens if the new label reaches the
        // vertex along a new shorter path to the one pickup that all labels have).
        bool insertLabelAtVertexAndClean(const int vertex, BucketContainer &bucketContainer,
                                         const PDPairAfterLastStopLabel &newLabel,
                                         const int minCostOfNewLabel) {

            // Check if labelViaV is dominated by any closed labels at vertex.
            // Min cost of closed label at vertex <= max cost of closed label at vertex <= max cost of new label at vertex
            // => new label cannot dominate closed label.
            for (const auto &closedLabel: bucketContainer.getBucketOf(vertex).closed()) {
                if (dominates(closedLabel, newLabel)) {
                    return false;
                }
            }

            // Check if labelViaV is dominated by any open labels at vertex.
            auto openLabels = bucketContainer.getBucketOf(vertex).open();
            for (int i = 0; i < openLabels.size(); ++i) {
                if (dominates(openLabels[i], newLabel))
                    return false;
            }

            // Check if new label dominates any open labels
            markedIndices.clear();
            // If first label has same pickup as new label, it may be dominated only based on the distance to the pickup
            // which can temporarily increase the minimum cost at this vertex. We mark this case explicitly with a flag.
            if (openLabels.size() > 0 &&
                dominates(newLabel, openLabels[0])) {
                markedIndices.push_back(0);
            }
            for (int i = 1; i < openLabels.size(); ++i) {
                if (dominates(newLabel, openLabels[i])) {
                    markedIndices.push_back(i);
                }
            }

            // Remove open labels dominated by new label
            bucketContainer.stableRemoveOpenLabelsAtIndices(vertex, markedIndices);

            // Insert new label at the right spot into the open labels
            openLabels = bucketContainer.getBucketOf(vertex).open(); // open labels changed
            if (openLabels.size() == 0) {
                bucketContainer.stableInsertOpenLabel(vertex, 0, newLabel);
                return true;
            }

            for (int i = 0; i < openLabels.size(); ++i) {
                if (minCostOfNewLabel < lowerBoundCostOfLabel(openLabels[i])) {
                    bucketContainer.stableInsertOpenLabel(vertex, i, newLabel);
                    return true;
                }
            }

            bucketContainer.stableInsertOpenLabel(vertex, bucketContainer.getBucketOf(vertex).open().size(), newLabel);
            return true;
        }

        // Checks if a label l can be pruned at vertex v via a stall-on-demand like criterion: Consider all outgoing edges
        // (v,w) from v in the upward search graph. For each label l' at w, we simulate propagating that label to v backwards
        // via (v,w) to attain a label l'' at v. If l'' dominates l at v, then we can prune l at v.
        bool pruneLabel(const int v, const PDPairAfterLastStopLabel &label) {
            FORALL_INCIDENT_EDGES(oppositeGraph, v, e) {
                const auto w = oppositeGraph.edgeHead(e);
                if (w == v) continue;
                const auto &bucketAtW = reverseLabelBuckets.getBucketOf(w);

                // Check if labelAtW is dominated by any closed labels at w.
                for (const auto &closedLabel: bucketAtW.closed()) {
                    auto closedLabelAtV = closedLabel;
                    closedLabelAtV.costToPickup += oppositeGraph.template get<CH::Weight>(e);

                    if (dominates(closedLabelAtV, label)) {
                        return true;
                    }
                }

                // Check if labelAtW is dominated by any open labels at vertex.
                for (const auto &openLabel: bucketAtW.open()) {
                    auto openLabelAtV = openLabel;
                    openLabelAtV.costToPickup += oppositeGraph.template get<CH::Weight>(e);
                    if (dominates(openLabelAtV, label)) {
                        return true;
                    }
                }
            }
            return false;
        }


        // Returns true iff label1 dominates label2, i.e. iff the cost of inserting the pickup and dropoff in label1 will
        // always be smaller than the cost of inserting the pickup and dropoff in label2 for any vehicle encountered in the
        // sub-search-tree rooted at the vertex that label1 and label2 can be found in. This ordering is only partial at
        // each vertex v where dist(v, p) < walkingDist(p) for any pickup p that has a label at v since in that case the
        // walking distance of p may still dominate the time till the departure at p.
        bool dominates(const PDPairAfterLastStopLabel &label1, const PDPairAfterLastStopLabel &label2) {
            ++numDominationRelationTests;

            if (label1.pickupId == label2.pickupId)
                return label1.dropoffId == label2.dropoffId && label1.costToPickup <= label2.costToPickup && label1.travelTimeToPickup <= label2.travelTimeToPickup;

            const auto &pickup1 = requestState.pickups[label1.pickupId];
            const auto &dropoff1 = requestState.dropoffs[label1.dropoffId];
            const auto &dropoff2 = requestState.dropoffs[label2.dropoffId];

            using F = CostCalculator::CostFunction;
            const auto maxDepTimeDiff =
                    std::max(label1.travelTimeToPickup + InputConfig::getInstance().stopTime, pickup1.walkingDist) -
                    (label2.travelTimeToPickup + InputConfig::getInstance().stopTime);
            const auto maxDetourDiff = maxDepTimeDiff + label1.directTravelTime - label2.directTravelTime;
            const auto maxTripDiff = maxDetourDiff + dropoff1.walkingDist - dropoff2.walkingDist;

            const auto maxWaitVioDiff = F::WAIT_VIO_WEIGHT * std::max(maxDepTimeDiff, 0);
            const auto maxTripVioDiff = F::TRIP_VIO_WEIGHT * std::max(maxTripDiff, 0);

            const auto maxCostDiff = F::VEH_WEIGHT * (label1.costToPickup + label1.directCost -
                                                      label2.costToPickup - label2.directCost) +
                                     maxWaitVioDiff + maxTripVioDiff;
            return maxCostDiff < 0;
        }

        void scanVehicleBucket(const int rank, const PDPairAfterLastStopLabel &label) {

            const auto &pickup = requestState.pickups[label.pickupId];
            const auto &dropoff = requestState.dropoffs[label.dropoffId];

            Assignment asgn;
            asgn.costFromPickup = 0;
            asgn.costFromDropoff = 0;
            asgn.travelTimeFromPickup = 0;
            asgn.travelTimeFromDropoff = 0;
            asgn.pickup = &pickup;
            asgn.dropoff = &dropoff;
            asgn.costToDropoff = label.directCost;
            asgn.travelTimeToDropoff = label.directTravelTime;

            int numEntriesScannedInBucket = 0;

            if constexpr (!LastStopBucketsEnvT::SORTED) {
                auto bucket = lastStopBuckets.getUnsortedBucketOf(rank);
                for (const auto &entry: bucket) {
                    ++numEntriesScannedInBucket;
                    const int fullCostToPickup = entry.distToTarget + label.costToPickup;
                    const int fullTravelTime = entry.travelTimeToTarget + label.travelTimeToPickup;
                    tryTentativeAssignment(entry.targetId, fullCostToPickup, fullTravelTime, asgn);
                }
            } else {


                // Scan sorted bucket. Sorted by distance, stop early if distance does not permit a better insertion than
                // one already seen.
                for (const auto &entry: lastStopBuckets.getBucketOf(rank)) {
                    ++numEntriesScannedInBucket;
                    const int fullCostToPickup = entry.distToTarget + label.costToPickup;
                    // Vehicles are ordered by distToTarget, i.e. the cost from the last stop to v.
                    // Thus, fullCostToPickup + label.directCost increases monotonously with every scanned entry.
                    // Once we scan an entry for which the upperBoundCost is exceeded, all remaining entries will also
                    // exceed it, and we can stop scanning the bucket.
                    if (fullCostToPickup + label.directCost > upperBoundCostWithConstraints)
                        break;

                    const int fullTravelTime = entry.travelTimeToTarget + label.travelTimeToPickup;
                    tryTentativeAssignment(entry.targetId, fullCostToPickup, fullTravelTime, asgn);
                }
            }

            numEntriesScanned += numEntriesScannedInBucket;
        }

        inline void tryTentativeAssignment(const int vehId, const int fullCostToPickup,
                                           const int fullTravelTimeToPickup, Assignment &asgn) {

            const int &numStops = routeState.numStopsOf(vehId);
            asgn.vehicle = &fleet[vehId];
            asgn.pickupStopIdx = numStops - 1;
            asgn.dropoffStopIdx = numStops - 1;
            asgn.costToPickup = fullCostToPickup;
            asgn.travelTimeToPickup = fullTravelTimeToPickup;

            // Have to ignore service time hard constraint since we only search for promising dropoffs.
            // If all promising dropoffs violate the service time constraint but there is an unpromising dropoff
            // that does not, then we have to still register the vehicle as seen in order to later be able to find
            // out that a PALS assignment better than the upper bound cost may exist.
            const auto costIgnoringHardConstraints = calculator.calcWithoutHardConstraints(asgn, requestState);

            assert(bestCostWithoutConstraints <= upperBoundCostWithConstraints);
            if (costIgnoringHardConstraints > bestCostWithoutConstraints)
                return;

            if (costIgnoringHardConstraints == bestCostWithoutConstraints) {
                if (!breakCostTie(asgn, bestAsgn))
                    return;
            }

            // If the cost is better than the best known cost for the vehicle or if the costs are equal and the
            // determinism argument decides so, update the best pair to this pair.
            bestCostWithoutConstraints = costIgnoringHardConstraints;
            bestAsgn = asgn;

            // fullDistToPickup is an upper bound on the actual distance from the last stop of this vehicle to this
            // pickup. Therefore, the cost with hard constraints and fullDistToPickup is an upper bound on the
            // actual cost of an assignment with this vehicle, pickup and dropoff. This is also an upper bound on
            // the cost of the best assignment. We can use this upper bound to prune the search.
            const auto costWithHardConstraints = calculator.calc(asgn, requestState);
            upperBoundCostWithConstraints = std::min(upperBoundCostWithConstraints, costWithHardConstraints);
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &queryGraph;
        const CH::SearchGraph &oppositeGraph;
        const Fleet &fleet;
        const RouteState &routeState;
        const CostCalculator &calculator;
        const typename LastStopBucketsEnvT::BucketContainer &lastStopBuckets;
        DirectSearchesT &directSearches;
        const RequestState &requestState;

        BucketContainer reverseLabelBuckets;
        QueueT reverseQueue;

        std::vector<int> markedIndices;

        int upperBoundCostWithConstraints;

        // At the end of a search run, bestCostWithoutConstraints is the cost of bestAsgn, ignoring the service time
        // hard constraint. This cost is a lower bound on the cost of the actual best PALS assignment.
        // If bestAsgn does not violate the constraint, the cost is exact and bestAsgn is the best PALS assignment.
        int bestCostWithoutConstraints;
        Assignment bestAsgn;

        int numInitialLabelsGenerated;
        int numInitialLabelsNotPruned;
        int numLabelsRelaxed;
        int numEntriesScanned;
        int numEdgeRelaxations;
        int numDominationRelationTests;
        int64_t initializationTime;
        int64_t runTime;

        std::vector<unsigned int> dropoffIdsForInitialLabels;
        std::vector<PDDistanceLabel> directCostsForInitialLabels;
        std::vector<PDDistanceLabel> directTravelTimesForInitialLabels;


    };
}