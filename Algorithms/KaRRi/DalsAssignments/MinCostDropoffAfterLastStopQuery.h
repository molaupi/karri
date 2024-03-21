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

namespace karri::DropoffAfterLastStopStrategies {

    // For each vehicle veh, this query finds the dropoff with the minimal cost when inserting the dropoff after the last
    // stop of the vehicle. Ignores the service time constraint.
    template<typename InputGraphT,
            typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename IsVehEligibleForDropoffAfterLastStop,
            bool STALL_LABELS = true,
            typename QueueT = AddressableQuadHeap>
    class MinCostDropoffAfterLastStopQuery {

        struct DropoffLabel {
            int dropoffId = INVALID_ID;
            int distToDropoff = std::numeric_limits<int>::max();

            constexpr bool operator==(const DropoffLabel &rhs) const noexcept {
                return dropoffId == rhs.dropoffId;
            }
        };

        using VertexBucketContainer = LabelBucketContainer<DropoffLabel>;
        using VehicleBucketContainer = DynamicBucketContainer<DropoffLabel>;

    public:

        MinCostDropoffAfterLastStopQuery(const InputGraphT &inputGraph,
                                         const Fleet &fleet,
                                         const CHEnvT &chEnv,
                                         const CostCalculator &calculator,
                                         const LastStopBucketsEnvT &lastStopBucketsEnv,
                                         const IsVehEligibleForDropoffAfterLastStop &isVehEligibleForDropoffAfterLastStop,
                                         const RouteState &routeState,
                                         const RequestState &requestState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  ch(chEnv.getCH()),
                  searchGraph(ch.downwardGraph()),
                  oppositeGraph(ch.upwardGraph()),
                  calculator(calculator),
                  lastStopBuckets(lastStopBucketsEnv.getBuckets()),
                  isVehEligibleForDropoffAfterLastStop(isVehEligibleForDropoffAfterLastStop),
                  routeState(routeState),
                  requestState(requestState),
                  vertexLabelBuckets(searchGraph.numVertices()),
                  reverseQueue(searchGraph.numVertices()),
                  vehicleLabelBuckets(fleet.size()),
                  vehiclesSeen(fleet.size()) {}

        void run() {
            Timer timer;

            init();

            initializationTime = timer.elapsed<std::chrono::nanoseconds>();
            timer.restart();

            int v;
            DropoffLabel label;
            while (!stopSearch()) {
                const bool unpruned = settleNextLabel(v, label);
                if (unpruned)
                    scanVehicleBucket(v, label);
            }

            runTime = timer.elapsed<std::chrono::nanoseconds>() + initializationTime;
        }

        ConstantVectorRange<DropoffLabel> getParetoBestDropoffLabelsFor(const int vehId) const {
            assert(vehId >= 0 && vehId < fleet.size());
            return vehicleLabelBuckets.getBucketOf(vehId);
        }

        // Returns the vehicles that have at least one pareto best label.
        const Subset &getVehiclesSeen() const {
            return vehiclesSeen;
        }

        int getNumEdgeRelaxations() const {
            return numEdgeRelaxations;
        }

        int getNumLabelsRelaxed() const {
            return numLabelsRelaxed;
        }

        int getNumEntriesScanned() const {
            return numEntriesScanned;
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
            return minCostLowerBound > requestState.getBestCost();
        }

        void init() {
            numEdgeRelaxations = 0;
            numLabelsRelaxed = 0;
            numEntriesScanned = 0;
            numDominationRelationTests = 0;

            vertexLabelBuckets.clear();
            reverseQueue.clear();

            // Faster reset of vehicle buckets: A vehicle's bucket has entries iff it is in vehiclesSeen.
            // This set is likely small.
            for (const auto &vehId: vehiclesSeen) {
                vehicleLabelBuckets.clearBucket(vehId);
            }
            assert(vehicleLabelBuckets.allEmpty());
            vehiclesSeen.clear();

            for (const auto &dropoff: requestState.dropoffs) {
                const auto tailRank = ch.rank(inputGraph.edgeTail(dropoff.loc));
                const auto offset = inputGraph.travelTime(dropoff.loc);
                const DropoffLabel initialLabel = {dropoff.id, offset};
                const auto initialCost = costOf(initialLabel);
                if (initialCost > requestState.getBestCost())
                    continue;

                if (insertLabelAtVertexAndClean(tailRank, vertexLabelBuckets, initialLabel)) {
                    if (!reverseQueue.contains(tailRank)) {
                        reverseQueue.insert(tailRank, initialCost);
                    } else {
                        const auto &minCostLabel = vertexLabelBuckets.minOpenLabel(tailRank);
                        reverseQueue.decreaseKey(tailRank, costOf(minCostLabel));
                    }
                }
            }
        }

        int costOf(const DropoffLabel &label) const {
            if (label.distToDropoff >= INFTY) return INFTY;
            return calculator.calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(
                    label.distToDropoff, requestState.dropoffs[label.dropoffId], requestState);
        }

        bool dominates(const DropoffLabel &label1, const DropoffLabel &label2) {
            ++numDominationRelationTests;

            const auto &dropoff1 = requestState.dropoffs[label1.dropoffId];
            const auto &dropoff2 = requestState.dropoffs[label2.dropoffId];

            const auto maxDetourDiff = label1.distToDropoff - label2.distToDropoff;
            const auto walkDiff = dropoff1.walkingDist - dropoff2.walkingDist;
            const auto maxTripDiff = maxDetourDiff + walkDiff;

            using F = CostCalculator::CostFunction;
            const auto costDiffNoTripVio =
                    F::VEH_WEIGHT * maxDetourDiff + F::PSG_WEIGHT * maxTripDiff + F::WALK_WEIGHT * walkDiff;
            const auto costDiffTripVio = costDiffNoTripVio + F::TRIP_VIO_WEIGHT * maxTripDiff;
            return costDiffNoTripVio < 0 && costDiffTripVio < 0;
        }


        // Settles the next label with the globally minimal cost. Sets labelAtV to the settled label and v to the vertex
        // at which the label was settled. Returns true if the label was propagated to the neighbors of v or false if the
        // label was pruned.
        bool settleNextLabel(int &v, DropoffLabel &labelAtV) {
            assert(!reverseQueue.empty());
            int cost;
            reverseQueue.min(v, cost);
            labelAtV = vertexLabelBuckets.closeMinOpenLabel(v);
            assert(costOf(labelAtV) == cost);

            // Check if this label can be pruned at v
            const bool pruned = STALL_LABELS && pruneLabel(v, labelAtV);

            if (!pruned) {
                ++numLabelsRelaxed;

                // Push dropoff label along all edges out of v.
                FORALL_INCIDENT_EDGES(searchGraph, v, e) {
                    ++numEdgeRelaxations;
                    const auto w = searchGraph.edgeHead(e);
                    if (w == v) continue;
                    const auto edgeTime = searchGraph.template get<CH::Weight>(e);
                    DropoffLabel labelViaV = labelAtV;
                    labelViaV.distToDropoff += edgeTime;

                    bool inserted = insertLabelAtVertexAndClean(w, vertexLabelBuckets, labelViaV);
                    if (inserted) {
                        // Update PQ of buckets for change at w.
                        if (!reverseQueue.contains(w)) {
                            reverseQueue.insert(w, costOf(labelViaV));
                        } else {
                            const auto minLabelAtW = vertexLabelBuckets.minOpenLabel(w);
                            const auto lowerBoundCost = costOf(minLabelAtW);
                            reverseQueue.decreaseKey(w, lowerBoundCost);
                        }
                    }
                }
            }

            // Update PQ of buckets for closing the min label at v.
            if (vertexLabelBuckets.getBucketOf(v).open().size() == 0) {
                int deletedV;
                reverseQueue.deleteMin(deletedV, cost);
                assert(v == deletedV && cost == costOf(labelAtV));
            } else {
                reverseQueue.increaseKey(v, costOf(vertexLabelBuckets.minOpenLabel(v)));
            }

            return !pruned;
        }

        // Checks if a label l can be pruned at vertex v via a stall-on-demand like criterion: Consider all outgoing edges
        // (v,w) from v in the upward search graph. For each label l' at w, we simulate propagating that label to v backwards
        // via (v,w) to attain a label l'' at v. If l'' dominates l at v, then we can prune l at v.
        bool pruneLabel(const int v, const DropoffLabel &label) {
            FORALL_INCIDENT_EDGES(oppositeGraph, v, e) {
                const auto w = oppositeGraph.edgeHead(e);
                if (w == v) continue;
                const auto &bucketAtW = vertexLabelBuckets.getBucketOf(w);

                // Check if labelAtW is dominated by any closed labels at w.
                for (const auto &closedLabel: bucketAtW.closed()) {
                    auto closedLabelAtV = closedLabel;
                    closedLabelAtV.distToDropoff += oppositeGraph.template get<CH::Weight>(e);

                    if (dominates(closedLabelAtV, label)) {
                        return true;
                    }
                }

                // Check if labelAtW is dominated by any open labels at vertex.
                for (const auto &openLabel: bucketAtW.open()) {
                    auto openLabelAtV = openLabel;
                    openLabelAtV.distToDropoff += oppositeGraph.template get<CH::Weight>(e);
                    if (dominates(openLabelAtV, label)) {
                        return true;
                    }
                }
            }
            return false;
        }

        // Checks whether newLabel is dominated by existing labels in the bucket at vertex. If newLabel is not dominated,
        // newLabel is inserted into the bucket, other open labels that are dominated by newLabel are removed and true is
        // returned. If newLabel is dominated by existing labels, no change to the bucket is made and false is returned.
        bool insertLabelAtVertexAndClean(const int vertex, VertexBucketContainer &bucketContainer,
                                         const DropoffLabel &newLabel) {

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
            for (int i = 0; i < openLabels.size(); ++i) {
                if (dominates(newLabel, openLabels[i]))
                    markedIndices.push_back(i);
            }

            // Remove open labels dominated by new label
            bucketContainer.stableRemoveOpenLabelsAtIndices(vertex, markedIndices);

            // Insert new label at the right spot into the open labels
            openLabels = bucketContainer.getBucketOf(vertex).open(); // open labels changed
            if (openLabels.size() == 0) {
                bucketContainer.stableInsertOpenLabel(vertex, 0, newLabel);
                return true;
            }

            const auto costOfNew = costOf(newLabel);
            if (costOfNew < costOf(openLabels[0])) {
                bucketContainer.stableInsertOpenLabel(vertex, 0, newLabel);
                return true;
            }

            for (int i = 0; i < openLabels.size(); ++i) {
                if (costOfNew < costOf(openLabels[i])) {
                    bucketContainer.stableInsertOpenLabel(vertex, i, newLabel);
                    return true;
                }
            }

            bucketContainer.stableInsertOpenLabel(vertex, bucketContainer.getBucketOf(vertex).open().size(), newLabel);
            return true;
        }

        void scanVehicleBucket(const int v, const DropoffLabel &label) {
            using namespace time_utils;

            const auto &dropoff = requestState.dropoffs[label.dropoffId];

            int numEntriesScannedHere = 0;

            if constexpr (!LastStopBucketsEnvT::SORTED) {
                auto bucket = lastStopBuckets.getUnsortedBucketOf(v);
                for (const auto &entry: bucket) {
                    ++numEntriesScannedHere;

                    const int &vehId = entry.targetId;
                    const int fullDistToDropoff = entry.distToTarget + label.distToDropoff;

                    const auto costFromLastStop = calculator.calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(
                            fullDistToDropoff, dropoff, requestState);

                    if (costFromLastStop > requestState.getBestCost())
                        continue;

                    // If vehicle is not eligible for dropoff after last stop assignments, it does not need to be regarded.
                    if (!isVehEligibleForDropoffAfterLastStop(vehId))
                        continue;

                    // If full distance to dropoff leads to violation of service time constraint, an assignment with this
                    // vehicle and dropoff does not need to be regarded.
                    const int vehDepTimeAtLastStop = getVehDepTimeAtStopForRequest(vehId,
                                                                                   routeState.numStopsOf(vehId) - 1,
                                                                                   requestState, routeState);
                    if (fleet[vehId].endOfServiceTime < vehDepTimeAtLastStop + fullDistToDropoff + InputConfig::getInstance().stopTime)
                        continue;

                    const DropoffLabel labelAtVeh = {dropoff.id, fullDistToDropoff};
                    insertLabelAtVehicleAndClean(vehId, labelAtVeh);
                }
            } else {

                // Idle vehicles cannot lead to dropoff after last stop queries, so only consider non-idle ones.
                auto nonIdleBucket = lastStopBuckets.getNonIdleBucketOf(v);

                for (const auto &entry: nonIdleBucket) {
                    ++numEntriesScannedHere;

                    const int &vehId = entry.targetId;
                    const int arrTimeAtDropoff = entry.distToTarget + label.distToDropoff;

                    const auto costFromLastStop = calculator.calcVehicleIndependentCostLowerBoundForDALSWithKnownMinArrTime(
                            dropoff.walkingDist, label.distToDropoff, arrTimeAtDropoff, requestState);

                    // Entries of idle bucket are sorted by arrival time at v. The vehicle-independent lower
                    // bound costFromLastStop increases monotonously with this arrival time. So once we find an entry
                    // where the lower bound exceeds the best known cost, all remaining entries in the bucket will, too.
                    if (costFromLastStop > requestState.getBestCost())
                        break;

                    // If vehicle is not eligible for dropoff after last stop assignments, it does not need to be regarded.
                    if (!isVehEligibleForDropoffAfterLastStop(vehId))
                        continue;

                    // If full distance to dropoff leads to violation of service time constraint, an assignment with this
                    // vehicle and dropoff does not need to be regarded.
                    if (fleet[vehId].endOfServiceTime < arrTimeAtDropoff + InputConfig::getInstance().stopTime)
                        continue;


                    const int &depTimeAtLastStop = routeState.schedDepTimesFor(vehId)[routeState.numStopsOf(vehId) - 1];
                    const int fullDistToDropoff = arrTimeAtDropoff - depTimeAtLastStop;
                    const DropoffLabel labelAtVeh = {dropoff.id, fullDistToDropoff};
                    insertLabelAtVehicleAndClean(vehId, labelAtVeh);
                }
            }

            numEntriesScanned += numEntriesScannedHere;
        }

        // Checks whether newLabel is dominated by existing labels in the bucket for pareto best dropoffs for after the last
        // stop of the given vehicle. If newLabel is not dominated, newLabel is inserted into the bucket, other labels that
        // are dominated by newLabel are removed and true is returned. If newLabel is dominated by existing labels, no
        // change to the bucket is made and false is returned.
        bool insertLabelAtVehicleAndClean(const int vehId, const DropoffLabel &newLabel) {

            const auto costOfNew = costOf(newLabel);
            auto bucket = vehicleLabelBuckets.getBucketOf(vehId);
            int i = 0;
            while (i < bucket.size() && costOf(bucket[i]) < costOfNew) {
                if (dominates(bucket[i], newLabel))
                    return false;
                ++i;
            }

            markedIndices.clear();
            while (i < bucket.size() && costOf(bucket[i]) == costOfNew) {
                if (dominates(bucket[i], newLabel)) {
                    assert(markedIndices.empty());
                    return false;
                } else if (dominates(newLabel, bucket[i])) {
                    markedIndices.push_back(i);
                }
                ++i;
            }

            vehicleLabelBuckets.stableInsert(vehId, i, newLabel);
            bucket = vehicleLabelBuckets.getBucketOf(vehId); // bucket has changed
            ++i; // for inserted label
            while (i < bucket.size()) {
                assert(costOf(bucket[i]) > costOfNew);
                if (dominates(newLabel, bucket[i]))
                    markedIndices.push_back(i);
                ++i;
            }

            vehicleLabelBuckets.stableRemoveSortedIndices(vehId, markedIndices);

            vehiclesSeen.insert(vehId);
            return true;
        }


        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CH &ch;
        const typename CH::SearchGraph &searchGraph;
        const typename CH::SearchGraph &oppositeGraph;
        const CostCalculator &calculator;
        const typename LastStopBucketsEnvT::BucketContainer &lastStopBuckets;
        const IsVehEligibleForDropoffAfterLastStop &isVehEligibleForDropoffAfterLastStop;
        const RouteState &routeState;
        const RequestState &requestState;

        VertexBucketContainer vertexLabelBuckets;
        QueueT reverseQueue;

        std::vector<int> markedIndices;

        VehicleBucketContainer vehicleLabelBuckets;
        Subset vehiclesSeen;

        int numEdgeRelaxations;
        int numLabelsRelaxed;
        int numEntriesScanned;
        int numDominationRelationTests;
        int64_t initializationTime;
        int64_t runTime;
    };

}