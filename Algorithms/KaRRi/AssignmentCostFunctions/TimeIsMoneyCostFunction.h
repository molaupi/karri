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

namespace karri {


    template<int PASSENGER_COST_SCALE = 1, int WALKING_COST_SCALE = 0, int VEHICLE_COST_SCALE = 1, int WAIT_TIME_VIOLATION_WEIGHT = 1, int TRIP_TIME_VIOLATION_WEIGHT = 10>
    struct TimeIsMoneyCostFunction {

        static constexpr int PSG_WEIGHT = PASSENGER_COST_SCALE;
        static constexpr int WALK_WEIGHT = WALKING_COST_SCALE;
        static constexpr int VEH_WEIGHT = VEHICLE_COST_SCALE;
        static constexpr int WAIT_VIO_WEIGHT = WAIT_TIME_VIOLATION_WEIGHT;
        static constexpr int TRIP_VIO_WEIGHT = TRIP_TIME_VIOLATION_WEIGHT;

        template<typename RequestContext>
        static inline int calcUpperBoundTripCostDifference(const int tripTimeDifference, const RequestContext &) {
            return (PASSENGER_COST_SCALE + TRIP_TIME_VIOLATION_WEIGHT) * tripTimeDifference;
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel
        calcKUpperBoundTripCostDifferences(const DistanceLabel &tripTimeDifference, const RequestContext &) {
            auto diff = tripTimeDifference;
            diff.multiplyWithScalar(PASSENGER_COST_SCALE + TRIP_TIME_VIOLATION_WEIGHT);
            return diff;
        }

        static inline int calcUpperBoundTripViolationCostDifference(const int tripTimeDifference) {
            assert(tripTimeDifference >= 0);
            return TRIP_TIME_VIOLATION_WEIGHT * tripTimeDifference;
        }

        template<typename RequestContext>
        static inline int calcLowerBoundTripCostDifference(const int tripTimeDifference, const RequestContext &) {
            return PASSENGER_COST_SCALE * tripTimeDifference;
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel
        calcKLowerBoundTripCostDifferences(const DistanceLabel &tripTimeDifference, const RequestContext &) {
            auto diff = tripTimeDifference;
            diff.multiplyWithScalar(PASSENGER_COST_SCALE);
            return diff;
        }

        template<typename RequestContext>
        static inline int calcTripCost(const int tripTime, const RequestContext &context) {

            const auto maxTripTime = context.getOriginalReqMaxTripTime();
            const auto regularCost = PASSENGER_COST_SCALE * tripTime;
            const auto violationPenalty = TRIP_TIME_VIOLATION_WEIGHT * std::max(tripTime - maxTripTime, 0);

            return regularCost + violationPenalty;
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel calcKTripCosts(const DistanceLabel &tripTime, const RequestContext &context) {

            DistanceLabel regularCost = tripTime;
            regularCost.multiplyWithScalar(PASSENGER_COST_SCALE);

            const DistanceLabel maxTripTime = DistanceLabel(context.getOriginalReqMaxTripTime());
            DistanceLabel violationCost = tripTime - maxTripTime;
            violationCost.max(0);
            violationCost.multiplyWithScalar(TRIP_TIME_VIOLATION_WEIGHT);

            const DistanceLabel cost = regularCost + violationCost;
            return cost;
        }

        static constexpr inline int calcWalkingCost(const int walkingDist, const int) {
            // Time is money => walking time is part of passengers trip time so do not count it again
            return WALKING_COST_SCALE * walkingDist;
        }

        template<typename DistanceLabel>
        static constexpr inline DistanceLabel calcKWalkingCosts(const DistanceLabel &walkingDist, const int) {
            // Time is money => walking time is part of passengers trip time so do not count it again
            auto cost = walkingDist;
            cost.multiplyWithScalar(WALKING_COST_SCALE);
            return cost;
        }

        template<typename RequestContext>
        static inline int calcWaitViolationCost(const int actualDepTimeAtPickup, const RequestContext &context) {
            return WAIT_TIME_VIOLATION_WEIGHT * std::max(actualDepTimeAtPickup - context.getMaxDepTimeAtPickup(), 0);
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel calcKWaitViolationCosts(const DistanceLabel &actualDepTimeAtPickup,
                                                            const RequestContext &context) {
            DistanceLabel violationCost = actualDepTimeAtPickup - DistanceLabel(context.getMaxDepTimeAtPickup());
            violationCost.max(0);
            violationCost.multiplyWithScalar(WAIT_TIME_VIOLATION_WEIGHT);
            return violationCost;
        }

        static inline int calcUpperBoundWaitViolationCostDifference(const int diffInTimeTillDepAtPickup) {
            assert(diffInTimeTillDepAtPickup >= 0);
            return WAIT_TIME_VIOLATION_WEIGHT * diffInTimeTillDepAtPickup;
        }

        static inline int calcChangeInTripCostsOfExistingPassengers(const int addedTripTimeForExistingPassengers) {
            return PASSENGER_COST_SCALE * addedTripTimeForExistingPassengers;
        }

        static inline int calcUpperBoundVehicleCostDifference(const int detourDiff) {
            return calcVehicleCost(detourDiff);
        }

        static inline int calcLowerBoundVehicleCostDifference(const int detourDiff) {
            return calcVehicleCost(detourDiff);
        }

        static inline int calcVehicleCost(const int residualDetourAtEnd) {
            return VEHICLE_COST_SCALE * residualDetourAtEnd;
        }

        template<typename DistanceLabel>
        static inline DistanceLabel calcKVehicleCosts(const DistanceLabel &totalDetour) {
            auto cost = totalDetour;
            cost.multiplyWithScalar(VEHICLE_COST_SCALE);
            return cost;
        }

        // Returns the smallest distance from a pickup or to a dropoff (distance that is part of the detour)
        // s.t. the vehicle cost alone leads to a greater cost than the one given. Uses the maximum length of
        // any route leg to get a global lower bound on the detour.
        static inline int
        calcMinDistFromOrToPDLocSuchThatVehCostReachesMinCost(const int cost, const int maxLegLength) {
            return cost / VEHICLE_COST_SCALE + (cost % VEHICLE_COST_SCALE != 0) + maxLegLength;
        }

        // Returns the smallest distance from a pickup or to a dropoff (distance that is part of the detour and the trip
        // time) s.t. the vehicle cost and trip cost lead to a greater cost than the one given. Uses the maximum length of
        // any route leg to get a global lower bound on the detour.
        static inline int
        calcMinDistFromOrToPDLocSuchThatVehAndTripCostsReachMinCost(const int cost, const int maxLegLength) {
            const auto c = cost + VEHICLE_COST_SCALE * maxLegLength;
            const auto d = VEHICLE_COST_SCALE + PASSENGER_COST_SCALE;
            return c / d + (c % d != 0);
        }

    };
}