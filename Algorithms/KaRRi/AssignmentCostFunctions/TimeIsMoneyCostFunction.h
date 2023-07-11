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


    template<int PASSENGER_COST_SCALE = 1, int VEHICLE_COST_SCALE = 1, int WAIT_TIME_VIOLATION_WEIGHT = 1, int TRIP_TIME_VIOLATION_WEIGHT = 10>
    struct TimeIsMoneyCostFunction {

        template<typename RequestContext>
        static inline int calcUpperBoundTripCostDifference(const int tripTimeDifference, const RequestContext &) {
            return PASSENGER_COST_SCALE * TRIP_TIME_VIOLATION_WEIGHT * tripTimeDifference;
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel
        calcKUpperBoundTripCostDifferences(const DistanceLabel &tripTimeDifference, const RequestContext &) {
            auto diff = tripTimeDifference;
            diff.multiplyWithScalar(PASSENGER_COST_SCALE * TRIP_TIME_VIOLATION_WEIGHT);
            return diff;
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
            if (tripTime <= maxTripTime) {
                return PASSENGER_COST_SCALE * tripTime;
            } else {
                return PASSENGER_COST_SCALE * (maxTripTime + (tripTime - maxTripTime) * TRIP_TIME_VIOLATION_WEIGHT);
            }
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel calcKTripCosts(const DistanceLabel &tripTime, const RequestContext &context) {

            const auto maxTripTime = context.getOriginalReqMaxTripTime();
            DistanceLabel violationCost = tripTime - maxTripTime;
            violationCost.max(0);
            violationCost.multiplyWithScalar(TRIP_TIME_VIOLATION_WEIGHT);

            DistanceLabel regularCost = tripTime;
            regularCost.min(maxTripTime);

            auto cost = regularCost + violationCost;
            cost.multiplyWithScalar(PASSENGER_COST_SCALE);

            return cost;
        }

        static constexpr inline int calcWalkingCost(const int, const int) {
            // Time is money => walking time is part of passengers trip time so do not count it again
            return PASSENGER_COST_SCALE * 0;
        }

        template<typename DistanceLabel>
        static constexpr inline DistanceLabel calcKWalkingCosts(const DistanceLabel &, const int) {
            // Time is money => walking time is part of passengers trip time so do not count it again
            return PASSENGER_COST_SCALE * 0;
        }

        template<typename RequestContext>
        static inline int calcWaitViolationCost(const int actualDepTimeAtPickup, const RequestContext &context) {
            if (actualDepTimeAtPickup <= context.getMaxDepTimeAtPickup())
                return 0;
            return PASSENGER_COST_SCALE * WAIT_TIME_VIOLATION_WEIGHT *
                   (actualDepTimeAtPickup - context.getMaxDepTimeAtPickup());
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel calcKWaitViolationCosts(const DistanceLabel &actualDepTimeAtPickup,
                                                            const RequestContext &context) {
            DistanceLabel violationCost = actualDepTimeAtPickup - context.getMaxDepTimeAtPickup();
            violationCost.max(0);
            violationCost.multiplyWithScalar(WAIT_TIME_VIOLATION_WEIGHT);
            violationCost.multiplyWithScalar(PASSENGER_COST_SCALE);
            return violationCost;
        }

        static inline int calcUpperBoundWaitViolationCostDifference(const int diffInTimeTillDepAtPickup) {
            assert(diffInTimeTillDepAtPickup >= 0);
            return PASSENGER_COST_SCALE * WAIT_TIME_VIOLATION_WEIGHT * diffInTimeTillDepAtPickup;
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