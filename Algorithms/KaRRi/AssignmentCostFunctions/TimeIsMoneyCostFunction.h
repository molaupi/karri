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


    template<int PASSENGER_COST_SCALE = 1, int WALKING_COST_SCALE = 0, int VEHICLE_COST_SCALE = 1>
    struct TimeIsMoneyCostFunction {

        static constexpr int PSG_WEIGHT = PASSENGER_COST_SCALE;
        static constexpr int WALK_WEIGHT = WALKING_COST_SCALE;
        static constexpr int VEH_WEIGHT = VEHICLE_COST_SCALE;

        static constexpr inline int calcTripCost(const int tripTime) {
            return PASSENGER_COST_SCALE * tripTime;
        }

        template<typename DistanceLabel>
        static constexpr inline DistanceLabel calcKTripCosts(const DistanceLabel &tripTime) {
            auto cost = tripTime;
            cost.multiplyWithScalar(PASSENGER_COST_SCALE);
            return cost;
        }

        // Walking cost based on time spent walking, i.e. given dist is a time, not distance.
        static constexpr inline int calcWalkingCost(const int walkingDist) {
            return WALKING_COST_SCALE * walkingDist;
        }

        // Walking cost based on time spent walking, i.e. given dist is a time, not distance.
        template<typename DistanceLabel>
        static constexpr inline DistanceLabel calcKWalkingCosts(const DistanceLabel &walkingDist) {
            auto cost = walkingDist;
            cost.multiplyWithScalar(WALKING_COST_SCALE);
            return cost;
        }

        static inline int calcChangeInTripCostsOfExistingPassengers(const int addedTripTimeForExistingPassengers) {
            return PASSENGER_COST_SCALE * addedTripTimeForExistingPassengers;
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
            if constexpr (VEHICLE_COST_SCALE == 0)
                return INFTY;
            else
                return cost / VEHICLE_COST_SCALE + (cost % VEHICLE_COST_SCALE != 0) + maxLegLength;
        }

    };
}