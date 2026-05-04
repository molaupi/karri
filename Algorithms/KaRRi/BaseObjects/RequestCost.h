
#pragma once

namespace karri {
    
    class RequestCost {

        public:
            int total = INFTY;

            int walkingCost = INFTY;
            int tripCost = INFTY;
            int waitTimeViolationCost = INFTY;
            int changeInTripCostsOfOthers = INFTY;
            int vehCost = INFTY;

            static RequestCost INFTY_COST() {
                RequestCost cost;

                cost.total = INFTY;
                cost.tripCost = INFTY;
                cost.waitTimeViolationCost = INFTY;
                cost.changeInTripCostsOfOthers = INFTY;
                cost.vehCost = INFTY;
                cost.walkingCost = INFTY;

                return cost;
            }

            friend bool operator==(const RequestCost &lhs, const RequestCost &rhs) = default;
    };

}