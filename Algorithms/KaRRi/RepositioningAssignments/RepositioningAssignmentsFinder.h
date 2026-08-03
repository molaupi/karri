/// *******************************************************************************
/// Minimal RepositioningAssignmentsFinder facade
/// *******************************************************************************
#pragma once

#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {

    template<typename InputGraphT, typename PDDistancesT, typename StrategyT>
    class RepositioningAssignmentsFinder {
    public:
        RepositioningAssignmentsFinder(StrategyT &strategy) : strategy(strategy) {}

        void findAssignments(const PDDistancesT &pdDistances, const PDLocs &pdLocs) {
            strategy.tryRepositioningAssignments(pdDistances, pdLocs);
        }

        void init() {
            strategy.init();
        }

    private:
        StrategyT &strategy;
    };

}
