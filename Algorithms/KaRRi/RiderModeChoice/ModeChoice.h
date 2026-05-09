#pragma once

#include "Algorithms/KaRRi/TimeUtils.h"
#include "Tools/Logging/NullLogger.h"
#include "Tools/Logging/LogManager.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "TransportMode.h"
#include "UtilityLogit/types.h"
#include "WalkingResult.h"
#include "TaxiResult.h"
#include "CarResult.h"
#include "PTJourneyData.h"
#include "Tools/Constants.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri::mode_choice {
    template<typename CriterionT, typename LoggerT = NullLogger>
    class ModeChoice {

        static double tenthsOfSecondsToMinutes(int tenthsOfSeconds) {
            return static_cast<double>(tenthsOfSeconds) / 600.0;
        }

    public:
        ModeChoice(const RouteState &routeState) : criterion(),
                                                   routeState(routeState),
                                                   logger(LogManager<LoggerT>::getLogger("modechoice.csv",
                                                       "request_id,"
                                                       "walk_travel_time,"
                                                       "car_travel_time,"
                                                       "pt_travel_time,"
                                                       "pt_wait_time,"
                                                       "pt_accegr_time,"
                                                       "taxi_travel_time,"
                                                       "taxi_wait_time,"
                                                       "taxi_accegr_time,"
                                                       "taxi_type,"
                                                       "mode\n")) {
        }

        TransportMode chooseMode(const RequestState &requestState,
                                 const WalkingResult &walkOnlyResult,
                                 const CarResult &carOnlyResult,
                                 const TaxiResult &taxiResult,
                                 const PTJourneyData &ptJourneyData) const {
            using namespace time_utils;
            using namespace utility_logit;

            std::vector<Alternative<TransportMode> > entries;

            int walkTravelTime = INFTY;
            if (walkOnlyResult.isValid()) {
                walkTravelTime = walkOnlyResult.walkingDist;
                entries.push_back({
                    TransportMode::Ped, constructAttributesForTimesInTenthsOfSeconds(walkTravelTime, 0, 0)
                });
            }

            int carTravelTime = INFTY;
            if (carOnlyResult.isValid()) {
                carTravelTime = carOnlyResult.carDist;
                entries.push_back({
                    TransportMode::Car, constructAttributesForTimesInTenthsOfSeconds(carTravelTime, 0, 0)
                });
            }

            int ptOnlyTravelTime = INFTY;
            int ptOnlyWaitTime = INFTY;
            int ptOnlyAccessEgressTime = INFTY;
            if (ptJourneyData.isValid()) {
                entries.push_back({
                    TransportMode::PublicTransport,
                    Attributes(ptJourneyData.travelTimeMinutes, ptJourneyData.waitTimeMinutes,
                               ptJourneyData.accessEgressTimeMinutes)
                });
                ptOnlyTravelTime = static_cast<int>(ptJourneyData.travelTimeMinutes * 600.0);
                ptOnlyWaitTime = static_cast<int>(ptJourneyData.waitTimeMinutes * 600.0);
                ptOnlyAccessEgressTime = static_cast<int>(ptJourneyData.accessEgressTimeMinutes * 600.0);
            }

            int taxiOnlyTravelTime = INFTY;
            int taxiOnlyWaitTime = INFTY;
            int taxiOnlyAccessEgressTime = INFTY;
            BestAsgnType taxiAsgnType = INVALID;
            if (taxiResult.isValid()) {
                taxiOnlyTravelTime = taxiResult.inVehicleTime;
                taxiOnlyWaitTime = taxiResult.waitTime;
                taxiOnlyAccessEgressTime = taxiResult.walkTime;
                taxiAsgnType = taxiResult.asgnType;
                entries.push_back({
                    TransportMode::Taxi,
                    constructAttributesForTimesInTenthsOfSeconds(taxiResult.inVehicleTime, taxiResult.waitTime,
                                                                 taxiResult.walkTime)
                });
            }

            const auto choice = criterion.apply(entries);

            logger << requestState.originalRequest.requestId << ","
                    << walkTravelTime << ","
                    << carTravelTime << ","
                    << ptOnlyTravelTime << ","
                    << ptOnlyWaitTime << ","
                    << ptOnlyAccessEgressTime << ","
                    << taxiOnlyTravelTime << ","
                    << taxiOnlyWaitTime << ","
                    << taxiOnlyAccessEgressTime << ","
                    << bestAsgnTypeName(taxiAsgnType) << ","
                    << transportModeToString(choice) << "\n";

            return choice;
        }

    private:
        CriterionT criterion;
        const RouteState &routeState;
        LoggerT &logger;
    };
} // namespace karri::mode_choice
