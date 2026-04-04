#pragma once

#include "Tools/Logging/NullLogger.h"
#include "Tools/Logging/LogManager.h"
#include "Tools/ThreadSafeRandom.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "TransportMode.h"
#include "ModeChoiceInputStats.h"
#include "PTJourneyData.h"

namespace karri::mode_choice {
    template<typename CriterionT, typename LoggerT = NullLogger>
    class ModeChoice {
        static bool isCarAllowed(const Request &req) {
            return req.allowPrivateCarProbability > 0.0 && ThreadSafeRandom::randomNumber() < req.
                   allowPrivateCarProbability;
        }

    public:
        ModeChoice(const RouteState &routeState, const bool allowPublicTransport)
        : criterion(routeState),
            routeState(routeState),
            allowPublicTransport(allowPublicTransport),
            logger(LogManager<LoggerT>::getLogger("modechoice.csv",
                                                  "request_id,"
                                                  "walk_travel_time,"
                                                  "car_travel_time,"
                                                  "taxi_travel_time,"
                                                  "taxi_wait_time,"
                                                  "taxi_accegr_time,"
                                                  "pt_travel_time,"
                                                  "pt_wait_time,"
                                                  "pt_accegr_time,"
                                                  "mode\n")) {
        }

        template<typename AsgnFinderResponseT>
        TransportMode
        chooseMode(const Request &req, const AsgnFinderResponseT &resp, const PTJourneyData &ptJourneyData) {
            using namespace time_utils;

            criterion.init();
            criterion.registerPed(resp.odWalkingDist, resp);

            if (isCarAllowed(req)) {
                criterion.registerCar(resp.originalReqDirectDist);
            }

            if (allowPublicTransport && ptJourneyData.isValid()) {
                criterion.registerPublicTransport(ptJourneyData);
            }

            const auto &bestAsgn = resp.getBestAssignment();
            if (bestAsgn.vehicle) {
                criterion.registerTaxi(bestAsgn, resp);
            }

            const auto choice = criterion.apply();

            const auto &stats = criterion.getStats();
            logger << req.requestId << ","
                    << stats.walkTravelTime << ","
                    << stats.privateCarTravelTime << ","
                    << stats.taxiTravelTime << ","
                    << stats.taxiWaitTime << ","
                    << stats.taxiAccEgrTime << ","
                    << stats.ptTravelTime << ","
                    << stats.ptWaitTime << ","
                    << stats.ptAccEgrTime << ","
                    << (choice == TransportMode::Car
                            ? "Car"
                            : choice == TransportMode::Ped
                                  ? "Ped"
                                  : choice == TransportMode::PublicTransport
                                        ? "PublicTransport"
                                        : "Taxi")
                    << "\n";

            return choice;
        }

    private:
        CriterionT criterion;
        const RouteState &routeState;
        const bool allowPublicTransport;
        LoggerT &logger;
    };
} // namespace karri::mode_choice
