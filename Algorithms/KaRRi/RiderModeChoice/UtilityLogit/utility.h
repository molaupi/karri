#pragma once
#include <stdexcept>
#include <string>

#include "types.h"
#include "../TransportMode.h"

namespace karri::mode_choice::utility_logit {
/**
 * The example utility function for transport choice, using only the global coefficient and the travel time coefficient
 * multiplied by the travel time. Note that travel time is usually assumed to be minutes.
 * @param attributes
 * @param params
 * @return
 */
    inline double exampleUtilityFunction(const Attributes &attributes, const ModeParameters &params) {
        return params.coeffConstant
               + attributes.travelTimeMinutes * params.coeffTravelTime
               + attributes.waitingTimeMinutes * params.coeffWaitingTime
               + attributes.accessEgressTimeMinutes * params.coeffAccessTime;
    }

    inline TransportMode stringToTransportMode(const std::string &str) {
        if (str == "Car") return TransportMode::Car;
        if (str == "Ped") return TransportMode::Ped;
        if (str == "Taxi") return TransportMode::Taxi;
        throw std::runtime_error("Unrecognized transport mode");
    }

    inline ModeParameters fromMap(const ParameterMap &params) {
        return ModeParameters{
                .coeffConstant = params["coeff_constant"],
                .coeffTravelTime = params["coeff_travel_time"],
                .coeffWaitingTime = params["coeff_waiting_time"],
                .coeffAccessTime = params["coeff_access_time"],

        };
    }
}