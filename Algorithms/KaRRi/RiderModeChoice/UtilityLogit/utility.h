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
}