#pragma once

#include <unordered_map>
#include <string>

namespace karri::mode_choice::utility_logit {


/**
 * A simple parameter wrapper containing the two basic parameters found in transport mode discrete choice models. A
 * constant factor
 */
    struct ModeParameters {
        double coeffConstant;
        double coeffTravelTime;
        double coeffWaitingTime;
        double coeffAccessTime;
    };

/**
 * Attributes should bundle all the variable information that influences the decision process in a discrete choice model
 * In our current simple model, we only consider travel time, waiting time and access + egress time.
 * Future work may include more detailed information.
 */
    struct Attributes {
        double travelTimeMinutes;
        double waitingTimeMinutes;
        double accessEgressTimeMinutes;
    };

/**
 * An alternative has an original generic option T, as well as a set of specific attributes for a given choice situation.
 * @tparam T
 */
    template<typename T>
    struct Alternative {
        T option;
        Attributes data;
    };

/**
 * A collection of named parameters, usually parsed from a file.
 */
    struct ParameterMap {
        std::unordered_map<std::string, double> values;

        const double &operator[](const std::string &key) const {
            return values.at(key);
        }
    };

}