#pragma once

#include <string>

namespace karri {
    enum BestAsgnType {
        ONE_LEG,
        TWO_LEGS,
        INVALID
    };

    inline std::string bestAsgnTypeName(const BestAsgnType &t) {
        switch (t) {
            case ONE_LEG:
                return "one_leg";
            case TWO_LEGS:
                return "two_legs";
            case INVALID:
                return "invalid";
            default:
                return "unknown";
        }
    }
}