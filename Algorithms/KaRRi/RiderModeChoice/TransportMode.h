#pragma once
#include <stdexcept>
#include <string>

namespace karri::mode_choice {

    enum class TransportMode {
        Car, Ped, Taxi, PublicTransport, None
    };

    inline TransportMode stringToTransportMode(const std::string &str) {
        if (str == "Car") return TransportMode::Car;
        if (str == "Ped") return TransportMode::Ped;
        if (str == "Taxi") return TransportMode::Taxi;
        if (str == "PublicTransport") return TransportMode::PublicTransport;
        throw std::runtime_error("Unrecognized transport mode");
    }

    inline std::string transportModeToString(const TransportMode &mode) {
        switch (mode) {
            case TransportMode::Car:
                return "Car";
            case TransportMode::Ped:
                return "Ped";
            case TransportMode::Taxi:
                return "Taxi";
            case TransportMode::PublicTransport:
                return "PublicTransport";
            case TransportMode::None:
                return "None";
            default:
                return "InvalidMode";
        }
    }
}
