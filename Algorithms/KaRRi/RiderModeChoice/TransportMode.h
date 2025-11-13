#pragma once

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
}