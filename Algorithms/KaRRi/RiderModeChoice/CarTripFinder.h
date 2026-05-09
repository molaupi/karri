#pragma once
#include "CarResult.h"
#include "Tools/Constants.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Tools/ThreadSafeRandom.h"

namespace karri {

    // Computes a car trip for a given request, i.e., a trip that consists of only walking from origin to destination.
    class CarTripFinder {
        static bool isCarAllowed(const Request &req) {
            return req.allowPrivateCarProbability > 0.0 &&
                   ThreadSafeRandom::randomNumber() < req.allowPrivateCarProbability;
        }

    public:
        CarTripFinder() = default;

        CarResult findCarTrip(const RequestState &requestState) {
            if (!isCarAllowed(requestState.originalRequest))
                return {INFTY};
            return {requestState.originalReqDirectDist};
        }

    private:
    };
}
