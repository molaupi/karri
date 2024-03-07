/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


namespace karri {

    class VehLocToPickupDistances {


        static constexpr int unknownDist = INFTY + 1;

    public:
        VehLocToPickupDistances(const RequestState &requestState, const int fleetSize)
                : requestState(requestState), fleetSize(fleetSize), distances(),
                  vehiclesWithDistances(fleetSize), prevNumPickups(0), initializedForRequestId(INVALID_ID) {}

        void init() {

            // Already initialized for current request
            if (initializedForRequestId == requestState.originalRequest.requestId)
                return;
            initializedForRequestId = requestState.originalRequest.requestId;

            // clean up old result
            for (const auto &vehId: vehiclesWithDistances)
                for (int i = 0; i < prevNumPickups; ++i)
                    distances[vehId * prevNumPickups + i] = unknownDist;

            // initialize for new result
            if (fleetSize * requestState.numPickups() > distances.size())
                distances.resize(fleetSize * requestState.numPickups(), unknownDist);

            assert(std::all_of(distances.begin(), distances.end(), [&](const auto& d) {return d == unknownDist;}));
            prevNumPickups = requestState.numPickups();
        }

        void updateDistance(const int vehId, const int pickupId, const int newDist) {
            assert(vehId < fleetSize && pickupId < requestState.numPickups());
            auto &distAtomic = distances[vehId * requestState.numPickups() + pickupId];
            distAtomic = std::min(distAtomic, newDist);
            vehiclesWithDistances.insert(vehId);
        }

        bool knowsDistance(const int vehId, const unsigned int pickupId) {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < requestState.numPickups());
            return distances[vehId * requestState.numPickups() + pickupId] != unknownDist;
        }

        int getDistance(const int vehId, const unsigned int pickupId) {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < requestState.numPickups());
            return distances[vehId * requestState.numPickups() + pickupId];
        }

    private:

        const RequestState &requestState;
        const int fleetSize;

        // todo: these distances may need to be atomics since during the parallel DALS+PBNS enumeration, multiple threads may
        //  try to write and read the distance for the same vehicle and pickup at the same time.
        //  So far does not seem to have an effect though. Okay since every thread will always write the same distance?
        std::vector<int> distances;

        ThreadSafeSubset vehiclesWithDistances;
        int prevNumPickups;

        int initializedForRequestId;

    };

}