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
                    distances[vehId * prevNumPickups + i].store(unknownDist, std::memory_order_seq_cst);

            // initialize for new result
            if (fleetSize * requestState.numPickups() > distances.size())
                distances.resize(fleetSize * requestState.numPickups(), CAtomic<int>(unknownDist));

            assert(std::all_of(distances.begin(), distances.end(), [&](const auto& d) {return d == unknownDist;}));
            prevNumPickups = requestState.numPickups();
        }

        void updateDistance(const int vehId, const int pickupId, const int newDist) {
            assert(vehId < fleetSize && pickupId < requestState.numPickups());
            auto &distAtomic = distances[vehId * requestState.numPickups() + pickupId];

            int expected = unknownDist;
            distAtomic.compare_exchange_strong(expected, newDist, std::memory_order_relaxed);
            assert(distAtomic.load(std::memory_order_relaxed) == newDist);
            vehiclesWithDistances.insert(vehId);
        }

        bool knowsDistance(const int vehId, const unsigned int pickupId) {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < requestState.numPickups());
            return distances[vehId * requestState.numPickups() + pickupId].load(std::memory_order_relaxed) != unknownDist;
        }

        int getDistance(const int vehId, const unsigned int pickupId) {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < requestState.numPickups());
            return distances[vehId * requestState.numPickups() + pickupId].load(std::memory_order_relaxed);
        }

    private:

        const RequestState &requestState;
        const int fleetSize;

        std::vector<CAtomic<int>> distances;

        ThreadSafeSubset vehiclesWithDistances;
        int prevNumPickups;

        int initializedForRequestId;

    };

}