
#pragma once

#include "Tools/Constants.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"

namespace karri {

    struct TransferPoint {
        TransferPoint() {}
        TransferPoint(const int loc, const Vehicle *pVeh, const Vehicle *dVeh) : loc(loc), pVeh(pVeh), dVeh(dVeh) {}
        TransferPoint(const Vehicle *pVeh, const Vehicle *dVeh) : pVeh(pVeh), dVeh(dVeh) {}
        TransferPoint(const TransferPoint&) = default;

        TransferPoint(
            const int loc,
            const Vehicle *pVeh, const Vehicle *dVeh,
            const int pVehIdx, const int dVehIdx,
            const int distanceToPveh, const int distanceFromPVeh,
            const int distanceToDVeh, const int distanceFromDVeh)
          : loc(loc),
            pVeh(pVeh),
            dVeh(dVeh),
            stopIdxDVeh(dVehIdx),
            stopIdxPVeh(pVehIdx),
            distancePVehToTransfer(distanceToPveh),
            distancePVehFromTransfer(distanceFromPVeh),
            distanceDVehToTransfer(distanceToDVeh),
            distanceDVehFromTransfer(distanceFromDVeh) {}

        int loc = INVALID_EDGE; // Location in the road network
        
        const Vehicle *pVeh = nullptr;
        const Vehicle *dVeh = nullptr;

        int stopIdxDVeh = INVALID_INDEX;
        int stopIdxPVeh = INVALID_INDEX;
        
        int distancePVehToTransfer = -1;
        int distancePVehFromTransfer = -1;
        int distanceDVehToTransfer = -1;
        int distanceDVehFromTransfer = -1;

    };

}