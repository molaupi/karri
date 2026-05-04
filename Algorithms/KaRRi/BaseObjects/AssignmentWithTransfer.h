/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Johannes Breitling <johannes.breitling@student.kit.edu>
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


#pragma once

#include <cassert>

#include "Algorithms/KaRRi/TransferPoints/TransferPoint.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/RequestCost.h"
#include "Algorithms/KaRRi/BaseObjects/PD.h"

namespace karri {

    enum INS_TYPES {
        NOT_SET = 0,
        BEFORE_NEXT_STOP = 1,
        ORDINARY = 2,
        AFTER_LAST_STOP = 3
    };

    struct AssignmentWithTransfer {

        AssignmentWithTransfer() {}

        AssignmentWithTransfer(const Vehicle *pVehArg, const Vehicle *dVehArg, const TransferPoint tpArg) {
            pVeh = pVehArg;
            dVeh = dVehArg;
            transfer = tpArg;
            distToTransferPVeh = tpArg.distancePVehToTransfer;
            distFromTransferPVeh = tpArg.distancePVehFromTransfer;
            distToTransferDVeh = tpArg.distanceDVehToTransfer;
            distFromTransferDVeh = tpArg.distanceDVehFromTransfer;
        }

        AssignmentWithTransfer(const Vehicle &pVehArg, const Vehicle &dVehArg, const TransferPoint tpArg) {
            pVeh = &pVehArg;
            dVeh = &dVehArg;
            transfer = tpArg;
            distToTransferPVeh = tpArg.distancePVehToTransfer;
            distFromTransferPVeh = tpArg.distancePVehFromTransfer;
            distToTransferDVeh = tpArg.distanceDVehToTransfer;
            distFromTransferDVeh = tpArg.distanceDVehFromTransfer;
        }

        AssignmentWithTransfer(const Vehicle *pVehArg, const Vehicle *dVehArg, const TransferPoint tpArg, const PDLoc *pickupPDLoc, int pickupIdxArg, int distToPickupArg, int distFromPickupArg, int tIdxPVeh, int tIdxDVeh) {
            pVeh = pVehArg;
            dVeh = dVehArg;
            transfer = tpArg;
            pickupIdx = pickupIdxArg;
            transferIdxPVeh = tIdxPVeh;
            transferIdxDVeh = tIdxDVeh;

            pickup = pickupPDLoc;
            distToPickup = distToPickupArg;
            distFromPickup = distFromPickupArg;
            distToTransferPVeh = tpArg.distancePVehToTransfer;
            distFromTransferPVeh = tpArg.distancePVehFromTransfer;
            distToTransferDVeh = tpArg.distanceDVehToTransfer;
            distFromTransferDVeh = tpArg.distanceDVehFromTransfer;
        }
        
        AssignmentWithTransfer(const Vehicle &pVehArg, const Vehicle &dVehArg, const TransferPoint tpArg, const PDLoc *pickupPDLoc, int pickupIdxArg, int distToPickupArg, int distFromPickupArg, int tIdxPVeh, int tIdxDVeh) {
            pVeh = &pVehArg;
            dVeh = &dVehArg;
            transfer = tpArg;
            pickupIdx = pickupIdxArg;
            transferIdxPVeh = tIdxPVeh;
            transferIdxDVeh = tIdxDVeh;

            pickup = pickupPDLoc;
            distToPickup = distToPickupArg;
            distFromPickup = distFromPickupArg;
            distToTransferPVeh = tpArg.distancePVehToTransfer;
            distFromTransferPVeh = tpArg.distancePVehFromTransfer;
            distToTransferDVeh = tpArg.distanceDVehToTransfer;
            distFromTransferDVeh = tpArg.distanceDVehFromTransfer;
        }


        bool isFinished() const {
            return !pickupBNSLowerBoundUsed && !pickupPairedLowerBoundUsed && !dropoffBNSLowerBoundUsed && !dropoffPairedLowerBoundUsed;
        }

        const Vehicle *pVeh = nullptr;
        const Vehicle *dVeh = nullptr;

        const PDLoc *pickup = nullptr;
        TransferPoint transfer;
        const PDLoc *dropoff = nullptr;

        int pickupIdx = INVALID_INDEX;
        int transferIdxPVeh = INVALID_INDEX;
        int transferIdxDVeh = INVALID_INDEX;
        int dropoffIdx = INVALID_INDEX;

        bool pickupBNSLowerBoundUsed = false;
        bool pickupPairedLowerBoundUsed = false;
        bool dropoffBNSLowerBoundUsed = false;
        bool dropoffPairedLowerBoundUsed = false;
        
        int distToPickup = 0; // distance from previous stop to pickup
        int distFromPickup = 0; // distance from pickup to next stop (or 0 if pickupIdx == transferIdxPVeh)
        int distToTransferPVeh = 0; // distance from previous stop to transfer point (or from transfer point if pickupIdx == transferIdxPVeh)
        int distFromTransferPVeh = 0; // distance from transfer point to next stop (or 0 if there is no next stop)
        
        int distToTransferDVeh = 0; // distance from previous stop to transfer point
        int distFromTransferDVeh = 0; // distance from transfer point to next stop (or 0 if transferIdxDVeh == dropoffIdx)
        int distToDropoff = 0; // distance from previous stop to dropoff (or from transfer point if transferIdxDVeh == dropoffIdx)
        int distFromDropoff = 0; // distance from dropoff to next stop (or 0 if there is no next stop)

        int waitTimeAtPickup; // Wait time at pickup

        // For statistics
        enum INS_TYPES pickupType = NOT_SET;
        enum INS_TYPES transferTypePVeh = NOT_SET;

        enum INS_TYPES transferTypeDVeh = NOT_SET;
        enum INS_TYPES dropoffType = NOT_SET;
    };

}