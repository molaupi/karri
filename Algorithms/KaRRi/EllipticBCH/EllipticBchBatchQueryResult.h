/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "Tools/Constants.h"

// Stores the result of an elliptic BCH query for a single batch of PDLocs.
// The type of batch is defined by the LabelSetT template parameter.
template<typename LabelSetT>
struct EllipticBchBatchQueryResult {

    static constexpr int K = LabelSetT::K;
    using LabelMask = typename LabelSetT::LabelMask;
    using DistanceLabel = typename LabelSetT::DistanceLabel;

    struct LocalResultEntry {

        explicit LocalResultEntry(const int stopId) : stopId(stopId) {}

        int stopId = INVALID_ID;

        DistanceLabel distFromStopToPdLoc = INFTY;
        DistanceLabel meetingVertexFromStopToPdLoc = INVALID_VERTEX;
        DistanceLabel distFromPdLocToNextStop = INFTY;
        DistanceLabel meetingVertexFromPdLocToNextStop = INVALID_VERTEX;
    };

    EllipticBchBatchQueryResult()
            : indexInEntriesVector(),
              entries() {}

    void updateNumStops(const int newNumStops) {
        if (newNumStops > indexInEntriesVector.size())
            indexInEntriesVector.resize(newNumStops, INVALID_INDEX);
    }


    // Updates the distance from stop to the PD loc. Distance is written if there are
    // entries for the stop already or dynamic allocation of entries is allowed.
    // Returns mask indicating where the distance has been improved (all false if we don't know the stop and dynamic
    // allocation is not allowed).
    LabelMask updateDistanceFromStopToPDLoc(const int stopId,
                                            const DistanceLabel newDistToPDLoc,
                                            const int meetingVertex) {
        assert(stopId >= 0 && stopId < indexInEntriesVector.size());
        assert(newDistToPDLoc.horizontalMin() >= 0 && newDistToPDLoc.horizontalMin() < INFTY);

        // If no entries exist yet for this stop, perform the allocation.
        if (indexInEntriesVector[stopId] == INVALID_INDEX) {
            indexInEntriesVector[stopId] = entries.size();
            entries.push_back(LocalResultEntry(stopId));
        }

        const auto &idx = indexInEntriesVector[stopId];
        auto &entry = entries[idx];

        const LabelMask improvedLocal = newDistToPDLoc < entry.distFromStopToPdLoc;
        entry.distFromStopToPdLoc.setIf(newDistToPDLoc, improvedLocal);
        entry.meetingVertexFromStopToPdLoc.setIf(meetingVertex, improvedLocal);

        return improvedLocal;
    }

    // Updates the distance from the PD loc to the stop that follows stopId. Distance is written only if entries
    // for the stop exist already.
    // Returns mask indicating where the distance has been improved (all false if we don't know the stop).
    LabelMask updateDistanceFromPDLocToNextStop(const int stopId,
                                                const DistanceLabel newDistFromPdLocToNextStop,
                                                const int meetingVertex) {
        assert(stopId >= 0 && stopId < indexInEntriesVector.size());
        assert(newDistFromPdLocToNextStop.horizontalMin() >= 0 && newDistFromPdLocToNextStop.horizontalMin() < INFTY);

        // We assume the same thread runs the to search and then the from search for a PDLoc. If the to search
        // did not find a result for this stop, we do not need to consider it in the from search.
        if (indexInEntriesVector[stopId] == INVALID_INDEX)
            return LabelMask(false);

        const auto &idx = indexInEntriesVector[stopId];
        auto &entry = entries[idx];

        const LabelMask improvedLocal = newDistFromPdLocToNextStop < entry.distFromPdLocToNextStop;
        entry.distFromPdLocToNextStop.setIf(newDistFromPdLocToNextStop, improvedLocal);
        entry.meetingVertexFromPdLocToNextStop.setIf(meetingVertex, improvedLocal);

        return improvedLocal;
    }

    std::vector<int> indexInEntriesVector;
    std::vector<LocalResultEntry> entries;

};

