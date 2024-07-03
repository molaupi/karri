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

#pragma once

#include "DataStructures/Utilities/IteratorRange.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"

namespace mixfix {


    class PickupDropoffInfo {

    public:
        PickupDropoffInfo(const int numVertices)
                : pickupIndex(numVertices, {0, 0}),
                  possiblePickups(),
                  pickupWalkingDists(),
                  dropoffIndex(numVertices, {0, 0}),
                  possibleDropoffs(),
                  dropoffWalkingDists() {}

        PickupDropoffInfo(std::ifstream &in) {
            bio::read(in, pickupIndex);
            bio::read(in, possiblePickups);
            bio::read(in, pickupWalkingDists);
            bio::read(in, dropoffIndex);
            bio::read(in, possibleDropoffs);
            bio::read(in, dropoffWalkingDists);
        }

        // Call defrag() first!
        void writeTo(std::ofstream &out) {
            defrag();
            bio::write(out, pickupIndex);
            bio::write(out, possiblePickups);
            bio::write(out, pickupWalkingDists);
            bio::write(out, dropoffIndex);
            bio::write(out, possibleDropoffs);
            bio::write(out, dropoffWalkingDists);
        }

        // Returns IDs of requests that may be picked up at vertex v.
        ConstantVectorRange<int> getPossiblePickupsAt(const int v) const {
            assert(v >= 0);
            assert(v < pickupIndex.size());
            const auto start = pickupIndex[v].start;
            const auto end = pickupIndex[v].end;
            return {possiblePickups.begin() + start, possiblePickups.begin() + end};
        }

        // Returns IDs of requests that may be picked up at vertex v.
        ConstantVectorRange<int> getPickupWalkingDistsAt(const int v) const {
            assert(v >= 0);
            assert(v < pickupIndex.size());
            const auto start = pickupIndex[v].start;
            const auto end = pickupIndex[v].end;
            return {pickupWalkingDists.begin() + start, pickupWalkingDists.begin() + end};
        }

        // Returns IDs of requests that may be dropped off at vertex v.
        ConstantVectorRange<int> getPossibleDropoffsAt(const int v) const {
            assert(v >= 0);
            assert(v < dropoffIndex.size());
            const auto start = dropoffIndex[v].start;
            const auto end = dropoffIndex[v].end;
            return {possibleDropoffs.begin() + start, possibleDropoffs.begin() + end};
        }

        // Returns IDs of requests that may be dropped off at vertex v.
        ConstantVectorRange<int> getDropoffWalkingDistsAt(const int v) const {
            assert(v >= 0);
            assert(v < dropoffIndex.size());
            const auto start = dropoffIndex[v].start;
            const auto end = dropoffIndex[v].end;
            return {dropoffWalkingDists.begin() + start, dropoffWalkingDists.begin() + end};
        }

        void addPickupAtVertex(const int requestId, const int walkingDist, const int v) {
            assert(v >= 0);
            assert(v < pickupIndex.size());
            const auto start = pickupIndex[v].start;
            const auto end = pickupIndex[v].end;

            // TODO: sort value arrays?
            // Do not add duplicates
            if (contains(possiblePickups.begin() + start, possiblePickups.begin() + end, requestId))
                return;

            const int idx = insertion(v, requestId, pickupIndex, possiblePickups, pickupWalkingDists);
            pickupWalkingDists[idx] = walkingDist;

            KASSERT(pickupIndex[v].start >= 0 && pickupIndex[v].end <= possiblePickups.size());
        }

        void addDropoffAtVertex(const int requestId, const int walkingDist, const int v) {
            assert(v >= 0);
            assert(v < dropoffIndex.size());
            const auto start = dropoffIndex[v].start;
            const auto end = dropoffIndex[v].end;

            // TODO: sort value arrays?
            // Do not add duplicates
            if (contains(possibleDropoffs.begin() + start, possibleDropoffs.begin() + end, requestId))
                return;

            const int idx = insertion(v, requestId, dropoffIndex, possibleDropoffs, dropoffWalkingDists);
            dropoffWalkingDists[idx] = walkingDist;

            KASSERT(dropoffIndex[v].start >= 0 && dropoffIndex[v].end <= possibleDropoffs.size());
        }

        // De-fragment 2D-arrays
        void defrag() {
            defragImpl(pickupIndex, possiblePickups, pickupWalkingDists);
            defragImpl(dropoffIndex, possibleDropoffs, dropoffWalkingDists);
        }

    private:

        void defragImpl(std::vector<ValueBlockPosition> &index, std::vector<int> &possiblePD,
                        std::vector<int> &walkingDists) {
            int totalPossiblePDs = 0;
            for (const auto &pos: index)
                totalPossiblePDs += pos.end - pos.start;

            std::vector<ValueBlockPosition> newIndex(index.size());
            std::vector<int> newPossiblePD;
            std::vector<int> newWalkingDists;
            newPossiblePD.reserve(totalPossiblePDs);
            newWalkingDists.reserve(totalPossiblePDs);

            for (int e = 0; e < index.size(); ++e) {
                newIndex[e].start = newPossiblePD.size();
                newPossiblePD.insert(newPossiblePD.end(), possiblePD.begin() + index[e].start,
                                     possiblePD.begin() + index[e].end);
                newWalkingDists.insert(newWalkingDists.end(), walkingDists.begin() + index[e].start,
                                       walkingDists.begin() + index[e].end);
                newIndex[e].end = newPossiblePD.size();
            }
            index = std::move(newIndex);
            possiblePD = std::move(newPossiblePD);
            walkingDists = std::move(newWalkingDists);
        }

        // We use two DynamicRagged2DArray to store the subset of requests that can be picked up/ dropped off at each
        // vertex. For both 2D-array, there is a single index array that for each vertex stores a range of indices where
        // the entries for that vertex are stored in a number of value arrays.

        // Pickup Index Array: For each vertex (of the vehicle graph) v, the according entries in the pickup value arrays
        // lie in the index interval [pickupIndex[v].start, pickupIndex[v].end).
        std::vector<ValueBlockPosition> pickupIndex;

        // IDs of requests that can be picked up at vertex.
        std::vector<int> possiblePickups;

        // IDs of requests that can be picked up at vertex.
        std::vector<int> pickupWalkingDists;

        // Dropoff Index Array: For each vertex (of the vehicle graph) v, the according entries in the dropoff value arrays
        // lie in the index interval [dropoffIndex[v].start, dropoffIndex[v].end).
        std::vector<ValueBlockPosition> dropoffIndex;

        // IDs of requests that can be dropped off at vertex.
        std::vector<int> possibleDropoffs;

        // IDs of requests that can be dropped off at vertex.
        std::vector<int> dropoffWalkingDists;
    };
}

