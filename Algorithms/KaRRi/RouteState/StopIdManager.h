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

#include <vector>
#include <stack>

// Manages stop ids, handing out unused IDs for new stops and accepting IDs of deleted stops back for reuse.
// Implemented as singleton to ensure uniqueness of stop IDs across the entire system.
class StopIdManager {

public:

    static int getUnusedStopId() {
        if (!theInstance().unusedStopIds.empty()) {
            const auto id = theInstance().unusedStopIds.top();
            theInstance().unusedStopIds.pop();
            return id;
        }
        return theInstance().nextUnusedStopId++;
    }

    static void markIdUnused(const int stopId) {
        theInstance().unusedStopIds.push(stopId);
    }

    static int getMaxStopId() {
        return theInstance().nextUnusedStopId - 1;
    }

private:

    static StopIdManager& theInstance()
    {
        static StopIdManager instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }

    StopIdManager() : unusedStopIds(), nextUnusedStopId(0) {}

    std::stack<int, std::vector<int>> unusedStopIds;
    int nextUnusedStopId;

};