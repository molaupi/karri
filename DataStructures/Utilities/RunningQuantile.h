/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2026 Moritz Laupichler <moritz.laupichler@kit.edu>
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
#include <cmath>
#include <queue>
#include <type_traits>
#include <vector>

// Maintains the q-quantile of a multiset of numerical values to which values may only be added
// (never removed), using two priority queues. This generalizes the classic two-heap technique
// for a running median: a max-heap holds the smallest ceil(q * n) values added so far (n being
// the number of values added so far), while a min-heap holds the remaining, larger values. The
// current q-quantile is always the top of the max-heap. Adding a value and querying the
// quantile both run in O(log n) and O(1) time, respectively.
template<double q, typename T = int>
class RunningQuantile {

    static_assert(q >= 0.0 && q <= 1.0, "q must be a quantile between 0 and 1.");

public:

    RunningQuantile() : lowerHeap(), upperHeap(), n(0) {}

    // Adds a new value to the set.
    void addValue(const T value) {
        if (lowerHeap.empty() || value <= lowerHeap.top())
            lowerHeap.push(value);
        else
            upperHeap.push(value);
        ++n;
        rebalance();
    }

    // Returns the current q-quantile of the values added so far. May only be called after at
    // least one value has been added.
    T getQuantile() const {
        assert(n > 0);
        return lowerHeap.empty() ? upperHeap.top() : lowerHeap.top();
    }

    // Returns the number of values added so far.
    int size() const noexcept {
        return n;
    }

private:

    // Moves the top element from one heap to the other until the lower heap holds exactly the
    // smallest ceil(q * n) values added so far.
    void rebalance() {
        // Subtracting a small epsilon before rounding up guards against q * n overshooting the
        // nearest integer due to floating-point rounding (e.g. q == 0.9 and n == 10).
        const int targetLowerSize = static_cast<int>(std::ceil(q * n - 1e-9));

        while (lowerHeap.size() > targetLowerSize) {
            upperHeap.push(lowerHeap.top());
            lowerHeap.pop();
        }
        while (lowerHeap.size() < targetLowerSize) {
            lowerHeap.push(upperHeap.top());
            upperHeap.pop();
        }
    }

    std::priority_queue<T, std::vector<T>, std::less<T>> lowerHeap;    // Max-heap of the smaller values.
    std::priority_queue<T, std::vector<T>, std::greater<T>> upperHeap; // Min-heap of the larger values.
    int n; // Number of values added so far.
};
