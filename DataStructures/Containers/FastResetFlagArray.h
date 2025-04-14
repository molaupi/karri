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

template<typename Type_ = uint16_t>
class FastResetFlagArray {

    static_assert(std::is_unsigned_v<Type_>);

public:

    explicit FastResetFlagArray(const size_t size = 0) : clock(1) {
        resize(size);
    }

    // Ensures that this container can hold the specified number of flags.
    void resize(const int size) {
        const auto currentSize = timestamps.size();
        if (size < currentSize) {
            timestamps.erase(timestamps.begin() + size, timestamps.end());
        } else {
            timestamps.insert(timestamps.end(), size - currentSize, 0);
        }
    }

    size_t size() const {
        return timestamps.size();
    }

    // Resets all flags to false.
    void reset() {
        ++clock;
        if (UNLIKELY(clock == 0)) {
            // Clock overflow occurred. Extremely unlikely.
            std::fill(timestamps.begin(), timestamps.end(), 0);
            clock = 1;
        }
    }

    // Set flag i to true.
    void set(const int i) {
        assert(i < timestamps.size());
        timestamps[i] = clock;
    }

    // Set flag i to false.
    void unset(const int i) {
        assert(i < timestamps.size());
        timestamps[i] = 0;
    }

    bool isSet(const int i) const {
        assert(i < timestamps.size());
        return timestamps[i] == clock;
    }

private:

    Type_ clock;
    std::vector<Type_> timestamps;
};