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


template<typename T, template<typename> typename UnderlyingVectorT = std::vector>
class TimestampedVector {

public:

    using value_type = typename UnderlyingVectorT<T>::value_type;

    TimestampedVector(const size_t size, const T &INVALID_VAL) : INVALID_VAL(INVALID_VAL), clock(1) {
        resize(size);
    }

// Ensures that this container can hold the specified number of elements.
    void resize(const int size) {
        const auto currentSize = elements.size();
        if (size < currentSize) {
            elements.erase(elements.begin() + size, elements.end());
            timestamps.erase(timestamps.begin() + size, timestamps.end());
        } else {
            elements.insert(elements.end(), size - currentSize, INVALID_VAL);
            timestamps.insert(timestamps.end(), size - currentSize, 0);
        }
    }

    size_t size() const {
        return elements.size();
    }

    // Sets all elements to invalid.
    void clear() {
        ++clock;
        if (UNLIKELY(clock == 0)) {
            // Clock overflow occurred. Extremely unlikely.
            std::fill(timestamps.begin(), timestamps.end(), 0);
            clock = 1;
        }
    }

    // Returns a reference to the element at i.
    T &operator[](const int i) {
        assert(i >= 0);
        assert(i < elements.size());
        if (timestamps[i] != clock) {
            assert(timestamps[i] < clock);
            elements[i] = INVALID_VAL;
            timestamps[i] = clock;
        }
        return elements[i];
    }

    // Returns a const reference to the element at i.
    const T &operator[](const int i) const {
        assert(i >= 0);
        assert(i < elements.size());
        if (timestamps[i] != clock) {
            assert(timestamps[i] < clock);
            elements[i] = INVALID_VAL;
            timestamps[i] = clock;
        }
        return elements[i];
    }

    const T &at(const int i) const {
        return (*this)[i];
    }

    bool allInvalid() const {
        for (const auto &timestamp: timestamps) {
            assert(timestamp <= clock);
            if (timestamp == clock) return false;
        }
        return true;
    }

    bool hasValidValue(const int i) const {
        return timestamps[i] == clock;
    }

private:

    T INVALID_VAL;

    uint32_t clock;
    // mutable to allow lazy updates of elements even in const subscript operator (which is logically const for outside
    // users).
    mutable UnderlyingVectorT<T> elements;
    // mutable to allow lazy updates of elements even in const subscript operator (which is logically const for outside
    // users).
    mutable std::vector<uint32_t> timestamps;
};

