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

#include <iterator>
#include <cassert>

// Range of iterators that satisfies iterability and allows random access.
// User needs to make sure that begin and end are well-defined.
template<typename IteratorT>
class IteratorRange {
public:
    using ValueType = typename std::iterator_traits<IteratorT>::value_type;
    using Iterator = IteratorT;

    IteratorRange(const IteratorT startingPoint, const IteratorT endingPoint)
            : startingPoint(startingPoint), endingPoint(endingPoint) {}

    Iterator begin() const noexcept {
        return startingPoint;
    }

    Iterator end() const noexcept {
        return endingPoint;
    }

    const ValueType &operator[](const int col) const {
        assert(col >= 0);
        assert(col < endingPoint - startingPoint);
        return startingPoint[col];
    }

    size_t size() const noexcept {
        return endingPoint - startingPoint;
    }

private:
    IteratorT startingPoint;
    IteratorT endingPoint;
};


template<typename T>
class ConstantVectorRange : public IteratorRange<typename std::vector<T>::const_iterator> {
    using IteratorRange<typename std::vector<T>::const_iterator>::IteratorRange;
};

template<typename T>
class VariableVectorRange : public IteratorRange<typename std::vector<T>::const_iterator> {
public:
    using Base = IteratorRange<typename std::vector<T>::const_iterator>;
    using Iterator = typename Base::Iterator;
    using ValueType = typename std::iterator_traits<Iterator>::value_type;

    VariableVectorRange(Iterator startingPoint, Iterator endingPoint, std::vector<bool> chosen) : Base(startingPoint, endingPoint), chosen(chosen) {
        numOfEntries = 0;
        for(int i = 0; i < chosen.size(); i++) {
            if (chosen[i]) {
                numOfEntries++;
                indexArr.push_back(i);
            }
        }
    }

    VariableVectorRange(Iterator startingPoint, Iterator endingPoint) : Base(startingPoint, endingPoint) {
        usesArr = false;
    }

    const ValueType &operator[](const int col) const {
        assert(col >= 0);
        assert(col < this->end() - this->begin());
        return (usesArr ? this->begin()[indexArr[col]] : this->begin()[col]);
    }

    size_t size() const noexcept {
        return (usesArr ? numOfEntries : (this->end() - this->begin()));
    }

private:
    std::vector<int> indexArr;
    std::vector<bool> chosen;
    size_t numOfEntries;
    bool usesArr = true;
};