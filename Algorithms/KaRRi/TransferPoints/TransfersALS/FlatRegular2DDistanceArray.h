/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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
#include <kassert/kassert.hpp>
#include "DataStructures/Utilities/IteratorRange.h"
#include "Tools/Constants.h"

namespace karri {

    struct FlatRegular2DDistanceArray {

        ConstantVectorRange<int> getDistancesFor(const int row) const {
            KASSERT(row >= 0 && row < curNumRows);
            return {distances.begin() + row * width, distances.begin() + (row + 1) * width};
        }

        int getMinDistanceFor(const int row) const {
            KASSERT(row >= 0 && row < curNumRows);
            return minDistancePerRow[row];
        }

    private:

        template<typename, typename, typename>
        friend class CHStrategyALS;

        template<typename, typename, typename, typename, typename>
        friend class PHASTStrategyALS;


        FlatRegular2DDistanceArray() : width(0), curNumRows(0), distances(), minDistancePerRow() {}

        // Initializes 2D distance array to be able to contain at least numRows rows and rowWidth distances per row.
        // No initialization of values takes place.
        void init(const size_t numRows, const size_t rowWidth) {
            width = rowWidth;
            curNumRows = numRows;
            const size_t newMinSize = numRows * rowWidth;
            KASSERT(newMinSize >= 0);
            std::fill(minDistancePerRow.begin(), minDistancePerRow.end(), INFTY);
            if (newMinSize > distances.size())
                distances.resize(newMinSize);
            if (numRows > minDistancePerRow.size())
                minDistancePerRow.resize(numRows, INFTY);
        }

        size_t width; // Number of distances per row
        size_t curNumRows;
        std::vector<int> distances;
        std::vector<int> minDistancePerRow;
    };
}