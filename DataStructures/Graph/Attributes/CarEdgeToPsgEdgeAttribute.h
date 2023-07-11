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

#include <cassert>

#include "DataStructures/Graph/Attributes/AbstractAttribute.h"
#include "Tools/Constants.h"

// An edge attribute mapping an edge in a car graph to an edge in a passenger graph.
// MULTIPLE PASSENGER EDGES MAY BE MAPPED TO THE SAME VEHICLE EDGE SO THIS ONLY RETURNS ONE
// (OUT OF POTENTIALLY MULTIPLE) PASSENGER EDGES!!!
class CarEdgeToPsgEdgeAttribute : public AbstractAttribute<int> {
public:
    // Returns the attribute's default value.
    static Type defaultValue() {
        return INVALID_ID;
    }

    // Returns the edge e' in the passenger graph that is equivalent to the given edge e in the car graph.
    const Type &toPsgEdge(const int e) const {
        assert(e >= 0);
        assert(e < values.size());
        return values[e];
    }

    // Returns a reference to the stored value e' that the given edge e in the car graph maps to.
    Type &toPsgEdge(const int e) {
        assert(e >= 0);
        assert(e < values.size());
        return values[e];
    }

protected:
    static constexpr const char *NAME = "car_edge_to_psg_edge"; // The attribute's unique name.
};
