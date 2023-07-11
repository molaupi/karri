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
#include <cstdint>

#include "DataStructures/Graph/Attributes/AbstractAttribute.h"
#include "Tools/Constants.h"

// An attribute the OSM global ID of the OSM node associated with each vertex of a graph.
class OsmNodeIdAttribute : public AbstractAttribute<uint64_t> {
public:
    // Returns the attribute's default value.
    static Type defaultValue() {
        return std::numeric_limits<uint64_t>::max();
    }

    // Returns the OSM global ID of the OSM node that vertex v represents.
    const Type &osmNodeId(const int v) const {
        assert(v >= 0);
        assert(v < values.size());
        return values[v];
    }

    // Returns a reference to the OSM global ID of the OSM node that vertex v represents.
    Type &osmNodeId(const int v) {
        assert(v >= 0);
        assert(v < values.size());
        return values[v];
    }

protected:
    static constexpr const char *NAME = "osm_node_id"; // The attribute's unique name.
};
