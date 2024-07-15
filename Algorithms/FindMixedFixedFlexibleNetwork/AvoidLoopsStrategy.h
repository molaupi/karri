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

#include "Tools/EnumParser.h"

namespace mixfix {
    enum class AvoidLoopsStrategy {
        NONE,
        VERTEX,
        EDGE,
        VERTEX_ROLLBACK,
        EDGE_ROLLBACK,
        EDGE_ALTERNATIVE
    };

} // end namespace mixfix

// Make EnumParser usable with LoopStrategy.
template<>
void EnumParser<mixfix::AvoidLoopsStrategy>::initNameToEnumMap() {
    nameToEnum = {
            {"none",                mixfix::AvoidLoopsStrategy::NONE},
            {"vertex",              mixfix::AvoidLoopsStrategy::VERTEX},
            {"edge",                mixfix::AvoidLoopsStrategy::EDGE},
            {"vertex-rollback",     mixfix::AvoidLoopsStrategy::VERTEX_ROLLBACK},
            {"edge-rollback",       mixfix::AvoidLoopsStrategy::EDGE_ROLLBACK},
            {"edge-alternative",    mixfix::AvoidLoopsStrategy::EDGE_ALTERNATIVE}
    };
}


