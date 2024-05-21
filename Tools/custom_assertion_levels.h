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

#include <kassert/kassert.hpp>

namespace kassert::assert {
#define KASSERT_ASSERTION_LEVEL_LIGHT 20
    constexpr int light = KASSERT_ASSERTION_LEVEL_LIGHT;

#define LIGHT_KASSERT_NO_MSG(cond) KASSERT(cond, "", kassert::assert::light)
#define LIGHT_KASSERT_MSG(cond, msg) KASSERT(cond, msg, kassert::assert::light)
#define LIGHT_KASSERT(...)                  \
    KASSERT_KASSERT_HPP_VARARG_HELPER_2(    \
        ,                                   \
        __VA_ARGS__,                        \
        LIGHT_KASSERT_MSG(__VA_ARGS__),     \
        LIGHT_KASSERT_NO_MSG(__VA_ARGS__),  \
        dummy                               \
    )


#define KASSERT_ASSERTION_LEVEL_HEAVY 40
    constexpr int heavy = KASSERT_ASSERTION_LEVEL_HEAVY;

#define HEAVY_KASSERT_NO_MSG(cond) KASSERT(cond, "", kassert::assert::heavy)
#define HEAVY_KASSERT_MSG(cond, msg) KASSERT(cond, msg, kassert::assert::heavy)
#define HEAVY_KASSERT(...)                  \
    KASSERT_KASSERT_HPP_VARARG_HELPER_2(    \
        ,                                   \
        __VA_ARGS__,                        \
        HEAVY_KASSERT_MSG(__VA_ARGS__),     \
        HEAVY_KASSERT_NO_MSG(__VA_ARGS__),  \
        dummy                               \
    )

} // namespace kassert::assert