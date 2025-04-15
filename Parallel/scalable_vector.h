/*******************************************************************************
 * MIT License
 *
 * This file is part of Mt-KaHyPar.
 *
 * Copyright (C) 2019 Lars Gottesbüren <lars.gottesbueren@kit.edu>
 * Copyright (C) 2019 Tobias Heuer <tobias.heuer@kit.edu>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#pragma once

#include <vector>
#include <tbb/scalable_allocator.h>
#include <tbb/cache_aligned_allocator.h>
#include "Tools/MachineSpecs.h"
#include "Tools/CompilerSpecific.h"

namespace parallel {
    template<typename T>
    using scalable_vector = std::vector<T, tbb::scalable_allocator<T> >;

    template<typename T>
    static inline void free(scalable_vector<T> &vec) {
        scalable_vector<T> tmp_vec;
        vec = std::move(tmp_vec);
    }

    // Scalable aligned vector can use the tbb::cache_aligned_allocator if the SIMD width is smaller than the cache
    // line size (which should be most cases). If the SIMD width is larger than the cache line size, we use our own
    // aligned allocator which may be slower in a parallel setting but ensures that SIMD vectors are aligned.
#if CACHE_LINE_SIZE >= MIN_ALIGNMENT
    template<typename T>
    using scalable_aligned_vector = std::vector<T, tbb::cache_aligned_allocator<T>>;
#else
    template<typename T>
    using scalable_aligned_vector = std::vector<T, AlignedAllocator<T>>;
#endif

}  // namespace parallel