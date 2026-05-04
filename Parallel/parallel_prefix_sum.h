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

#include <numeric>
#include <vector>

#include <tbb/parallel_scan.h>

#include "scalable_vector.h"

namespace parallel {

    template<typename InIt, typename OutIt, typename BinOp, bool exclusive = false>
    struct ParallelPrefixSumBody {
        using T = typename ::std::iterator_traits<InIt>::value_type;

        InIt first;
        OutIt out;
        T sum, neutral_element;
        BinOp& f;

        ParallelPrefixSumBody(InIt first, OutIt out, T neutral_element, BinOp& f):
                first(first),
                out(out),
                sum(neutral_element),
                neutral_element(neutral_element),
                f(f) { }

        ParallelPrefixSumBody(ParallelPrefixSumBody& other, tbb::split) :
                first(other.first),
                out(other.out),
                sum(other.neutral_element),
                neutral_element(other.neutral_element),
                f(other.f) { }

        void operator()(const tbb::blocked_range<uint32_t>& r, tbb::pre_scan_tag ) {
            const auto rend = r.end();
            for (uint32_t i = r.begin(); i < rend; ++i) {
                sum = f(sum, *(first + i));
            }
        }

        void operator()(const tbb::blocked_range<uint32_t>& r, tbb::final_scan_tag ) {
            const auto rend = r.end();
            for (uint32_t i = r.begin(); i < rend; ++i) {
                if constexpr (exclusive) {
                    const auto tmp = *(first + i);
                    *(out + i) = sum;
                    sum = f(sum, tmp);
                } else {
                    sum = f(sum, *(first + i));
                    *(out + i) = sum;
                }
            }
        }

        void reverse_join(ParallelPrefixSumBody& other) {
            sum = f(sum, other.sum);
        }

        void assign(ParallelPrefixSumBody& other) {
            sum = other.sum;
        }

    };

    template <class InIt, class OutIt, class BinOp>
    static void sequential_inclusive_prefix_sum(InIt first, InIt last, OutIt d, BinOp f, typename std::iterator_traits<InIt>::value_type init) {
        while (first != last) {
            init = f(init, *first);
            *d = init;
            ++d;
            ++first;
        }
    }

    template <class InIt, class OutIt, class BinOp>
    static void parallel_inclusive_prefix_sum(InIt first, InIt last, OutIt d, BinOp f,
                                              typename std::iterator_traits<InIt>::value_type neutral_element) {

        typename std::iterator_traits<InIt>::difference_type n = last - first;

        if (n < (1 << 16)) {
            return sequential_inclusive_prefix_sum(first, last, d, f, neutral_element);
        }

        ParallelPrefixSumBody<InIt, OutIt, BinOp, false> body(first, d, neutral_element, f);
        tbb::parallel_scan(tbb::blocked_range<uint32_t>(0, static_cast<uint32_t>(n)), body);
    }

    template <class InIt, class OutIt, class BinOp>
    static void sequential_exclusive_prefix_sum(InIt first, InIt last, OutIt d, BinOp f, typename std::iterator_traits<InIt>::value_type init) {
        while (first != last) {
            const auto tmp = *first;
            *d = init;
            init = f(init, tmp);
            ++d;
            ++first;
        }
    }

    template <class InIt, class OutIt, class BinOp>
    static void parallel_exclusive_prefix_sum(InIt first, InIt last, OutIt d, BinOp f,
                                    typename std::iterator_traits<InIt>::value_type neutral_element) {

        typename std::iterator_traits<InIt>::difference_type n = last - first;

        if (n < (1 << 16)) {
            return sequential_exclusive_prefix_sum(first, last, d, f, neutral_element);
        }

        ParallelPrefixSumBody<InIt, OutIt, BinOp, true> body(first, d, neutral_element, f);
        tbb::parallel_scan(tbb::blocked_range<uint32_t>(0, static_cast<uint32_t>(n)), body);
    }
}