/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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

// Hints to the compiler that the specified condition is expected to be true.
#ifdef __GNUC__ // GNU compiler.
# define LIKELY(cond) __builtin_expect(!!(cond), 1)
#endif
#ifndef LIKELY  // Default definition.
# define LIKELY(cond) (cond)
#endif

// Hints to the compiler that the specified condition is expected to be false.
#ifdef __GNUC__  // GNU compiler.
# define UNLIKELY(cond) __builtin_expect(!!(cond), 0)
#endif
#ifndef UNLIKELY // Default definition.
# define UNLIKELY(cond) (cond)
#endif

// The weakest alignment a type shall have for use with SIMD instructions.
#if defined __AVX2__
constexpr int MIN_ALIGNMENT = 32;
#elif defined __SSE4_2__
constexpr int MIN_ALIGNMENT = 16;
#else
constexpr int MIN_ALIGNMENT = 1;
#endif
